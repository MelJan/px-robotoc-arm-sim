const MODES = new Set(['simulation', 'connected-read-only', 'operational-disarmed', 'operational-armed', 'fault-stopped']);

export class RobotArmClient extends EventTarget {
  constructor({ apiBase = '/api', fetchImpl = fetch } = {}) {
    super();
    this.apiBase = apiBase.replace(/\/$/, '');
    this.fetchImpl = (...args) => fetchImpl(...args);
    this.mode = 'simulation';
    this.state = null;
  }

  setMode(mode) {
    if (!MODES.has(mode)) throw new Error(`Unknown operating mode: ${mode}`);
    this.mode = mode;
    this.dispatchEvent(new CustomEvent('modechange', { detail: { mode, state: this.state } }));
  }

  async request(path, options = {}) {
    const response = await this.fetchImpl(`${this.apiBase}/${path}`, { cache: 'no-store', ...options });
    const data = await response.json().catch(() => ({}));
    if (!response.ok) throw new Error(data.error || `${path} failed (${response.status})`);
    return data;
  }

  async connectReadOnly() {
    try {
      this.state = await this.request('state');
      if (!this.state || !Array.isArray(this.state.joints) || typeof this.state.mode !== 'string' || typeof this.state.commands !== 'object') {
        throw new Error('The same-origin endpoint did not return a robot-arm state document');
      }
      this.setMode('connected-read-only');
      return this.state;
    } catch (error) {
      this.setMode('fault-stopped');
      throw new Error(`Read-only connection failed. The simulator must be served from the robot API's same origin. ${error.message}`);
    }
  }

  enterOperationalDisarmed() {
    if (this.mode !== 'connected-read-only') throw new Error('Connect read-only before selecting operational mode');
    this.setMode('operational-disarmed');
  }

  async post(path, body = {}) {
    return this.request(path, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  }

  async arm({ parkConfirmed = false } = {}) {
    if (this.mode !== 'operational-disarmed') throw new Error('Operational mode must be selected before arming');
    if (!parkConfirmed) throw new Error('Fresh physical supported PARK confirmation is required');
    await this.post('power_on', { park_confirmed: true });
    await this.waitForIdle();
    if (!this.state?.armed) throw new Error(this.state?.error || 'Backend did not report an armed state');
    this.setMode('operational-armed');
  }

  async refresh() { this.state = await this.request('state'); return this.state; }

  async waitForIdle(timeoutMs = 60000) {
    const deadline = Date.now() + timeoutMs;
    while (Date.now() < deadline) {
      await this.refresh();
      if (this.state.error) throw new Error(this.state.error);
      if (!this.state.busy) return this.state;
      await new Promise((resolve) => setTimeout(resolve, 200));
    }
    throw new Error('Robot API did not become idle before the timeout');
  }

  async executeSingleJointPath(spec, path) {
    if (this.mode !== 'operational-armed') throw new Error('Explicit operational arming is required');
    await this.refresh();
    const commanded = this.state?.commands ?? {};
    for (const joint of spec.joints) {
      const channel = Number(joint.id.slice(1)) - 1;
      if (!Number.isInteger(commanded[channel]) || commanded[channel] !== path[0][joint.id]) {
        throw new Error('Planned start does not match the backend commanded pose; re-plan from the connected state');
      }
    }
    for (let index = 1; index < path.length; index += 1) {
      const changed = spec.joints.filter((joint) => path[index][joint.id] !== path[index - 1][joint.id]);
      if (changed.length !== 1) throw new Error('Operational playback accepts only verified single-servo steps');
      const joint = changed[0];
      await this.post('move', { channel: Number(joint.id.slice(1)) - 1, target: path[index][joint.id], first_clear: false });
      await this.waitForIdle();
    }
  }

  async powerOff() {
    if (!['operational-armed', 'operational-disarmed', 'connected-read-only'].includes(this.mode)) return;
    await this.post('power_off');
    await this.waitForIdle();
    this.setMode('connected-read-only');
  }

  async emergencyStop() {
    try { await this.post('stop'); } finally { this.setMode('fault-stopped'); }
  }

  returnToSimulation() { this.state = null; this.setMode('simulation'); }
}

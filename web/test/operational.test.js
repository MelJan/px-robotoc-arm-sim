import test from 'node:test';
import assert from 'node:assert/strict';
import { RobotArmClient, validateRobotState } from '../src/operational/client.js';

const home = { S1: 320, S2: 427, S3: 403, S4: 220, S5: 480, S6: 650 };
const commands = () => Object.fromEntries(Object.values(home).map((value, channel) => [channel, value]));
const joints = () => Object.keys(home).map((_id, channel) => ({ channel, low: channel === 1 ? 185 : channel === 5 ? 490 : 90, high: 660, allowed: true }));
const robotState = (overrides = {}) => ({
  mode: 'preview', armed: false, busy: false, outputs_off: true, error: null,
  joints: joints(), commands: commands(), ...overrides,
});
const json = (data, status = 200) => new Response(JSON.stringify(data), { status, headers: { 'Content-Type': 'application/json' } });
const spec = { joints: Object.keys(home).map((id) => ({ id })) };

test('read-only connection performs no POST and cannot execute motion', async () => {
  const calls = [];
  const client = new RobotArmClient({ fetchImpl: async (url, options = {}) => {
    calls.push({ url, method: options.method ?? 'GET' });
    return json(robotState());
  } });
  await client.connectReadOnly();
  assert.equal(client.mode, 'connected-read-only');
  assert.deepEqual(calls.map((call) => call.method), ['GET']);
  await assert.rejects(() => client.executeSingleJointPath({ joints: [] }, [{}]), /arming is required/i);
});

test('operational mode requires explicit PARK confirmation before power_on', async () => {
  const client = new RobotArmClient({ fetchImpl: async () => json(robotState()) });
  await client.connectReadOnly(); client.enterOperationalDisarmed();
  await assert.rejects(() => client.arm({ parkConfirmed: false }), /PARK confirmation/);
});

test('state validation rejects incomplete, duplicated and non-integer robot data', () => {
  assert.throws(() => validateRobotState({}), /identify preview/);
  assert.throws(() => validateRobotState(robotState({ joints: joints().slice(0, 5) })), /all six joints/);
  assert.throws(() => validateRobotState(robotState({ joints: joints().map((joint) => ({ ...joint, channel: 0 })) })), /exactly 0 through 5/);
  assert.throws(() => validateRobotState(robotState({ joints: joints().map((joint, index) => index ? joint : { ...joint, low: null }) })), /bounds are invalid/);
  assert.throws(() => validateRobotState(robotState({ commands: { ...commands(), 2: 403.5 } })), /integer or null/);
});

test('non-JSON and malformed same-origin endpoints fail closed', async () => {
  const htmlClient = new RobotArmClient({ fetchImpl: async () => new Response('<html>Vite fallback</html>') });
  await assert.rejects(() => htmlClient.connectReadOnly(), /non-JSON response/);
  assert.equal(htmlClient.mode, 'fault-stopped');
  const malformedClient = new RobotArmClient({ fetchImpl: async () => json({ mode: 'preview' }) });
  await assert.rejects(() => malformedClient.connectReadOnly(), /all six joints/);
  assert.equal(malformedClient.mode, 'fault-stopped');
});

test('arming refreshes state and requires backend confirmation', async () => {
  let backend = robotState();
  const calls = [];
  const client = new RobotArmClient({ fetchImpl: async (url, options = {}) => {
    const path = new URL(url, 'http://sim').pathname;
    calls.push({ path, method: options.method ?? 'GET' });
    if (path.endsWith('/power_on')) { backend = robotState({ armed: true, outputs_off: false }); return json({ ok: true }); }
    return json(backend);
  } });
  await client.connectReadOnly();
  client.enterOperationalDisarmed();
  await client.arm({ parkConfirmed: true });
  assert.equal(client.mode, 'operational-armed');
  assert.deepEqual(calls.map((call) => `${call.method} ${call.path}`), ['GET /api/state', 'GET /api/state', 'POST /api/power_on', 'GET /api/state']);
});

test('operational playback accepts one-servo steps and confirms each command', async () => {
  let backend = robotState({ armed: true, outputs_off: false });
  const posts = [];
  const client = new RobotArmClient({ fetchImpl: async (url, options = {}) => {
    const path = new URL(url, 'http://sim').pathname;
    if (path.endsWith('/move')) {
      const body = JSON.parse(options.body); posts.push(body);
      backend = robotState({ armed: true, outputs_off: false, commands: { ...backend.commands, [body.channel]: body.target } });
      return json({ ok: true });
    }
    return json(backend);
  } });
  client.setState(backend); client.setMode('operational-armed');
  await client.executeSingleJointPath(spec, [home, { ...home, S1: 321 }, { ...home, S1: 321, S2: 428 }]);
  assert.deepEqual(posts, [
    { channel: 0, target: 321, first_clear: false },
    { channel: 1, target: 428, first_clear: false },
  ]);
});

test('operational playback rejects unsafe path shape and backend bounds before POST', async () => {
  const calls = [];
  const backend = robotState({ armed: true, outputs_off: false });
  const client = new RobotArmClient({ fetchImpl: async (_url, options = {}) => { calls.push(options.method ?? 'GET'); return json(backend); } });
  client.setState(backend); client.setMode('operational-armed');
  await assert.rejects(() => client.executeSingleJointPath(spec, [home, { ...home, S1: 321, S2: 428 }]), /single-servo/);
  await assert.rejects(() => client.executeSingleJointPath(spec, [home, { ...home, S1: 700 }]), /outside backend session bounds/);
  assert.equal(calls.includes('POST'), false);
});

test('a failed move attempts stop and enters fault-stopped mode', async () => {
  const backend = robotState({ armed: true, outputs_off: false });
  const posts = [];
  const client = new RobotArmClient({ fetchImpl: async (url, options = {}) => {
    const path = new URL(url, 'http://sim').pathname;
    if (options.method === 'POST') posts.push(path);
    if (path.endsWith('/move')) return json({ error: 'driver fault' }, 500);
    if (path.endsWith('/stop')) return json({ ok: true });
    return json(backend);
  } });
  client.setState(backend); client.setMode('operational-armed');
  await assert.rejects(() => client.executeSingleJointPath(spec, [home, { ...home, S1: 321 }]), /Operational playback stopped/);
  assert.deepEqual(posts, ['/api/move', '/api/stop']);
  assert.equal(client.mode, 'fault-stopped');
});

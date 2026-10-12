import test from 'node:test';
import assert from 'node:assert/strict';
import { RobotArmClient } from '../src/operational/client.js';

const state = { mode: 'preview', armed: false, busy: false, joints: [{ channel: 0 }], commands: { 0: 320, 1: 427, 2: 403, 3: 220, 4: 480, 5: 650 } };

test('read-only connection performs no POST and cannot execute motion', async () => {
  const calls = [];
  const client = new RobotArmClient({ fetchImpl: async (url, options = {}) => {
    calls.push({ url, method: options.method ?? 'GET' });
    return new Response(JSON.stringify(state), { status: 200, headers: { 'Content-Type': 'application/json' } });
  } });
  await client.connectReadOnly();
  assert.equal(client.mode, 'connected-read-only');
  assert.deepEqual(calls.map((call) => call.method), ['GET']);
  await assert.rejects(() => client.executeSingleJointPath({ joints: [] }, [{}]), /arming is required/i);
});

test('operational mode requires explicit PARK confirmation before power_on', async () => {
  const client = new RobotArmClient({ fetchImpl: async () => new Response(JSON.stringify(state), { status: 200, headers: { 'Content-Type': 'application/json' } }) });
  await client.connectReadOnly(); client.enterOperationalDisarmed();
  await assert.rejects(() => client.arm({ parkConfirmed: false }), /PARK confirmation/);
});

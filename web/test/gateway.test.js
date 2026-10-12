import test from 'node:test';
import assert from 'node:assert/strict';
import { connectionInfo, proxyDecision } from '../src/operational/gateway.js';
import { createGateway } from '../server.mjs';

test('gateway is read-only plus emergency stop by default', () => {
  assert.deepEqual(proxyDecision('GET', '/api/state'), { allowed: true, kind: 'read' });
  assert.deepEqual(proxyDecision('POST', '/api/stop'), { allowed: true, kind: 'stop' });
  assert.equal(proxyDecision('POST', '/api/move').allowed, false);
  assert.equal(proxyDecision('POST', '/api/power_on').status, 403);
});

test('explicit motion mode exposes only required operational endpoints', () => {
  for (const path of ['/api/power_on', '/api/power_off', '/api/move']) {
    assert.deepEqual(proxyDecision('POST', path, { allowMotion: true }), { allowed: true, kind: 'motion' });
  }
  assert.equal(proxyDecision('POST', '/api/limits', { allowMotion: true }).allowed, false);
  assert.equal(proxyDecision('GET', '/api/openapi.json', { allowMotion: true }).allowed, false);
});

test('connection metadata never implies motion without both controller and flag', () => {
  assert.deepEqual(connectionInfo(null, true), {
    backend: 'none', controllerConfigured: false, motionAllowed: false, policy: 'simulation-only',
  });
  assert.equal(connectionInfo(new URL('http://robot.local:8765'), false).policy, 'read-only-plus-stop');
  assert.equal(connectionInfo(new URL('http://robot.local:8765'), true).motionAllowed, true);
});

test('gateway construction has no listening or controller side effect', () => {
  const gateway = createGateway({ controller: new URL('http://robot.local:8765'), allowMotion: false });
  assert.equal(gateway.listening, false);
  gateway.close();
});

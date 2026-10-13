import test from 'node:test';
import assert from 'node:assert/strict';
import { SERVOS } from '../js/config.js';
import { World } from '../js/world.js';
import { planTransfer, parseCommand, verify } from '../js/planner.js';
import { applyCanonicalServoMetadata, configureCanonicalSpec } from '../js/advanced.js';
import { loadSpec } from './helpers.js';

test('a pick-and-place return uses a clearance transit before HOME', () => {
  const spec = configureCanonicalSpec(loadSpec());
  const servos = structuredClone(SERVOS);
  applyCanonicalServoMetadata(servos, spec);
  const world = new World(servos);
  for (const [id, x, y, level, yaw] of [
    ['orange', 10.8, 14, 0, 3], ['blue', -8, 12, 0, 0],
    ['red', 3, 10, 0, 0], ['green', -4, 17, 0, 10],
  ]) world.addCube(id, id, x, y, level, yaw);
  const home = servos.map((servo) => servo.home);
  const plan = planTransfer(world, parseCommand('red on blue', world), home, { servos, home });
  const homeIndex = plan.steps.findIndex((step) => step.label === 'HOME');
  assert.ok(homeIndex > 0);
  assert.match(plan.steps[homeIndex - 1].label, /^Transit/);
  assert.equal(verify(world, plan, home, { servos }).ok, true);
});

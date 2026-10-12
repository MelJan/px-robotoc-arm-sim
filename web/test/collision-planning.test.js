import test from 'node:test';
import assert from 'node:assert/strict';
import { collisionReport, verifyPath } from '../src/core/collision.js';
import { referencePose } from '../src/core/kinematics.js';
import { controllerCompatiblePath, parseTextTask, planReferenceTransition, planTextTask } from '../src/core/planning.js';
import { loadSpec } from './helpers.js';

test('HOME and supported PARK are valid, including a verified optimized transition', () => {
  const spec = loadSpec(); const home = referencePose(spec, 'home'); const park = referencePose(spec, 'park');
  assert.equal(collisionReport(spec, home, { ignoreCubes: true }).valid, true);
  const parkReport = collisionReport(spec, park, { ignoreCubes: true });
  assert.equal(parkReport.valid, true);
  assert.equal(parkReport.minimumClearance, 0);
  const plan = planReferenceTransition(spec, home, 'park');
  assert.equal(plan.success, true, plan.reason);
  assert.equal(verifyPath(spec, plan.path, { ignoreCubes: true }).valid, true);
});

test('controller-compatible playback changes one servo per waypoint', () => {
  const spec = loadSpec(); const plan = planReferenceTransition(spec, referencePose(spec, 'home'), 'park');
  const converted = controllerCompatiblePath(spec, plan.path);
  assert.equal(converted.success, true, converted.reason);
  for (let index = 1; index < converted.path.length; index += 1) {
    const changed = spec.joints.filter((joint) => converted.path[index][joint.id] !== converted.path[index - 1][joint.id]);
    assert.equal(changed.length, 1);
  }
});

test('text task parser accepts color zones and explicit millimetre coordinates', () => {
  const spec = loadSpec();
  const colored = parseTextTask('Move the red cube to the blue zone', spec);
  assert.equal(colored.cube, 'red');
  assert.deepEqual(colored.destination, spec.world.dropZones.blue);
  assert.deepEqual(parseTextTask('move orange to (80, 240) mm', spec).destination.slice(0, 2), [80, 240]);
  assert.throws(() => parseTextTask('move a cube somewhere', spec), /Name a cube/);
});

test('pick/place task verifies approach, carried-cube and placed-cube phases', () => {
  const spec = loadSpec();
  const plan = planTextTask(spec, referencePose(spec, 'home'), 'Move the red cube to the blue zone');
  assert.equal(plan.success, true, plan.reason);
  assert.equal(plan.verification.phases.length, 8);
  assert.ok(plan.verification.phases.every((phase) => phase.valid));
  assert.deepEqual(plan.events.map((event) => event.type), ['grasp', 'allow-place-support', 'release']);
});

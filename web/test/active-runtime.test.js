import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { fileURLToPath } from 'node:url';
import { applyCanonicalServoMetadata, configureCanonicalSpec, runtimeSpec } from '../js/advanced.js';
import { SERVOS } from '../js/config.js';
import { anglesFromCounts, fk } from '../js/kinematics.js';
import { forwardKinematics, referencePose } from '../src/core/kinematics.js';
import { loadSpec } from './helpers.js';

const webFile = (relative) => fs.readFileSync(fileURLToPath(new URL(`../${relative}`, import.meta.url)), 'utf8');

test('active preserved UI is wired to the canonical planning, Rapier and operational layers', () => {
  const index = webFile('index.html');
  const app = webFile('js/app.js');
  assert.match(index, /src="js\/app\.js"/);
  assert.match(index, /id="task-preset"/);
  assert.match(index, /Move the red cube to the blue zone/);
  assert.ok(index.indexOf('id="go-home"') < index.indexOf('<h2>Task<\/h2>'));
  assert.match(index, /id="go-straight"/);
  assert.match(app, /planTextTask/);
  assert.match(app, /optimizeJointPath/);
  assert.match(app, /straightUpPose/);
  assert.match(app, /contactNeedsArmStop/);
  const poseHandler = app.slice(app.indexOf('const poseCmd'), app.indexOf("$('go-home')"));
  assert.ok(poseHandler.indexOf('ctl.runPoses') < poseHandler.lastIndexOf('moveVisualS5Offset'), 'visual wrist motion starts only after simulated pose acceptance');
  assert.match(app, /\$\('task-preset'\)\.onchange/);
  assert.match(app, /PhysicsWorld/);
  assert.match(app, /RobotArmClient/);
  assert.match(app, /runtimeSpec/);
  assert.match(app, /photoReference/);
  assert.match(app, /latestGripperObservation/);
  assert.match(webFile('js/camview.js'), /estimateGripperMarkerPose/);
  assert.match(webFile('index.html'), /gripper-marker-id6\.svg/);
});

test('only one application shell remains after the runtime consolidation', () => {
  assert.equal(fs.existsSync(fileURLToPath(new URL('../src/main.js', import.meta.url))), false);
  assert.equal(fs.existsSync(fileURLToPath(new URL('../src/style.css', import.meta.url))), false);
  assert.match(webFile('index.html'), /href="style\.css"/);
});

test('millimetre runtime FK follows the preserved photo-fitted visual hierarchy', () => {
  const canonical = configureCanonicalSpec(loadSpec());
  const servos = structuredClone(SERVOS);
  const visual = applyCanonicalServoMetadata(servos, canonical);
  const runtime = runtimeSpec(servos, null, canonical);
  assert.equal(servos[5].home, 650);
  assert.ok(Math.abs(visual.HOME - 90) < 1e-6);
  assert.ok(Math.abs(visual.PARK) < 1e-6);

  for (const name of ['home', 'park']) {
    const counts = referencePose(runtime, name);
    const rendered = fk(anglesFromCounts(Object.values(counts), servos));
    const planned = forwardKinematics(runtime, counts);
    for (const [renderedPoint, plannedPoint] of [
      [rendered.shoulder.p, planned.shoulder],
      [rendered.elbow.p, planned.elbow],
      [rendered.wrist.p, planned.wrist],
      [rendered.hand.p, planned.flange],
    ]) {
      renderedPoint.forEach((value, axis) => assert.ok(Math.abs(value * 10 - plannedPoint[axis]) < 0.001));
    }
  }
});

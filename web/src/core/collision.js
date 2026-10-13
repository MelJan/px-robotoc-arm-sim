import { add, clamp, distance, dot, scale, subtract, transformPoint } from './math.js';
import { forwardKinematics, interpolateCounts, referencePose } from './kinematics.js';

function segmentDistance(p1, q1, p2, q2) {
  const d1 = subtract(q1, p1); const d2 = subtract(q2, p2); const r = subtract(p1, p2);
  const a = dot(d1, d1); const e = dot(d2, d2); const f = dot(d2, r);
  let s; let t;
  if (a <= 1e-9 && e <= 1e-9) return distance(p1, p2);
  if (a <= 1e-9) { s = 0; t = clamp(f / e, 0, 1); }
  else {
    const c = dot(d1, r);
    if (e <= 1e-9) { t = 0; s = clamp(-c / a, 0, 1); }
    else {
      const b = dot(d1, d2); const denominator = a * e - b * b;
      s = denominator ? clamp((b * f - c * e) / denominator, 0, 1) : 0;
      t = (b * s + f) / e;
      if (t < 0) { t = 0; s = clamp(-c / a, 0, 1); }
      else if (t > 1) { t = 1; s = clamp((b - c) / a, 0, 1); }
    }
  }
  return distance(add(p1, scale(d1, s)), add(p2, scale(d2, t)));
}

function pointBoxDistance(point, box) {
  const half = box.size.map((value) => value / 2);
  const delta = point.map((value, index) => Math.max(Math.abs(value - box.center[index]) - half[index], 0));
  return Math.hypot(...delta);
}

function capsuleBoxClearance(capsule, box) {
  let closest = Number.POSITIVE_INFINITY;
  for (let index = 0; index <= 16; index += 1) {
    const point = add(capsule.start, scale(subtract(capsule.end, capsule.start), index / 16));
    closest = Math.min(closest, pointBoxDistance(point, box));
  }
  return closest - capsule.radius;
}

export function robotCapsules(spec, counts, options = {}) {
  const pose = forwardKinematics(spec, counts);
  const radii = spec.collision.linkRadii;
  const gripperLength = pose.gripper.length;
  const gripperStart = options.physicsGripper ? Math.min(25, gripperLength * 0.25) : Math.min(40, gripperLength * 0.35);
  // Put each gripper capsule on the rendered jaw, with its inner surface at
  // the requested pad gap. The old fixed +/-30 mm centres described an open
  // claw even when S6 was closed, so Rapier could miss visible cube impacts.
  const padRadius = options.physicsGripper ? Math.min(4, radii.gripper) : Math.min(3, radii.gripper);
  const padCenter = options.physicsGripper ? pose.gripper.gap / 2 + padRadius : 30;
  return [
    { id: 'pedestal', start: [0, 0, spec.fixture.upperPlate.top], end: pose.shoulder, radius: radii.pedestal, fixed: true },
    { id: 'upperArm', start: pose.shoulder, end: pose.elbow, radius: radii.upperArm },
    { id: 'forearm', start: pose.elbow, end: pose.wrist, radius: radii.forearm },
    { id: 'wrist', start: pose.wrist, end: pose.flange, radius: radii.wrist },
    { id: 'gripperFront', start: transformPoint(pose.toolTransform, [padCenter, 0, gripperStart]), end: transformPoint(pose.toolTransform, [padCenter, 0, gripperLength]), radius: padRadius },
    { id: 'gripperBack', start: transformPoint(pose.toolTransform, [-padCenter, 0, gripperStart]), end: transformPoint(pose.toolTransform, [-padCenter, 0, gripperLength]), radius: padRadius },
  ];
}

export function worldBoxes(spec, options = {}) {
  const boxes = [
    { id: 'lower fixture plate', ...spec.fixture.lowerPlate },
    { id: 'upper fixture plate', ...spec.fixture.upperPlate },
  ];
  if (!options.ignoreCubes) {
    for (const cube of spec.world.cubes) {
      if (!options.removedCubeIds?.includes(cube.id)) boxes.push({ id: `${cube.id} cube`, cubeId: cube.id, center: cube.position, size: [spec.dimensions.testCube, spec.dimensions.testCube, spec.dimensions.testCube] });
    }
  }
  for (const box of options.extraBoxes ?? []) boxes.push(box);
  return boxes;
}

function poseNearPark(spec, counts) {
  const park = referencePose(spec, 'park');
  return spec.joints.every((joint) => Math.abs(counts[joint.id] - park[joint.id]) <= (joint.id === 'S4' ? 5 : 3));
}

export function collisionReport(spec, counts, options = {}) {
  const margin = options.margin ?? spec.collision.safetyMargin;
  const capsules = robotCapsules(spec, counts);
  const contacts = [];
  let minimumClearance = Number.POSITIVE_INFINITY;

  for (const capsule of capsules.filter((item) => !item.fixed)) {
    const lowest = Math.min(capsule.start[2], capsule.end[2]) - capsule.radius;
    minimumClearance = Math.min(minimumClearance, lowest);
    if (lowest < spec.collision.boardClearance - margin) contacts.push({ a: capsule.id, b: 'board', clearance: lowest });
  }

  const selfPairs = [[0, 3], [1, 3], [1, 4], [1, 5], [2, 4], [2, 5]];
  for (const [first, second] of selfPairs) {
    const a = capsules[first]; const b = capsules[second];
    const clearance = segmentDistance(a.start, a.end, b.start, b.end) - a.radius - b.radius;
    minimumClearance = Math.min(minimumClearance, clearance);
    if (clearance < margin) contacts.push({ a: a.id, b: b.id, clearance });
  }

  const pedestalBody = { id: 'pedestal body', center: [0, 0, 46], size: [43, 43, 42] };
  for (const capsule of capsules.filter((item) => item.id.startsWith('gripper'))) {
    const clearance = capsuleBoxClearance(capsule, pedestalBody);
    minimumClearance = Math.min(minimumClearance, clearance);
    if (clearance < margin) contacts.push({ a: capsule.id, b: pedestalBody.id, clearance });
  }

  for (const capsule of capsules.filter((item) => !item.fixed)) {
    for (const box of worldBoxes(spec, options)) {
      if (box.id.includes('fixture') && capsule.id === 'upperArm') continue;
      const clearance = capsuleBoxClearance(capsule, box);
      const isParkSupport = box.id === 'upper fixture plate' && capsule.id.startsWith('gripper') && poseNearPark(spec, counts) && clearance >= -margin;
      const isGraspContact = box.cubeId && capsule.id.startsWith('gripper') && options.allowedGripperContactCubeIds?.includes(box.cubeId) && clearance >= -margin;
      minimumClearance = Math.min(minimumClearance, isParkSupport || isGraspContact ? 0 : clearance);
      if (clearance < margin && !isParkSupport && !isGraspContact) contacts.push({ a: capsule.id, b: box.id, clearance });
    }
  }

  if (options.carriedCubeId) {
    const pose = forwardKinematics(spec, counts);
    const carriedSize = spec.dimensions.testCube;
    const carriedCenter = transformPoint(pose.toolTransform, [0, 0, pose.gripper.length - carriedSize * 0.88]);
    const carried = { id: `${options.carriedCubeId} cube (carried)`, center: carriedCenter, size: [carriedSize, carriedSize, carriedSize] };
    const carriedBottom = carriedCenter[2] - carriedSize / 2;
    const isBoardSupport = options.allowCarriedBoardSupport && carriedBottom >= -margin;
    minimumClearance = Math.min(minimumClearance, isBoardSupport ? 0 : carriedBottom);
    if (carriedBottom < margin && !isBoardSupport) contacts.push({ a: carried.id, b: 'board', clearance: carriedBottom });
    for (const box of worldBoxes(spec, { ...options, removedCubeIds: [...(options.removedCubeIds ?? []), options.carriedCubeId] })) {
      const halfA = carried.size.map((value) => value / 2); const halfB = box.size.map((value) => value / 2);
      const separation = carried.center.map((value, index) => Math.abs(value - box.center[index]) - halfA[index] - halfB[index]);
      const clearance = Math.max(...separation);
      minimumClearance = Math.min(minimumClearance, clearance);
      if (clearance < margin) contacts.push({ a: carried.id, b: box.id, clearance });
    }
    for (const capsule of capsules.filter((item) => !item.fixed && !item.id.startsWith('gripper'))) {
      const clearance = capsuleBoxClearance(capsule, carried);
      minimumClearance = Math.min(minimumClearance, clearance);
      if (clearance < margin) contacts.push({ a: carried.id, b: capsule.id, clearance });
    }
  }

  const deduplicated = contacts.filter((contact, index) => contacts.findIndex((candidate) => candidate.a === contact.a && candidate.b === contact.b) === index);
  return { valid: deduplicated.length === 0, contacts: deduplicated, minimumClearance };
}

export function verifyPath(spec, path, options = {}) {
  const samples = [];
  for (let segment = 0; segment < path.length - 1; segment += 1) {
    const start = path[segment]; const end = path[segment + 1];
    const largestDelta = Math.max(...spec.joints.map((joint) => Math.abs(end[joint.id] - start[joint.id])));
    const sampleCount = Math.max(1, Math.ceil(largestDelta / (options.stepCounts ?? spec.collision.sampleStepCounts)));
    for (let index = segment === 0 ? 0 : 1; index <= sampleCount; index += 1) {
      const counts = interpolateCounts(spec, start, end, index / sampleCount);
      const report = collisionReport(spec, counts, options);
      samples.push({ counts, report });
      if (!report.valid) return { valid: false, samples, collision: report.contacts[0], segment };
    }
  }
  if (path.length === 1) {
    const report = collisionReport(spec, path[0], options);
    samples.push({ counts: path[0], report });
    return { valid: report.valid, samples, collision: report.contacts[0] };
  }
  return { valid: true, samples, collision: null };
}

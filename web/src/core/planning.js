import { collisionReport, verifyPath } from './collision.js';
import { countDistance, forwardKinematics, interpolateCounts, referencePose, solveInverseKinematics } from './kinematics.js';
import { clamp, distance, lerp, seededRandom } from './math.js';

function changedJointCount(spec, a, b) {
  return spec.joints.filter((joint) => a[joint.id] !== b[joint.id]).length;
}

export function pathMetrics(spec, path) {
  let countTravel = 0; let changedServos = 0; let tcpLength = 0; let acceleration = 0;
  let previousDelta = null; let minimumClearance = Number.POSITIVE_INFINITY;
  for (let index = 0; index < path.length; index += 1) {
    const report = collisionReport(spec, path[index], { ignoreCubes: true });
    minimumClearance = Math.min(minimumClearance, report.minimumClearance);
    if (!index) continue;
    const previous = path[index - 1];
    const delta = spec.joints.map((joint) => path[index][joint.id] - previous[joint.id]);
    countTravel += delta.reduce((sum, value) => sum + Math.abs(value), 0);
    changedServos += changedJointCount(spec, previous, path[index]);
    tcpLength += distance(forwardKinematics(spec, previous).tcp, forwardKinematics(spec, path[index]).tcp);
    if (previousDelta) acceleration += delta.reduce((sum, value, jointIndex) => sum + Math.abs(value - previousDelta[jointIndex]), 0);
    previousDelta = delta;
  }
  const duration = countTravel / 220;
  return { countTravel, changedServos, tcpLength, acceleration, duration, minimumClearance };
}

export function costFromMetrics(spec, metrics) {
  const weights = spec.planning.weights;
  const clearancePenalty = metrics.minimumClearance < 0 ? Number.POSITIVE_INFINITY : 1 / Math.max(metrics.minimumClearance, 1);
  return weights.clearance * clearancePenalty
    + weights.changedServos * metrics.changedServos
    + weights.countTravel * metrics.countTravel
    + weights.duration * metrics.duration
    + weights.acceleration * metrics.acceleration
    + weights.tcpLength * metrics.tcpLength;
}

export function pathCost(spec, path) {
  return costFromMetrics(spec, pathMetrics(spec, path));
}

function edgeValid(spec, start, end, options) {
  return verifyPath(spec, [start, end], options).valid;
}

function steer(spec, from, to, maxStep) {
  const magnitude = countDistance(spec, from, to);
  if (magnitude <= maxStep) return { ...to };
  return interpolateCounts(spec, from, to, maxStep / magnitude);
}

function samplePose(spec, random, goal) {
  if (random() < 0.18) return { ...goal };
  return Object.fromEntries(spec.joints.map((joint) => [joint.id, Math.round(lerp(joint.min, joint.max, random()))]));
}

function recoverPath(nodes, index) {
  const result = [];
  while (index != null) { result.push(nodes[index].pose); index = nodes[index].parent; }
  return result.reverse();
}

function shortcut(spec, path, options, random) {
  let result = [...path];
  for (let iteration = 0; iteration < 120 && result.length > 2; iteration += 1) {
    const first = Math.floor(random() * (result.length - 2));
    const second = first + 2 + Math.floor(random() * (result.length - first - 2));
    if (edgeValid(spec, result[first], result[second], options)) result.splice(first + 1, second - first - 1);
  }
  return result;
}

function smooth(spec, path, options) {
  if (path.length < 3) return path;
  let result = path.map((pose) => ({ ...pose }));
  for (let pass = 0; pass < 24; pass += 1) {
    for (let index = 1; index < result.length - 1; index += 1) {
      const candidate = Object.fromEntries(spec.joints.map((joint) => [joint.id, Math.round((result[index - 1][joint.id] + 2 * result[index][joint.id] + result[index + 1][joint.id]) / 4)]));
      const candidatePath = [...result.slice(0, index), candidate, ...result.slice(index + 1)];
      if (verifyPath(spec, candidatePath.slice(Math.max(0, index - 1), index + 2), options).valid && pathCost(spec, candidatePath) <= pathCost(spec, result)) result = candidatePath;
    }
  }
  return result;
}

export function optimizeJointPath(spec, start, goal, options = {}) {
  const collisionOptions = {
    ignoreCubes: options.ignoreCubes ?? false,
    margin: options.margin,
    removedCubeIds: options.removedCubeIds ?? [],
    allowedGripperContactCubeIds: options.allowedGripperContactCubeIds ?? [],
    carriedCubeId: options.carriedCubeId,
    allowCarriedBoardSupport: options.allowCarriedBoardSupport ?? false,
    extraBoxes: options.extraBoxes ?? [],
  };
  const direct = [start, goal];
  if (verifyPath(spec, direct, collisionOptions).valid) {
    const path = smooth(spec, direct, collisionOptions);
    return { success: true, path, verification: verifyPath(spec, path, collisionOptions), metrics: pathMetrics(spec, path), algorithm: 'direct optimal edge + smoothing' };
  }

  const s4 = spec.joints.find((joint) => joint.id === 'S4');
  const open = spec.gripper.openTick;
  const staged = [start, { ...start, S4: s4.min }, { ...start, S4: s4.min, S6: open }, { ...goal, S4: s4.min, S6: open }, { ...goal, S4: s4.min }, goal];
  if (verifyPath(spec, staged, collisionOptions).valid) {
    const random = seededRandom(options.seed ?? spec.planning.seed);
    const path = smooth(spec, shortcut(spec, staged, collisionOptions, random), collisionOptions);
    return { success: true, path, verification: verifyPath(spec, path, collisionOptions), metrics: pathMetrics(spec, path), algorithm: 'clearance seed + shortcut trajectory optimization' };
  }

  const random = seededRandom(options.seed ?? spec.planning.seed);
  const nodes = [{ pose: { ...start }, parent: null, cost: 0 }];
  const maxIterations = options.maxIterations ?? spec.planning.maxIterations;
  const step = options.stepCounts ?? spec.planning.stepCounts;
  let goalIndex = null;
  for (let iteration = 0; iteration < maxIterations; iteration += 1) {
    const target = samplePose(spec, random, goal);
    let nearestIndex = 0; let nearestDistance = Number.POSITIVE_INFINITY;
    nodes.forEach((node, index) => {
      const candidateDistance = countDistance(spec, node.pose, target);
      if (candidateDistance < nearestDistance) { nearestDistance = candidateDistance; nearestIndex = index; }
    });
    const pose = steer(spec, nodes[nearestIndex].pose, target, step);
    if (!collisionReport(spec, pose, collisionOptions).valid || !edgeValid(spec, nodes[nearestIndex].pose, pose, collisionOptions)) continue;
    const nearby = nodes.map((node, index) => ({ index, distance: countDistance(spec, node.pose, pose) })).filter((item) => item.distance <= step * 2.2);
    let parent = nearestIndex; let cost = nodes[parent].cost + countDistance(spec, nodes[parent].pose, pose);
    for (const item of nearby) {
      const candidateCost = nodes[item.index].cost + item.distance;
      if (candidateCost < cost && edgeValid(spec, nodes[item.index].pose, pose, collisionOptions)) { parent = item.index; cost = candidateCost; }
    }
    const index = nodes.push({ pose, parent, cost }) - 1;
    for (const item of nearby) {
      const candidateCost = cost + item.distance;
      if (candidateCost < nodes[item.index].cost && edgeValid(spec, pose, nodes[item.index].pose, collisionOptions)) {
        nodes[item.index].parent = index; nodes[item.index].cost = candidateCost;
      }
    }
    if (countDistance(spec, pose, goal) <= step && edgeValid(spec, pose, goal, collisionOptions)) {
      goalIndex = nodes.push({ pose: { ...goal }, parent: index, cost: cost + countDistance(spec, pose, goal) }) - 1;
      break;
    }
  }
  if (goalIndex == null) return { success: false, path: [], verification: verifyPath(spec, staged, collisionOptions), reason: 'No collision-free path found within the deterministic planning budget', algorithm: 'RRT* search' };
  let path = recoverPath(nodes, goalIndex);
  path = smooth(spec, shortcut(spec, path, collisionOptions, random), collisionOptions);
  return { success: true, path, verification: verifyPath(spec, path, collisionOptions), metrics: pathMetrics(spec, path), algorithm: 'deterministic RRT* + shortcut/STOMP-style smoothing' };
}

export function controllerCompatiblePath(spec, path) {
  const coordinated = [{ ...path[0] }];
  for (let index = 1; index < path.length; index += 1) {
    const largestDelta = Math.max(...spec.joints.map((joint) => Math.abs(path[index][joint.id] - path[index - 1][joint.id])));
    const steps = Math.max(1, Math.ceil(largestDelta / spec.collision.sampleStepCounts));
    for (let sample = 1; sample <= steps; sample += 1) coordinated.push(interpolateCounts(spec, path[index - 1], path[index], sample / steps));
  }
  const result = [{ ...coordinated[0] }];
  for (let index = 1; index < coordinated.length; index += 1) {
    let cursor = { ...result.at(-1) };
    const pending = spec.joints.filter((joint) => cursor[joint.id] !== coordinated[index][joint.id]);
    while (pending.length) {
      let selected = -1; let selectedPose = null; let selectedClearance = -Infinity;
      pending.forEach((joint, pendingIndex) => {
        const candidate = { ...cursor, [joint.id]: coordinated[index][joint.id] };
        const report = verifyPath(spec, [cursor, candidate], { ignoreCubes: true });
        const clearance = report.samples.reduce((minimum, sample) => Math.min(minimum, sample.report.minimumClearance), Infinity);
        if (report.valid && clearance > selectedClearance) { selected = pendingIndex; selectedPose = candidate; selectedClearance = clearance; }
      });
      if (selected < 0) return { success: false, path: result, reason: 'No collision-free single-servo ordering exists for this coordinated segment' };
      cursor = selectedPose; result.push(cursor); pending.splice(selected, 1);
    }
  }
  return { success: true, path: result };
}

export function parseTextTask(text, spec) {
  const normalized = text.trim().toLowerCase();
  const colors = spec.world.cubes.map((cube) => cube.id);
  const cube = colors.find((color) => new RegExp(`\\b${color}\\b`).test(normalized));
  if (!cube) throw new Error(`Name a cube: ${colors.join(', ')}`);
  const coordinateMatch = normalized.match(/(?:to|at)\s*\(?\s*(-?\d+(?:\.\d+)?)\s*[, ]\s*(-?\d+(?:\.\d+)?)\s*(?:mm)?\s*\)?/);
  const destinationColor = colors.find((color) => color !== cube && new RegExp(`(?:to|onto|at)\\s+(?:the\\s+)?${color}`).test(normalized));
  const destination = coordinateMatch
    ? [Number(coordinateMatch[1]), Number(coordinateMatch[2]), spec.dimensions.testCube / 2]
    : spec.world.dropZones[destinationColor ?? cube];
  return { type: 'pick-place', cube, destination, description: `Move ${cube} cube to (${destination[0]}, ${destination[1]}) mm` };
}

export function planTextTask(spec, start, text) {
  const task = parseTextTask(text, spec);
  const cube = spec.world.cubes.find((item) => item.id === task.cube);
  const approach = spec.planning.approachHeight; const lift = spec.planning.liftHeight;
  const contactZ = spec.dimensions.testCube + 10;
  const targets = [
    { label: 'approach pick', point: [cube.position[0], cube.position[1], contactZ + approach], grip: spec.gripper.openTick },
    { label: 'pick', point: [cube.position[0], cube.position[1], contactZ], grip: spec.gripper.openTick },
    { label: 'grasp', point: [cube.position[0], cube.position[1], contactZ], grip: spec.gripper.closedTick },
    { label: 'lift', point: [cube.position[0], cube.position[1], contactZ + lift], grip: spec.gripper.closedTick },
    { label: 'approach place', point: [task.destination[0], task.destination[1], contactZ + lift], grip: spec.gripper.closedTick },
    { label: 'place', point: [task.destination[0], task.destination[1], contactZ], grip: spec.gripper.closedTick },
    { label: 'release', point: [task.destination[0], task.destination[1], contactZ], grip: spec.gripper.openTick },
    { label: 'retreat', point: [task.destination[0], task.destination[1], contactZ + approach], grip: spec.gripper.openTick },
  ];
  const fullPath = [{ ...start }]; const waypoints = []; const events = []; const phaseVerifications = []; let cursor = { ...start };
  const placedCube = { id: `${task.cube} cube (placed)`, cubeId: task.cube, center: task.destination, size: [spec.dimensions.testCube, spec.dimensions.testCube, spec.dimensions.testCube] };
  for (const target of targets) {
    const ik = solveInverseKinematics(spec, target.point, { start: cursor, gripperTick: target.grip });
    if (!ik.reachable) return { success: false, task, reason: `${target.label} is outside the provisional IK tolerance (${ik.error.toFixed(1)} mm error)`, waypoints };
    const grasping = ['pick', 'grasp'].includes(target.label);
    const carrying = ['lift', 'approach place', 'place'].includes(target.label);
    const released = ['release', 'retreat'].includes(target.label);
    const planned = optimizeJointPath(spec, cursor, ik.counts, {
      seed: spec.planning.seed + waypoints.length,
      removedCubeIds: carrying || released ? [task.cube] : [],
      allowedGripperContactCubeIds: grasping || target.label === 'release' ? [task.cube] : [],
      carriedCubeId: carrying ? task.cube : undefined,
      allowCarriedBoardSupport: target.label === 'place',
      extraBoxes: released ? [placedCube] : [],
    });
    if (!planned.success) return { success: false, task, reason: `${target.label}: ${planned.reason}`, waypoints };
    fullPath.push(...planned.path.slice(1));
    if (target.label === 'grasp') events.push({ type: 'grasp', cube: task.cube, counts: { ...ik.counts } });
    if (target.label === 'approach place') events.push({ type: 'allow-place-support', cube: task.cube, counts: { ...ik.counts } });
    if (target.label === 'release') events.push({ type: 'release', cube: task.cube, counts: { ...ik.counts } });
    phaseVerifications.push(planned.verification);
    waypoints.push({ ...target, counts: ik.counts, ikError: ik.error, collisionVerified: planned.verification.valid });
    cursor = ik.counts;
  }
  const verification = { valid: phaseVerifications.every((phase) => phase.valid), phases: phaseVerifications };
  const metrics = pathMetrics(spec, fullPath);
  metrics.minimumClearance = Math.min(...phaseVerifications.flatMap((phase) => phase.samples.map((sample) => sample.report.minimumClearance)));
  return { success: verification.valid, task, path: fullPath, waypoints, events, verification, metrics, cost: costFromMetrics(spec, metrics), algorithm: 'Cartesian approach/lift waypoints + IK + optimizing joint planner' };
}

export function planReferenceTransition(spec, from, name, options = {}) {
  return optimizeJointPath(spec, from, referencePose(spec, name), {
    ...options,
    ignoreCubes: options.ignoreCubes ?? false,
  });
}

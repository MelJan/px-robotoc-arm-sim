import RAPIER from '@dimforge/rapier3d-compat';
import { robotCapsules } from './collision.js';
import { forwardKinematics } from './kinematics.js';
import { transformPoint } from './math.js';

const METRES_PER_MILLIMETRE = 0.001;
const vector = ([x, y, z]) => ({ x: x * METRES_PER_MILLIMETRE, y: y * METRES_PER_MILLIMETRE, z: z * METRES_PER_MILLIMETRE });

function segmentPose(start, end) {
  const a = vector(start); const b = vector(end);
  const direction = { x: b.x - a.x, y: b.y - a.y, z: b.z - a.z };
  const length = Math.hypot(direction.x, direction.y, direction.z);
  const unit = length ? { x: direction.x / length, y: direction.y / length, z: direction.z / length } : { x: 0, y: 1, z: 0 };
  const from = { x: 0, y: 1, z: 0 };
  const cross = { x: from.y * unit.z - from.z * unit.y, y: from.z * unit.x - from.x * unit.z, z: from.x * unit.y - from.y * unit.x };
  const dot = from.x * unit.x + from.y * unit.y + from.z * unit.z;
  let rotation;
  if (dot < -0.999999) rotation = { x: 1, y: 0, z: 0, w: 0 };
  else {
    const scalar = Math.sqrt((1 + dot) * 2);
    rotation = { x: cross.x / scalar, y: cross.y / scalar, z: cross.z / scalar, w: scalar / 2 };
  }
  return { center: { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2, z: (a.z + b.z) / 2 }, rotation, length };
}

export class PhysicsWorld {
  constructor(spec) {
    this.spec = spec;
    this.ready = false;
    this.cubeBodies = new Map();
    this.cubeColliders = new Map();
    this.robotBodies = new Map();
    this.robotColliders = new Map();
    this.carriedCubeId = null;
    this.initialCubePositions = Object.fromEntries(spec.world.cubes.map((cube) => [cube.id, [...cube.position]]));
  }

  async initialize(initialCounts) {
    await RAPIER.init();
    const [gx, gy, gz] = this.spec.world.physics.gravity;
    this.world = new RAPIER.World({ x: gx, y: gy, z: gz });
    this.world.timestep = this.spec.world.physics.fixedTimeStep;
    this.addFixedBox('board', this.spec.dimensions.boardCenter, this.spec.dimensions.board);
    this.addFixedBox('lower fixture', this.spec.fixture.lowerPlate.center, this.spec.fixture.lowerPlate.size);
    this.addFixedBox('upper fixture', this.spec.fixture.upperPlate.center, this.spec.fixture.upperPlate.size);
    for (const cube of this.spec.world.cubes) {
      const body = this.world.createRigidBody(RAPIER.RigidBodyDesc.dynamic().setTranslation(...Object.values(vector(cube.position))).setCanSleep(true));
      const half = this.spec.dimensions.testCube * METRES_PER_MILLIMETRE / 2;
      const collider = this.world.createCollider(RAPIER.ColliderDesc.cuboid(half, half, half).setDensity(220).setFriction(0.72).setRestitution(0.04), body);
      this.cubeBodies.set(cube.id, body);
      this.cubeColliders.set(cube.id, collider);
    }
    for (const capsule of robotCapsules(this.spec, initialCounts, { physicsGripper: true })) {
      if (capsule.fixed) continue;
      const pose = segmentPose(capsule.start, capsule.end);
      const body = this.world.createRigidBody(RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(pose.center.x, pose.center.y, pose.center.z).setRotation(pose.rotation));
      const halfHeight = Math.max(0.0001, pose.length / 2 - capsule.radius * METRES_PER_MILLIMETRE);
      const collider = this.world.createCollider(RAPIER.ColliderDesc.capsule(halfHeight, capsule.radius * METRES_PER_MILLIMETRE).setFriction(0.65), body);
      this.robotBodies.set(capsule.id, body);
      this.robotColliders.set(capsule.id, collider);
    }
    this.ready = true;
    return this;
  }

  addFixedBox(_name, center, size) {
    const [x, y, z] = center.map((value) => value * METRES_PER_MILLIMETRE);
    const [hx, hy, hz] = size.map((value) => value * METRES_PER_MILLIMETRE / 2);
    const body = this.world.createRigidBody(RAPIER.RigidBodyDesc.fixed().setTranslation(x, y, z));
    this.world.createCollider(RAPIER.ColliderDesc.cuboid(hx, hy, hz).setFriction(0.75), body);
  }

  setRobotPose(counts) {
    if (!this.ready) return;
    for (const capsule of robotCapsules(this.spec, counts, { physicsGripper: true })) {
      const body = this.robotBodies.get(capsule.id);
      if (!body) continue;
      const pose = segmentPose(capsule.start, capsule.end);
      body.setNextKinematicTranslation(pose.center);
      body.setNextKinematicRotation(pose.rotation);
    }
    if (this.carriedCubeId) {
      const body = this.cubeBodies.get(this.carriedCubeId);
      const pose = forwardKinematics(this.spec, counts);
      const center = transformPoint(pose.toolTransform, [0, 0, pose.gripper.length - this.spec.dimensions.testCube * 0.88]);
      body.setNextKinematicTranslation(vector(center));
    }
  }

  graspCube(id, counts) {
    const body = this.cubeBodies.get(id);
    if (!body) throw new Error(`Unknown physics cube: ${id}`);
    this.carriedCubeId = id;
    body.setBodyType(RAPIER.RigidBodyType.KinematicPositionBased, true);
    this.setRobotPose(counts);
  }

  releaseCube() {
    if (!this.carriedCubeId) return;
    const body = this.cubeBodies.get(this.carriedCubeId);
    body.setBodyType(RAPIER.RigidBodyType.Dynamic, true);
    body.setLinvel({ x: 0, y: 0, z: 0 }, true);
    body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    this.carriedCubeId = null;
  }

  step(dt = this.spec.world.physics.fixedTimeStep) {
    if (!this.ready) return;
    this.world.timestep = dt;
    this.world.step();
  }

  robotCubeContacts() {
    if (!this.ready) return [];
    const cubeByHandle = new Map([...this.cubeColliders].map(([id, collider]) => [collider.handle, id]));
    const contacts = [];
    for (const [part, collider] of this.robotColliders) {
      this.world.contactPairsWith(collider, (other) => {
        const cube = cubeByHandle.get(other.handle);
        if (!cube) return;
        let touching = false;
        this.world.contactPair(collider, other, (manifold) => {
          for (let index = 0; index < manifold.numContacts(); index += 1) {
            // Rapier retains broad/narrow-phase pairs before surfaces touch.
            // Treat only penetration or a sub-millimetre speculative contact as
            // a runtime collision.
            if (manifold.contactDist(index) <= 0.0005) touching = true;
          }
        });
        if (touching) contacts.push({ a: part, b: `${cube} cube` });
      });
    }
    return contacts;
  }

  cubeTransforms() {
    return Object.fromEntries([...this.cubeBodies].map(([id, body]) => {
      const position = body.translation(); const rotation = body.rotation();
      const velocity = body.linvel(); const angularVelocity = body.angvel();
      return [id, {
        position: [position.x, position.y, position.z],
        rotation: [rotation.x, rotation.y, rotation.z, rotation.w],
        velocity: [velocity.x, velocity.y, velocity.z],
        angularVelocity: [angularVelocity.x, angularVelocity.y, angularVelocity.z],
        sleeping: body.isSleeping(),
      }];
    }));
  }

  isSettled() {
    return [...this.cubeBodies.values()].every((body) => body.isSleeping() || body.bodyType() !== RAPIER.RigidBodyType.Dynamic);
  }

  resetCubes() {
    this.releaseCube();
    for (const cube of this.spec.world.cubes) {
      const body = this.cubeBodies.get(cube.id); const [x, y, z] = this.initialCubePositions[cube.id].map((value) => value * METRES_PER_MILLIMETRE);
      body.setTranslation({ x, y, z }, true); body.setRotation({ x: 0, y: 0, z: 0, w: 1 }, true);
      body.setLinvel({ x: 0, y: 0, z: 0 }, true); body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    }
  }
}

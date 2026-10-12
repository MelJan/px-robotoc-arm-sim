import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import './style.css';

const spec = await fetch('/arm-spec.json').then((response) => {
  if (!response.ok) throw new Error(`Could not load arm specification (${response.status})`);
  return response.json();
});

const app = document.querySelector('#app');
app.innerHTML = `
  <div class="app-shell">
    <header class="topbar">
      <div>
        <p class="eyebrow">PX hackathon</p>
        <h1>PX-robo-arm-simulator</h1>
      </div>
      <div class="status-cluster" aria-label="Twin status">
        <span class="status-dot" aria-hidden="true"></span>
        <span>Offline twin</span>
        <span class="status-separator" aria-hidden="true"></span>
        <span>Calibration provisional</span>
      </div>
    </header>

    <main class="workspace">
      <section class="viewport-panel" aria-label="Interactive robot model">
        <div id="scene" class="scene" aria-label="Three dimensional robot arm simulation"></div>
        <div class="view-actions">
          <button id="reset-view" type="button">Reset view</button>
          <label class="toggle"><input id="show-axes" type="checkbox" checked /> Joint axes</label>
        </div>
        <div class="readout" aria-live="polite">
          <div><span>TCP</span><strong id="tcp-value">—</strong></div>
          <div><span>Clearance</span><strong id="clearance-value">—</strong></div>
        </div>
      </section>

      <aside class="control-panel" aria-label="Joint controls">
        <div class="panel-heading">
          <div>
            <p class="eyebrow">Commanded pose</p>
            <h2>Servo counts</h2>
          </div>
          <button id="play-transition" class="primary" type="button">Play HOME → PARK</button>
        </div>

        <div class="preset-row" aria-label="Pose presets">
          <button type="button" data-preset="home">HOME</button>
          <button type="button" data-preset="park">PARK</button>
          <button type="button" data-preset="zero">Straight-up</button>
        </div>

        <div id="joint-controls" class="joint-controls"></div>

        <div class="notice">
          <strong>Geometry is measured; angular zeros are provisional.</strong>
          This model never sends commands to the physical arm.
        </div>
      </aside>
    </main>
  </div>
`;

const sceneHost = document.querySelector('#scene');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x111820);

const camera = new THREE.PerspectiveCamera(38, 1, 0.01, 10);
camera.up.set(0, 0, 1);
const defaultCamera = {
  position: new THREE.Vector3(0.58, -0.62, 0.42),
  target: new THREE.Vector3(0, 0.15, 0.19),
};
camera.position.copy(defaultCamera.position);

const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace;
sceneHost.append(renderer.domElement);

const orbit = new OrbitControls(camera, renderer.domElement);
orbit.target.copy(defaultCamera.target);
orbit.enableDamping = true;
orbit.minDistance = 0.22;
orbit.maxDistance = 1.8;
orbit.update();

scene.add(new THREE.HemisphereLight(0xdbeafe, 0x17202a, 2.1));
const keyLight = new THREE.DirectionalLight(0xffffff, 3.2);
keyLight.position.set(-0.35, -0.45, 0.75);
keyLight.castShadow = true;
keyLight.shadow.mapSize.set(2048, 2048);
scene.add(keyLight);

const fillLight = new THREE.DirectionalLight(0x9fc5ff, 1.2);
fillLight.position.set(0.55, 0.4, 0.3);
scene.add(fillLight);

const colors = {
  board: 0x293541,
  base: 0x1b2229,
  arm: 0xdbe3e8,
  armDark: 0x75828d,
  servo: 0x252c33,
  joint: 0xf3a23a,
  gripper: 0xb8c3ca,
  pad: 0x22272c,
  tcp: 0x54d18c,
};

function material(color, roughness = 0.62, metalness = 0.14) {
  return new THREE.MeshStandardMaterial({ color, roughness, metalness });
}

function box(name, size, color, position = [0, 0, 0]) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(...size), material(color));
  mesh.name = name;
  mesh.position.set(...position);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

function cylinder(name, radius, depth, color, position = [0, 0, 0]) {
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, depth, 32), material(color));
  mesh.name = name;
  mesh.rotation.x = Math.PI / 2;
  mesh.position.set(...position);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

function link(name, length, width = 0.026, depth = 0.018) {
  const group = new THREE.Group();
  group.name = name;
  group.add(box(`${name}_beam`, [width, depth, length], colors.arm, [0, 0, length / 2]));
  group.add(box(`${name}_rail_left`, [0.005, depth + 0.006, length], colors.armDark, [-width / 2, 0, length / 2]));
  group.add(box(`${name}_rail_right`, [0.005, depth + 0.006, length], colors.armDark, [width / 2, 0, length / 2]));
  return group;
}

function jointMarker(name) {
  const marker = new THREE.Mesh(new THREE.SphereGeometry(0.014, 24, 16), material(colors.joint, 0.45, 0.2));
  marker.name = `${name}_marker`;
  marker.castShadow = true;
  return marker;
}

const d = spec.dimensions;
const board = box('Board', d.board, colors.board, d.boardCenter);
scene.add(board);

const grid = new THREE.GridHelper(0.8, 16, 0x5f7181, 0x34414d);
grid.name = 'Board_grid';
grid.rotation.x = Math.PI / 2;
grid.position.set(0, 0.19, 0.001);
grid.material.opacity = 0.35;
grid.material.transparent = true;
scene.add(grid);

const worldAxes = new THREE.AxesHelper(0.09);
worldAxes.position.z = 0.002;
scene.add(worldAxes);

const robotRoot = new THREE.Group();
robotRoot.name = 'ARM_Root';
scene.add(robotRoot);
robotRoot.add(box('Base_plate', [0.09, 0.084, 0.008], colors.base, [0, 0, 0.004]));

const s1 = new THREE.Group();
s1.name = 'S1_joint';
robotRoot.add(s1);
s1.add(cylinder('S1_servo', 0.032, 0.05, colors.servo, [0, 0, 0.032]));
s1.add(box('Pedestal', [0.055, 0.045, d.shoulderHeight - 0.03], colors.armDark, [0, 0, 0.03 + (d.shoulderHeight - 0.03) / 2]));

const s2 = new THREE.Group();
s2.name = 'S2_joint';
s2.position.z = d.shoulderHeight;
s1.add(s2);
s2.add(jointMarker('S2'));
s2.add(cylinder('S2_servo', 0.018, 0.052, colors.servo));
s2.add(link('Upper_arm', d.upperArm));

const s3 = new THREE.Group();
s3.name = 'S3_joint';
s3.position.z = d.upperArm;
s2.add(s3);
s3.add(jointMarker('S3'));
s3.add(cylinder('S3_servo', 0.018, 0.052, colors.servo));
s3.add(link('Forearm', d.forearm, 0.024, 0.016));

const s4 = new THREE.Group();
s4.name = 'S4_joint';
s4.position.z = d.forearm;
s3.add(s4);
s4.add(jointMarker('S4'));
s4.add(cylinder('S4_servo', 0.017, 0.047, colors.servo));
s4.add(link('Wrist', d.wrist, 0.022, 0.016));

const s5 = new THREE.Group();
s5.name = 'S5_joint';
s5.position.z = d.wrist;
s4.add(s5);
s5.add(jointMarker('S5'));
s5.add(cylinder('S5_servo', 0.017, 0.045, colors.servo));

const gripper = new THREE.Group();
gripper.name = 'S6_gripper';
s5.add(gripper);
gripper.add(box('Gripper_palm', [0.062, 0.024, 0.018], colors.gripper, [0, 0, 0.014]));

const fingerLeft = box('Finger_left', [0.012, 0.005, 0.085], colors.gripper);
const fingerRight = box('Finger_right', [0.012, 0.005, 0.085], colors.gripper);
const padLeft = box('Pad_left', [0.014, 0.004, 0.012], colors.pad);
const padRight = box('Pad_right', [0.014, 0.004, 0.012], colors.pad);
gripper.add(fingerLeft, fingerRight, padLeft, padRight);

const tcp = new THREE.Object3D();
tcp.name = 'TCP';
gripper.add(tcp);
const tcpMarker = new THREE.Mesh(new THREE.SphereGeometry(0.006, 20, 12), material(colors.tcp, 0.35, 0.05));
tcp.add(tcpMarker);

const jointAxes = [s1, s2, s3, s4, s5].map((joint, index) => {
  const helper = new THREE.AxesHelper(index === 0 ? 0.065 : 0.045);
  joint.add(helper);
  return helper;
});

const jointGroups = { S1: s1, S2: s2, S3: s3, S4: s4, S5: s5 };
const currentTicks = Object.fromEntries(spec.joints.map((joint) => [joint.id, joint.home]));

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function tickToDegrees(joint, tick) {
  if (!joint.ticksPerDegree) return null;
  return joint.direction * (tick - joint.zero) / joint.ticksPerDegree;
}

function gripperState(tick) {
  const openTick = 490;
  const closedTick = 650;
  const closure = clamp((tick - openTick) / (closedTick - openTick), 0, 1);
  return {
    closure,
    gap: THREE.MathUtils.lerp(d.gripperOpenGap, d.gripperClosedGap, closure),
    length: THREE.MathUtils.lerp(d.gripperOpenLength, d.gripperClosedLength, closure),
  };
}

function updateGripper(tick) {
  const state = gripperState(tick);
  const fingerLength = Math.max(0.055, state.length - 0.025);
  const side = state.gap / 2 + 0.004;
  const z = 0.025 + fingerLength / 2;

  fingerLeft.scale.z = fingerLength / 0.085;
  fingerRight.scale.z = fingerLength / 0.085;
  fingerLeft.position.set(0, side, z);
  fingerRight.position.set(0, -side, z);
  padLeft.position.set(0, state.gap / 2 + 0.002, state.length - 0.006);
  padRight.position.set(0, -(state.gap / 2 + 0.002), state.length - 0.006);
  tcp.position.set(0, 0, state.length);
  return state;
}

function updateModel() {
  for (const joint of spec.joints.slice(0, 5)) {
    const radians = THREE.MathUtils.degToRad(tickToDegrees(joint, currentTicks[joint.id]));
    if (joint.id === 'S1' || joint.id === 'S5') jointGroups[joint.id].rotation.z = radians;
    else jointGroups[joint.id].rotation.x = -radians;
  }
  const grip = updateGripper(currentTicks.S6);
  scene.updateMatrixWorld(true);
  updateReadout(grip);
}

const tcpValue = document.querySelector('#tcp-value');
const clearanceValue = document.querySelector('#clearance-value');
const worldPosition = new THREE.Vector3();

function updateReadout(grip) {
  tcp.getWorldPosition(worldPosition);
  tcpValue.textContent = `${(worldPosition.x * 100).toFixed(1)}, ${(worldPosition.y * 100).toFixed(1)}, ${(worldPosition.z * 100).toFixed(1)} cm`;
  const clearance = worldPosition.z * 100;
  clearanceValue.textContent = clearance < 0
    ? `${Math.abs(clearance).toFixed(1)} cm below board`
    : `${clearance.toFixed(1)} cm · gap ${(grip.gap * 100).toFixed(1)} cm`;
  clearanceValue.dataset.warning = clearance < 0 ? 'true' : 'false';
}

const controlsHost = document.querySelector('#joint-controls');
for (const joint of spec.joints) {
  const row = document.createElement('div');
  row.className = 'joint-control';
  row.innerHTML = `
    <div class="joint-label">
      <label for="joint-${joint.id}"><strong>${joint.id}</strong> ${joint.name}</label>
      <output id="output-${joint.id}" for="joint-${joint.id}"></output>
    </div>
    <input id="joint-${joint.id}" type="range" min="${joint.min}" max="${joint.max}" step="1" value="${joint.home}" />
    <div class="joint-bounds"><span>${joint.min}</span><span>${joint.max}</span></div>
  `;
  controlsHost.append(row);

  const input = row.querySelector('input');
  input.addEventListener('input', () => {
    currentTicks[joint.id] = Number(input.value);
    updateOutput(joint);
    updateModel();
  });
  updateOutput(joint);
}

function updateOutput(joint) {
  const output = document.querySelector(`#output-${joint.id}`);
  const tick = currentTicks[joint.id];
  if (joint.id === 'S6') {
    output.textContent = `${tick} · ${(gripperState(tick).gap * 100).toFixed(1)} cm`;
  } else {
    output.textContent = `${tick} · ${tickToDegrees(joint, tick).toFixed(1)}°`;
  }
}

function syncControls() {
  for (const joint of spec.joints) {
    document.querySelector(`#joint-${joint.id}`).value = currentTicks[joint.id];
    updateOutput(joint);
  }
  updateModel();
}

let poseAnimation;
function animateTo(target, duration = 650) {
  if (poseAnimation) cancelAnimationFrame(poseAnimation);
  const from = { ...currentTicks };
  const reduceMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  const started = performance.now();
  const effectiveDuration = reduceMotion ? 0 : duration;

  function frame(now) {
    const progress = effectiveDuration === 0 ? 1 : clamp((now - started) / effectiveDuration, 0, 1);
    const eased = 1 - Math.pow(1 - progress, 3);
    for (const joint of spec.joints) {
      currentTicks[joint.id] = Math.round(THREE.MathUtils.lerp(from[joint.id], target[joint.id], eased));
    }
    syncControls();
    if (progress < 1) poseAnimation = requestAnimationFrame(frame);
  }
  poseAnimation = requestAnimationFrame(frame);
}

function preset(name) {
  if (name === 'home' || name === 'park') {
    return Object.fromEntries(spec.joints.map((joint) => [joint.id, joint[name]]));
  }
  return Object.fromEntries(spec.joints.map((joint) => [joint.id, joint.id === 'S6' ? 490 : clamp(joint.zero, joint.min, joint.max)]));
}

document.querySelectorAll('[data-preset]').forEach((button) => {
  button.addEventListener('click', () => animateTo(preset(button.dataset.preset)));
});

document.querySelector('#play-transition').addEventListener('click', async () => {
  animateTo(preset('home'), 500);
  window.setTimeout(() => animateTo(preset('park'), 950), 650);
});

document.querySelector('#reset-view').addEventListener('click', () => {
  camera.position.copy(defaultCamera.position);
  orbit.target.copy(defaultCamera.target);
  orbit.update();
});

document.querySelector('#show-axes').addEventListener('change', (event) => {
  jointAxes.forEach((helper) => { helper.visible = event.target.checked; });
  worldAxes.visible = event.target.checked;
});

const resizeObserver = new ResizeObserver(() => {
  const width = sceneHost.clientWidth;
  const height = sceneHost.clientHeight;
  if (!width || !height) return;
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
});
resizeObserver.observe(sceneHost);

function render() {
  orbit.update();
  renderer.render(scene, camera);
  requestAnimationFrame(render);
}

updateModel();
render();

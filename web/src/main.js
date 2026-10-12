import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { loadArmSpec, createRenderSpec } from './config/spec.js';
import { collisionReport } from './core/collision.js';
import { TrajectoryPlayer } from './core/controller.js';
import { controllerCompatiblePath, pathCost, planReferenceTransition, planTextTask } from './core/planning.js';
import { createGeometryKit } from './model/geometry.js';
import { RobotArmClient } from './operational/client.js';
import './style.css';

const modelSpec = await loadArmSpec();
const spec = createRenderSpec(modelSpec);
const initialCubePositions = Object.fromEntries(modelSpec.world.cubes.map((cube) => [cube.id, [...cube.position]]));

const app = document.querySelector('#app');
app.innerHTML = `
  <div class="app-shell">
    <header class="topbar">
      <div>
        <p class="eyebrow">PX hackathon</p>
        <h1>PX-robo-arm-simulator</h1>
      </div>
      <div class="status-cluster" aria-label="Twin status">
        <span id="status-dot" class="status-dot" aria-hidden="true"></span>
        <span id="mode-label">Simulation</span>
        <span class="status-separator" aria-hidden="true"></span>
        <span id="physics-label">Physics loading…</span>
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
          <div><span>Board constraint</span><strong id="constraint-value">—</strong></div>
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

        <section class="control-card task-card" aria-labelledby="task-heading">
          <div class="card-heading">
            <div><p class="eyebrow">Planning</p><h2 id="task-heading">Text task</h2></div>
            <span class="badge">mm · IK · RRT*</span>
          </div>
          <label class="field-label" for="task-input">Instruction</label>
          <textarea id="task-input" rows="2">Move the red cube to the blue zone</textarea>
          <div class="button-row">
            <button id="plan-task" class="primary" type="button">Plan and verify</button>
            <button id="run-plan" type="button" disabled>Play plan</button>
            <button id="reset-objects" type="button">Reset objects</button>
          </div>
          <label class="field-label" for="playback-mode">Playback</label>
          <select id="playback-mode">
            <option value="coordinated">Coordinated trajectory</option>
            <option value="single">Controller-compatible · one servo at a time</option>
          </select>
          <div id="plan-result" class="result-box" aria-live="polite">No plan yet.</div>
        </section>

        <details class="control-card mode-card">
          <summary><span><span class="eyebrow">Safety boundary</span>Mode &amp; robot API</span><span id="mode-badge" class="badge safe">SIMULATION</span></summary>
          <p class="mode-copy">Simulation is the default. The robot API has no measured position feedback, so real-mode values are commanded pose only.</p>
          <label class="field-label" for="mode-select">Operating mode</label>
          <select id="mode-select">
            <option value="simulation">Simulation</option>
            <option value="connected-read-only">Connected read-only</option>
            <option value="operational-disarmed">Operational · disarmed</option>
          </select>
          <label class="arming-confirm"><input id="park-confirm" type="checkbox" /> I have physically supported the arm and confirmed PARK with servo power readiness.</label>
          <div class="button-row">
            <button id="arm-real" type="button" disabled>Arm operational mode</button>
            <button id="power-off" type="button" disabled>Return to PARK &amp; power off</button>
            <button id="emergency-stop" class="danger" type="button">Emergency stop</button>
          </div>
          <div id="mode-result" class="result-box" aria-live="polite">No robot API connection. Same-origin serving is required.</div>
        </details>

        <div class="notice">
          <strong>Measured dimensions, provisional calibration and historical bounds remain distinct.</strong>
          Planning is simulation-only unless operational mode is explicitly connected and armed.
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
  position: new THREE.Vector3(-0.72, -0.32, 0.44),
  target: new THREE.Vector3(0, 0.15, 0.17),
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
  boardEdge: 0x30343a,
  base: 0x111315,
  bracket: 0x1b1e21,
  bracketEdge: 0x34393e,
  servo: 0x0d0f11,
  horn: 0xc7ccd0,
  fastener: 0xe0e4e7,
  gripper: 0x17191b,
  pad: 0x303438,
};

const { addFaceFasteners, addOutline, box, bracketLink, cableTube, cylinder, servoAssembly } = createGeometryKit(THREE, colors);

const d = spec.dimensions;

function drawFiducial(context, x, y, size, pattern) {
  context.fillStyle = '#111111';
  context.fillRect(x - size / 2, y - size / 2, size, size);
  const cell = size / 6;
  context.fillStyle = '#ffffff';
  for (const [column, row, width = 1, height = 1] of pattern) {
    context.fillRect(x - size / 2 + column * cell, y - size / 2 + row * cell, width * cell, height * cell);
  }
}

function createBoardTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = 900;
  canvas.height = 1140;
  const context = canvas.getContext('2d');
  const xToCanvas = (millimetres) => ((millimetres + 150) / 300) * canvas.width;
  const yToCanvas = (millimetres) => canvas.height - ((millimetres + 15) / 380) * canvas.height;

  context.fillStyle = '#f7f7f3';
  context.fillRect(0, 0, canvas.width, canvas.height);

  for (let x = -150; x <= 150; x += 10) {
    const major = x % 50 === 0;
    context.strokeStyle = major ? '#8c9296' : '#d8dbdc';
    context.lineWidth = major ? 2.2 : 0.8;
    context.beginPath();
    context.moveTo(xToCanvas(x), 0);
    context.lineTo(xToCanvas(x), canvas.height);
    context.stroke();
  }
  for (let y = 0; y <= 350; y += 10) {
    const major = y % 50 === 0;
    context.strokeStyle = major ? '#8c9296' : '#d8dbdc';
    context.lineWidth = major ? 2.2 : 0.8;
    context.beginPath();
    context.moveTo(0, yToCanvas(y));
    context.lineTo(canvas.width, yToCanvas(y));
    context.stroke();
  }

  context.strokeStyle = '#303438';
  context.lineWidth = 8;
  context.strokeRect(5, 5, canvas.width - 10, canvas.height - 10);
  context.strokeStyle = '#25282b';
  context.lineWidth = 3.5;
  context.beginPath();
  context.moveTo(xToCanvas(0), 0);
  context.lineTo(xToCanvas(0), canvas.height);
  context.stroke();

  context.fillStyle = '#202326';
  context.textAlign = 'center';
  context.textBaseline = 'middle';
  context.font = '600 34px ui-sans-serif, system-ui, sans-serif';
  for (const x of [-100, -50, 0, 50, 100]) context.fillText(`${x > 0 ? '+' : ''}${x}`, xToCanvas(x), yToCanvas(280));
  for (const y of [50, 100, 150, 200, 250, 300, 350]) {
    context.fillText(`${y}`, xToCanvas(-126), yToCanvas(y));
    context.fillText(`${y}`, xToCanvas(126), yToCanvas(y));
  }
  context.font = '700 27px ui-sans-serif, system-ui, sans-serif';
  context.fillText('Y (mm)', xToCanvas(0), yToCanvas(305));
  context.fillText('X (mm)', xToCanvas(82), yToCanvas(172));
  context.fillText('+Z UP', xToCanvas(0), yToCanvas(350));
  context.font = '700 21px ui-sans-serif, system-ui, sans-serif';
  context.fillText('360 – FAR EDGE', xToCanvas(0), yToCanvas(366));

  const patterns = [
    [[1, 1, 1, 3], [2, 3, 3, 1], [4, 1, 1, 2]],
    [[1, 1, 3, 1], [1, 2, 1, 3], [3, 3, 2, 1]],
    [[1, 1, 1, 2], [2, 2, 3, 1], [4, 3, 1, 2]],
  ];
  [32, 152, 338].forEach((y, index) => {
    drawFiducial(context, xToCanvas(-116), yToCanvas(y), 82, patterns[index]);
    drawFiducial(context, xToCanvas(116), yToCanvas(y), 82, patterns[(index + 1) % patterns.length]);
  });

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = renderer.capabilities.getMaxAnisotropy();
  return texture;
}

const board = box('Board', d.board, colors.boardEdge, d.boardCenter);
scene.add(board);
const boardSurface = new THREE.Mesh(
  new THREE.PlaneGeometry(d.board[0], d.board[1]),
  new THREE.MeshStandardMaterial({ map: createBoardTexture(), roughness: 0.82, metalness: 0.02 }),
);
boardSurface.name = 'Board_calibration_surface';
boardSurface.position.set(d.boardCenter[0], d.boardCenter[1], 0.0006);
boardSurface.receiveShadow = true;
scene.add(boardSurface);
scene.add(box('Board_left_rail', [0.007, d.board[1], 0.008], 0x4b5054, [-d.board[0] / 2 + 0.0035, d.boardCenter[1], -0.001]));
scene.add(box('Board_right_rail', [0.007, d.board[1], 0.008], 0x4b5054, [d.board[0] / 2 - 0.0035, d.boardCenter[1], -0.001]));

function createSolidCube(name, color, position) {
  const block = new THREE.Group();
  block.name = name;
  block.position.set(...position);
  block.add(addOutline(box(`${name}_body`, [d.testCube, d.testCube, d.testCube], color), colors.boardEdge));
  return block;
}

const solidCubes = new THREE.Group();
solidCubes.name = 'Solid_cubes';
const cubeMeshes = new Map();
for (const cube of spec.world.cubes) {
  const mesh = createSolidCube(`Cube_${cube.id}`, cube.color, cube.position);
  cubeMeshes.set(cube.id, mesh);
  solidCubes.add(mesh);
}
scene.add(solidCubes);

const worldAxes = new THREE.AxesHelper(0.09);
worldAxes.position.z = 0.002;
scene.add(worldAxes);

const robotRoot = new THREE.Group();
robotRoot.name = 'ARM_Root';
scene.add(robotRoot);
robotRoot.add(addOutline(box('Base_lower_plate', spec.fixture.lowerPlate.size, colors.base, spec.fixture.lowerPlate.center)));
robotRoot.add(addOutline(box('Base_upper_plate', spec.fixture.upperPlate.size, colors.bracket, spec.fixture.upperPlate.center)));
for (const [index, center] of spec.fixture.standoffs.centers.entries()) {
  robotRoot.add(cylinder(`Base_standoff_${index}`, spec.fixture.standoffs.radius, spec.fixture.standoffs.height, colors.horn, center, 'Z'));
  robotRoot.add(cylinder(`Base_fastener_${index}`, 0.003, 0.002, colors.fastener, [center[0], center[1], spec.fixture.upperPlate.top + 0.001], 'Z'));
}

const s1 = new THREE.Group();
s1.name = 'S1_joint';
robotRoot.add(s1);
s1.add(cylinder('S1_turntable', 0.03, 0.008, colors.bracket, [0, 0, 0.029], 'Z'));
s1.add(cylinder('S1_horn', 0.014, 0.005, colors.horn, [0, 0, 0.035], 'Z'));
s1.add(addOutline(box('S1_servo_body', [0.043, 0.043, 0.042], colors.servo, [0, 0, 0.046]), 0x464b50));
s1.add(box('Shoulder_yoke_left', [0.004, 0.05, 0.045], colors.bracket, [-0.028, 0, 0.073]));
s1.add(box('Shoulder_yoke_right', [0.004, 0.05, 0.045], colors.bracket, [0.028, 0, 0.073]));
s1.add(box('Shoulder_yoke_bridge', [0.06, 0.05, 0.005], colors.bracket, [0, 0, 0.062]));
s1.add(addOutline(box('S1_front_link_plate', [0.005, 0.050, 0.071], colors.bracket, [-0.031, 0, 0.060])));
s1.add(box('S1_servo_label', [0.0015, 0.015, 0.007], 0xd6a526, [-0.034, 0, 0.050]));
for (let row = 0; row < 3; row += 1) {
  for (let column = 0; column < 3; column += 1) {
    s1.add(cylinder(`S1_vent_${row}_${column}`, 0.0018, 0.007, colors.servo, [-0.034, -0.014 + column * 0.014, 0.032 + row * 0.014]));
  }
}
addFaceFasteners(s1, 'S1_plate_bolt', 0.068, [[-0.019, 0.027], [0.019, 0.027], [-0.019, 0.088], [0.019, 0.088]]);
s1.add(cableTube('S1_pink_cable', [[-0.036, -0.020, 0.026], [-0.038, -0.024, 0.047], [-0.037, -0.022, 0.071], [-0.033, -0.018, 0.091]], 0xe37b9e, 0.0013));
s1.add(cableTube('S1_orange_cable', [[-0.038, -0.016, 0.030], [-0.040, -0.019, 0.054], [-0.037, -0.017, 0.078], [-0.031, -0.014, 0.094]], 0xf0782c, 0.0012));

const s2 = new THREE.Group();
s2.name = 'S2_joint';
s2.position.z = d.shoulderHeight;
s1.add(s2);
s2.add(servoAssembly('S2', 0.06));
s2.add(bracketLink('Upper_arm', d.upperArm, 0.06, 0.02));
s2.add(cableTube('Upper_arm_signal_cable', [[-0.034, -0.015, 0.006], [-0.036, -0.018, 0.034], [-0.035, -0.018, 0.071], [-0.030, -0.014, d.upperArm - 0.006]], 0xf0782c));
s2.add(cableTube('Upper_arm_power_cable', [[-0.030, -0.012, 0.004], [-0.032, -0.014, 0.036], [-0.031, -0.014, 0.073], [-0.027, -0.011, d.upperArm - 0.004]], 0x202326, 0.0012));

const s3 = new THREE.Group();
s3.name = 'S3_joint';
s3.position.z = d.upperArm;
s2.add(s3);
s3.add(servoAssembly('S3', 0.054));
s3.add(bracketLink('Forearm', d.forearm, 0.054, 0.018));
s3.add(cylinder('Forearm_aluminium_tube', 0.0065, d.forearm - 0.038, colors.horn, [0, 0, d.forearm / 2], 'Z'));
s3.add(cylinder('Forearm_tube_collar_lower', 0.009, 0.010, colors.fastener, [0, 0, 0.024], 'Z'));
s3.add(cylinder('Forearm_tube_collar_upper', 0.009, 0.010, colors.fastener, [0, 0, d.forearm - 0.024], 'Z'));
s3.add(cableTube('Forearm_orange_cable', [[-0.031, -0.014, 0.005], [-0.034, -0.017, 0.042], [-0.033, -0.017, 0.086], [-0.028, -0.013, d.forearm - 0.005]], 0xf0782c));
s3.add(cableTube('Forearm_black_cable', [[-0.027, -0.011, 0.006], [-0.030, -0.013, 0.043], [-0.029, -0.013, 0.087], [-0.025, -0.010, d.forearm - 0.004]], 0x202326, 0.00115));

const s4 = new THREE.Group();
s4.name = 'S4_joint';
s4.position.z = d.forearm;
s3.add(s4);
s4.add(servoAssembly('S4', 0.05));
s4.add(bracketLink('Wrist', d.wrist, 0.05, 0.017));
s4.add(box('Wrist_side_strap', [0.005, 0.026, d.wrist - 0.014], colors.bracket, [-0.027, 0, d.wrist / 2]));
s4.add(cableTube('Wrist_signal_cable', [[-0.029, -0.013, 0.004], [-0.031, -0.015, 0.025], [-0.029, -0.014, d.wrist - 0.005]], 0xf0782c, 0.00125));

const s5 = new THREE.Group();
s5.name = 'S5_joint';
s5.position.z = d.wrist;
s4.add(s5);
s5.add(addOutline(box('S5_servo_body', [0.038, 0.024, 0.042], colors.servo, [0, 0, -0.018]), 0x464b50));
s5.add(cylinder('S5_output_horn', 0.014, 0.006, colors.horn, [0, 0, 0.006], 'Z'));
s5.add(box('S5_servo_label', [0.0015, 0.014, 0.006], 0xd6a526, [-0.0198, 0, -0.020]));
addFaceFasteners(s5, 'S5_case_bolt', 0.044, [[-0.008, -0.032], [0.008, -0.032], [-0.008, -0.006], [0.008, -0.006]], 0.0018);

const gripper = new THREE.Group();
gripper.name = 'S6_gripper';
s5.add(gripper);
gripper.add(addOutline(box('Gripper_front_plate', [0.006, 0.052, 0.038], colors.gripper, [-0.022, 0, 0.022])));
gripper.add(addOutline(box('Gripper_back_plate', [0.006, 0.052, 0.038], colors.gripper, [0.022, 0, 0.022])));
gripper.add(box('Gripper_plate_bridge_lower', [0.044, 0.010, 0.007], colors.bracket, [0, 0, 0.008]));
gripper.add(box('Gripper_plate_bridge_upper', [0.044, 0.010, 0.007], colors.bracket, [0, 0, 0.034]));
gripper.add(cylinder('Gripper_gear_left', 0.013, 0.052, colors.bracketEdge, [0, 0.014, 0.023]));
gripper.add(cylinder('Gripper_gear_right', 0.013, 0.052, colors.bracketEdge, [0, -0.014, 0.023]));

function pivotHardware(name, washerRadius = 0.0055, depth = 0.060) {
  const hardware = new THREE.Group();
  hardware.name = name;
  hardware.add(cylinder(`${name}_washer`, washerRadius, depth, colors.horn));
  hardware.add(cylinder(`${name}_bolt`, washerRadius * 0.63, depth + 0.002, colors.fastener));
  hardware.add(cylinder(`${name}_socket`, washerRadius * 0.24, depth + 0.003, colors.pad));
  return hardware;
}

const drivePivotLeft = pivotHardware('Drive_pivot_left', 0.0058);
const drivePivotRight = pivotHardware('Drive_pivot_right', 0.0058);
drivePivotLeft.position.set(0, 0.014, 0.023);
drivePivotRight.position.set(0, -0.014, 0.023);
gripper.add(drivePivotLeft, drivePivotRight);

for (const side of [1, -1]) {
  for (let toothIndex = 0; toothIndex < 10; toothIndex += 1) {
    const angle = (toothIndex / 10) * Math.PI * 2;
    const tooth = box(`Drive_gear_${side}_${toothIndex}`, [0.054, 0.0035, 0.005], colors.bracket);
    tooth.position.set(0, side * 0.014 + Math.sin(angle) * 0.0135, 0.023 + Math.cos(angle) * 0.0135);
    tooth.rotation.x = -angle;
    gripper.add(tooth);
  }
}

const actuator = new THREE.Group();
actuator.name = 'S6_actuator';
actuator.add(addOutline(box('S6_actuator_body', [0.028, 0.031, 0.032], colors.servo, [-0.033, -0.028, 0.020]), 0x464b50));
actuator.add(box('S6_actuator_mount', [0.036, 0.040, 0.004], colors.bracket, [-0.018, -0.020, 0.007]));
actuator.add(box('S6_actuator_label', [0.0015, 0.017, 0.008], 0xe3ae27, [-0.0475, -0.028, 0.024]));
actuator.add(box('S6_actuator_label_stripe', [0.0018, 0.012, 0.0015], colors.servo, [-0.0484, -0.028, 0.024]));
gripper.add(actuator);

gripper.add(cylinder('S6_mounting_collar', 0.016, 0.006, colors.horn, [0, 0, 0.004], 'Z'));
gripper.add(cylinder('S6_mounting_screw', 0.004, 0.008, colors.fastener, [0, 0, 0.004], 'Z'));

for (const [index, y, z] of [[0, 0.022, 0.008], [1, -0.022, 0.008], [2, 0.022, 0.036], [3, -0.022, 0.036]]) {
  gripper.add(cylinder(`Palm_rivet_${index}`, 0.0023, 0.060, colors.fastener, [0, y, z]));
}

const padLeft = box('Pad_left', [0.014, 0.004, 0.012], colors.pad);
const padRight = box('Pad_right', [0.014, 0.004, 0.012], colors.pad);
const padFaceLeft = box('Pad_face_left', [0.016, 0.0015, 0.010], colors.horn);
const padFaceRight = box('Pad_face_right', [0.016, 0.0015, 0.010], colors.horn);

function gripperBar(name, x, width = 0.005, depth = 0.007) {
  return addOutline(box(name, [width, depth, 1], colors.gripper, [x, 0, 0]));
}

const linkageBars = [
  { mesh: gripperBar('Link_left_inner', -0.018), side: 1, baseY: 0.013, baseZ: 0.022, target: 'elbow' },
  { mesh: gripperBar('Link_left_outer', 0.018), side: 1, baseY: 0.025, baseZ: 0.018, target: 'shoulder' },
  { mesh: gripperBar('Link_right_inner', -0.018), side: -1, baseY: 0.013, baseZ: 0.022, target: 'elbow' },
  { mesh: gripperBar('Link_right_outer', 0.018), side: -1, baseY: 0.025, baseZ: 0.018, target: 'shoulder' },
];

const clawSegments = [
  { mesh: gripperBar('Claw_left_upper', 0, 0.016, 0.009), side: 1, section: 'upper' },
  { mesh: gripperBar('Claw_left_tip', 0, 0.016, 0.009), side: 1, section: 'tip' },
  { mesh: gripperBar('Claw_right_upper', 0, 0.016, 0.009), side: -1, section: 'upper' },
  { mesh: gripperBar('Claw_right_tip', 0, 0.016, 0.009), side: -1, section: 'tip' },
];
gripper.add(padLeft, padRight, padFaceLeft, padFaceRight, ...linkageBars.map(({ mesh }) => mesh), ...clawSegments.map(({ mesh }) => mesh));

const fingerPivotLeft = pivotHardware('Finger_pivot_left');
const fingerPivotRight = pivotHardware('Finger_pivot_right');
const shoulderPivotLeft = pivotHardware('Shoulder_pivot_left', 0.0048);
const shoulderPivotRight = pivotHardware('Shoulder_pivot_right', 0.0048);
gripper.add(fingerPivotLeft, fingerPivotRight, shoulderPivotLeft, shoulderPivotRight);

const tcp = new THREE.Object3D();
tcp.name = 'TCP';
gripper.add(tcp);

const jointAxes = [s1, s2, s3, s4, s5].map((joint, index) => {
  const helper = new THREE.AxesHelper(index === 0 ? 0.065 : 0.045);
  joint.add(helper);
  return helper;
});

const jointGroups = { S1: s1, S2: s2, S3: s3, S4: s4, S5: s5 };
const currentTicks = Object.fromEntries(spec.joints.map((joint) => [joint.id, joint.home]));
let currentS5VisualOffset = spec.visualReferences?.home?.s5OffsetDegrees ?? 0;
let currentReferencePose = 'home';
let physicsWorld = null;
let activeCollisionOptions = {};

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function tickToDegrees(joint, tick) {
  if (joint.calibrationAnchors?.length >= 2) {
    const anchors = [...joint.calibrationAnchors].sort((a, b) => a.tick - b.tick);
    let lower = anchors[0];
    let upper = anchors[1];
    if (tick >= anchors.at(-1).tick) {
      lower = anchors.at(-2);
      upper = anchors.at(-1);
    } else if (tick > anchors[0].tick) {
      for (let index = 0; index < anchors.length - 1; index += 1) {
        if (tick <= anchors[index + 1].tick) {
          lower = anchors[index];
          upper = anchors[index + 1];
          break;
        }
      }
    }
    const progress = (tick - lower.tick) / (upper.tick - lower.tick);
    return THREE.MathUtils.lerp(lower.degrees, upper.degrees, progress);
  }
  if (!joint.ticksPerDegree) return null;
  return joint.direction * (tick - joint.zero) / joint.ticksPerDegree;
}

function gripperState(tick) {
  const openTick = spec.gripper.openTick;
  const closedTick = spec.gripper.closedTick;
  const closure = clamp((tick - openTick) / (closedTick - openTick), 0, 1);
  return {
    closure,
    gap: THREE.MathUtils.lerp(d.gripperOpenGap, d.gripperClosedGap, closure),
    length: THREE.MathUtils.lerp(d.gripperOpenLength, d.gripperClosedLength, closure),
  };
}

function updateGripper(tick) {
  const state = gripperState(tick);
  const joints = new Map();
  for (const side of [1, -1]) {
    joints.set(side, {
      shoulder: { y: side * (0.029 - 0.004 * state.closure), z: 0.041 },
      elbow: { y: side * (state.gap / 2 + 0.014), z: state.length - 0.035 },
      tip: { y: side * (state.gap / 2 + 0.002), z: state.length - 0.006 },
    });
  }

  for (const segment of clawSegments) {
    const { shoulder, elbow, tip } = joints.get(segment.side);
    const start = segment.section === 'upper' ? shoulder : elbow;
    const end = segment.section === 'upper' ? elbow : tip;
    setSegmentBetween(segment.mesh, start.y, start.z, end.y, end.z);
  }

  const left = joints.get(1);
  const right = joints.get(-1);
  padLeft.position.set(0, left.tip.y, left.tip.z);
  padRight.position.set(0, right.tip.y, right.tip.z);
  padFaceLeft.position.set(0, left.tip.y - 0.0022, left.tip.z);
  padFaceRight.position.set(0, right.tip.y + 0.0022, right.tip.z);
  fingerPivotLeft.position.set(0, left.elbow.y, left.elbow.z);
  fingerPivotRight.position.set(0, right.elbow.y, right.elbow.z);
  shoulderPivotLeft.position.set(0, left.shoulder.y, left.shoulder.z);
  shoulderPivotRight.position.set(0, right.shoulder.y, right.shoulder.z);

  for (const bar of linkageBars) {
    const startY = bar.side * bar.baseY;
    const startZ = bar.baseZ;
    const end = joints.get(bar.side)[bar.target];
    setSegmentBetween(bar.mesh, startY, startZ, end.y, end.z);
  }
  tcp.position.set(0, 0, state.length);
  return state;
}

function setSegmentBetween(mesh, startY, startZ, endY, endZ) {
  const deltaY = endY - startY;
  const deltaZ = endZ - startZ;
  mesh.scale.z = Math.hypot(deltaY, deltaZ);
  mesh.position.y = (startY + endY) / 2;
  mesh.position.z = (startZ + endZ) / 2;
  mesh.rotation.x = Math.atan2(-deltaY, deltaZ);
}

function updateModel() {
  for (const joint of spec.joints.slice(0, 5)) {
    const radians = THREE.MathUtils.degToRad(tickToDegrees(joint, currentTicks[joint.id]));
    if (joint.id === 'S1') jointGroups[joint.id].rotation.z = radians;
    else if (joint.id === 'S5') jointGroups[joint.id].rotation.z = radians + THREE.MathUtils.degToRad(currentS5VisualOffset);
    else jointGroups[joint.id].rotation.x = -radians;
  }
  const grip = updateGripper(currentTicks.S6);
  scene.updateMatrixWorld(true);
  const minimumZ = movingArmMinimumZ();
  updateReadout(grip, minimumZ);
  return minimumZ;
}

const tcpValue = document.querySelector('#tcp-value');
const clearanceValue = document.querySelector('#clearance-value');
const constraintValue = document.querySelector('#constraint-value');
const worldPosition = new THREE.Vector3();
const localBounds = new THREE.Box3();

function movingArmMinimumZ() {
  let minimumZ = Number.POSITIVE_INFINITY;
  s1.traverse((object) => {
    if (!object.isMesh || !object.geometry) return;
    if (!object.geometry.boundingBox) object.geometry.computeBoundingBox();
    localBounds.copy(object.geometry.boundingBox).applyMatrix4(object.matrixWorld);
    minimumZ = Math.min(minimumZ, localBounds.min.z);
  });
  return minimumZ;
}

function updateReadout(grip, minimumZ) {
  tcp.getWorldPosition(worldPosition);
  tcpValue.textContent = `${(worldPosition.x * 100).toFixed(1)}, ${(worldPosition.y * 100).toFixed(1)}, ${(worldPosition.z * 100).toFixed(1)} cm`;
  const clearance = worldPosition.z * 100;
  clearanceValue.textContent = clearance < 0
    ? `${Math.abs(clearance).toFixed(1)} cm below board`
    : `${clearance.toFixed(1)} cm · gap ${(grip.gap * 100).toFixed(1)} cm`;
  clearanceValue.dataset.warning = clearance < 0 ? 'true' : 'false';
  const fixtureTop = spec.fixture.upperPlate.top;
  const parkedOnFixture = currentReferencePose === 'park' && minimumZ >= fixtureTop - 0.004 && minimumZ <= fixtureTop + 0.006;
  constraintValue.textContent = minimumZ < 0
    ? `Below board · ${(Math.abs(minimumZ) * 100).toFixed(1)} cm`
    : parkedOnFixture
      ? `PARK rest · raised plate ${(fixtureTop * 100).toFixed(1)} cm`
      : `Clear · ${(minimumZ * 100).toFixed(1)} cm minimum`;
  constraintValue.dataset.warning = minimumZ < 0 ? 'true' : 'false';
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
    tryApplyTicks({ ...currentTicks, [joint.id]: Number(input.value) }, 0, null);
  });
  updateOutput(joint);
}

function updateOutput(joint) {
  const output = document.querySelector(`#output-${joint.id}`);
  const tick = currentTicks[joint.id];
  if (joint.id === 'S6') {
    output.textContent = `${tick} · ${(gripperState(tick).gap * 100).toFixed(1)} cm`;
  } else {
    const visualDegrees = tickToDegrees(joint, tick) + (joint.id === 'S5' ? currentS5VisualOffset : 0);
    output.textContent = `${tick} · ${visualDegrees.toFixed(1)}°${joint.id === 'S5' && currentS5VisualOffset ? ' photo' : ''}`;
  }
}

function syncControlValues() {
  for (const joint of spec.joints) {
    document.querySelector(`#joint-${joint.id}`).value = currentTicks[joint.id];
    updateOutput(joint);
  }
}

function tryApplyTicks(nextTicks, nextS5VisualOffset = currentS5VisualOffset, nextReferencePose = null) {
  const previousTicks = { ...currentTicks };
  const previousS5VisualOffset = currentS5VisualOffset;
  const previousReferencePose = currentReferencePose;
  Object.assign(currentTicks, nextTicks);
  currentS5VisualOffset = nextS5VisualOffset;
  currentReferencePose = nextReferencePose;
  const attemptedMinimumZ = updateModel();
  const coreCollision = collisionReport(modelSpec, nextTicks, { ignoreCubes: false, ...activeCollisionOptions });
  if (attemptedMinimumZ < 0 || !coreCollision.valid) {
    Object.assign(currentTicks, previousTicks);
    currentS5VisualOffset = previousS5VisualOffset;
    currentReferencePose = previousReferencePose;
    updateModel();
    syncControlValues();
    const contact = coreCollision.contacts[0];
    constraintValue.textContent = attemptedMinimumZ < 0
      ? `Blocked · ${(Math.abs(attemptedMinimumZ) * 100).toFixed(1)} cm below board`
      : `Blocked · ${contact.a} × ${contact.b} (${contact.clearance.toFixed(1)} mm)`;
    constraintValue.dataset.warning = 'true';
    return false;
  }
  physicsWorld?.setRobotPose(currentTicks);
  syncControlValues();
  return true;
}

let poseAnimation;
function animateTo(target, duration = 650, referenceName = null, onComplete = null) {
  if (poseAnimation) cancelAnimationFrame(poseAnimation);
  const from = { ...currentTicks };
  const fromS5VisualOffset = currentS5VisualOffset;
  const targetS5VisualOffset = spec.visualReferences?.[referenceName]?.s5OffsetDegrees ?? 0;
  const reduceMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  const started = performance.now();
  const effectiveDuration = reduceMotion ? 0 : duration;

  function frame(now) {
    const progress = effectiveDuration === 0 ? 1 : clamp((now - started) / effectiveDuration, 0, 1);
    const eased = 1 - Math.pow(1 - progress, 3);
    const nextTicks = {};
    for (const joint of spec.joints) {
      nextTicks[joint.id] = Math.round(THREE.MathUtils.lerp(from[joint.id], target[joint.id], eased));
    }
    const nextS5VisualOffset = THREE.MathUtils.lerp(fromS5VisualOffset, targetS5VisualOffset, eased);
    const nextReferencePose = progress === 1 ? referenceName : null;
    const accepted = tryApplyTicks(nextTicks, nextS5VisualOffset, nextReferencePose);
    if (accepted && progress < 1) {
      poseAnimation = requestAnimationFrame(frame);
    } else {
      poseAnimation = undefined;
      if (accepted && progress === 1) onComplete?.();
    }
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
  button.addEventListener('click', () => {
    if (button.dataset.preset === 'park') runReferencePose('park');
    else if (button.dataset.preset === 'home') runReferencePose('home');
    else animateToHome(() => animateTo(preset(button.dataset.preset), 650, button.dataset.preset));
  });
});

function clearanceTick() {
  return spec.joints.find((joint) => joint.id === 'S4').min;
}

function animateToPark() {
  const park = preset('park');
  const raisedCurrent = { ...currentTicks, S4: clearanceTick() };
  const raisedOpenCurrent = { ...raisedCurrent, S6: 490 };
  const raisedOpenPark = { ...park, S4: clearanceTick(), S6: 490 };
  const raisedClosedPark = { ...raisedOpenPark, S6: park.S6 };
  animateTo(raisedCurrent, 300, null, () => {
    animateTo(raisedOpenCurrent, 250, null, () => {
      animateTo(raisedOpenPark, 500, null, () => {
        animateTo(raisedClosedPark, 250, null, () => animateTo(park, 450, 'park'));
      });
    });
  });
}

function animateToHome(onComplete = null) {
  const home = preset('home');
  const raisedCurrent = { ...currentTicks, S4: clearanceTick() };
  const raisedOpenCurrent = { ...raisedCurrent, S6: 490 };
  const raisedOpenHome = { ...home, S4: clearanceTick(), S6: 490 };
  const openHome = { ...home, S6: 490 };
  animateTo(raisedCurrent, 300, null, () => {
    animateTo(raisedOpenCurrent, 250, null, () => {
      animateTo(raisedOpenHome, 500, null, () => {
        animateTo(openHome, 450, 'home', onComplete);
      });
    });
  });
}

document.querySelector('#play-transition').addEventListener('click', () => {
  runHomeParkVerification();
});

const physicsLabel = document.querySelector('#physics-label');
try {
  const { PhysicsWorld } = await import('./core/physics.js');
  physicsWorld = await new PhysicsWorld(modelSpec).initialize(currentTicks);
  physicsLabel.textContent = 'Rapier3D ready';
} catch (error) {
  physicsLabel.textContent = 'Physics unavailable';
  physicsLabel.title = error.message;
}

const trajectoryPlayer = new TrajectoryPlayer(modelSpec, (ticks) => tryApplyTicks(ticks));
const robotClient = new RobotArmClient({ apiBase: modelSpec.operational.sameOriginApiBase });
const planResult = document.querySelector('#plan-result');
const runPlanButton = document.querySelector('#run-plan');
const modeResult = document.querySelector('#mode-result');
const modeSelect = document.querySelector('#mode-select');
const modeLabel = document.querySelector('#mode-label');
const modeBadge = document.querySelector('#mode-badge');
const statusDot = document.querySelector('#status-dot');
const armRealButton = document.querySelector('#arm-real');
const powerOffButton = document.querySelector('#power-off');
const parkConfirm = document.querySelector('#park-confirm');
let currentPlan = null;
let currentPlanReference = null;

function describePlan(plan) {
  if (!plan?.success) return plan?.reason ?? 'Planning failed.';
  const metrics = plan.metrics;
  const cost = plan.cost ?? pathCost(modelSpec, plan.path);
  return `${plan.algorithm}. ${plan.path.length} waypoints · ${metrics.countTravel.toFixed(0)} count travel · ${metrics.changedServos} servo adjustments · ${(metrics.tcpLength / 10).toFixed(1)} cm TCP travel · ${metrics.minimumClearance.toFixed(1)} mm minimum clearance · cost ${cost.toFixed(0)}.`;
}

function publishPlan(plan, referenceName = null) {
  currentPlan = plan?.success ? plan : null;
  currentPlanReference = referenceName;
  planResult.textContent = describePlan(plan);
  planResult.classList.toggle('error', !plan?.success);
  runPlanButton.disabled = !plan?.success;
}

async function executePlan(plan = currentPlan, referenceName = currentPlanReference) {
  if (!plan?.success) return;
  try {
    let playbackPath = plan.path;
    if (document.querySelector('#playback-mode').value === 'single' || robotClient.mode === 'operational-armed') {
      const converted = controllerCompatiblePath(modelSpec, playbackPath);
      if (!converted.success) throw new Error(converted.reason);
      playbackPath = converted.path;
    }
    if (robotClient.mode === 'operational-armed') {
      await robotClient.executeSingleJointPath(modelSpec, playbackPath);
      modeResult.textContent = 'Verified controller-compatible commands accepted. Pose remains commanded, not measured.';
    } else if (robotClient.mode !== 'operational-disarmed') {
      const firedEvents = new Set();
      if (plan.task) activeCollisionOptions = { allowedGripperContactCubeIds: [plan.task.cube] };
      try {
        await trajectoryPlayer.play(playbackPath, { onWaypoint: (_index, pose) => {
          for (const [eventIndex, event] of (plan.events ?? []).entries()) {
            if (firedEvents.has(eventIndex)) continue;
            const reached = modelSpec.joints.every((joint) => pose[joint.id] === event.counts[joint.id]);
            if (!reached) continue;
            if (event.type === 'grasp') {
              activeCollisionOptions = { removedCubeIds: [event.cube], carriedCubeId: event.cube };
              physicsWorld?.graspCube(event.cube, pose);
            } else if (event.type === 'allow-place-support') {
              activeCollisionOptions = { removedCubeIds: [event.cube], carriedCubeId: event.cube, allowCarriedBoardSupport: true };
            } else {
              const size = modelSpec.dimensions.testCube;
              activeCollisionOptions = { removedCubeIds: [event.cube], extraBoxes: [{ id: `${event.cube} cube (placed)`, cubeId: event.cube, center: plan.task.destination, size: [size, size, size] }] };
              modelSpec.world.cubes.find((cube) => cube.id === event.cube).position = [...plan.task.destination];
              physicsWorld?.releaseCube();
            }
            firedEvents.add(eventIndex);
          }
        } });
      } finally {
        activeCollisionOptions = {};
      }
    } else {
      throw new Error('Operational mode is disarmed. Return to Simulation for virtual playback or explicitly arm the backend.');
    }
    if (referenceName) {
      const visualOffset = spec.visualReferences?.[referenceName]?.s5OffsetDegrees ?? 0;
      tryApplyTicks(playbackPath.at(-1), visualOffset, referenceName);
    }
  } catch (error) {
    planResult.textContent = `Playback stopped: ${error.message}`;
    planResult.classList.add('error');
  }
}

async function runReferencePose(name) {
  planResult.textContent = `Planning verified ${name.toUpperCase()} transition…`;
  await new Promise((resolve) => setTimeout(resolve, 16));
  const plan = planReferenceTransition(modelSpec, { ...currentTicks }, name);
  publishPlan(plan, name);
  if (plan.success) await executePlan(plan, name);
}

async function runHomeParkVerification() {
  await runReferencePose('home');
  if (currentReferencePose === 'home') await runReferencePose('park');
}

document.querySelector('#plan-task').addEventListener('click', async () => {
  planResult.textContent = 'Solving IK and optimizing a collision-verified trajectory…';
  planResult.classList.remove('error');
  runPlanButton.disabled = true;
  await new Promise((resolve) => setTimeout(resolve, 16));
  try {
    publishPlan(planTextTask(modelSpec, { ...currentTicks }, document.querySelector('#task-input').value));
  } catch (error) {
    publishPlan({ success: false, reason: error.message });
  }
});

runPlanButton.addEventListener('click', () => executePlan());
document.querySelector('#reset-objects').addEventListener('click', () => {
  for (const cube of modelSpec.world.cubes) cube.position = [...initialCubePositions[cube.id]];
  physicsWorld?.resetCubes();
});

function renderOperatingMode(message = '') {
  const labels = {
    simulation: 'Simulation',
    'connected-read-only': 'Connected read-only',
    'operational-disarmed': 'Operational · disarmed',
    'operational-armed': 'Operational · ARMED',
    'fault-stopped': 'FAULT / stopped',
  };
  modeLabel.textContent = labels[robotClient.mode];
  modeBadge.textContent = robotClient.mode.replaceAll('-', ' ').toUpperCase();
  modeBadge.className = `badge ${robotClient.mode === 'operational-armed' ? 'danger' : robotClient.mode === 'simulation' ? 'safe' : ''}`;
  statusDot.dataset.mode = robotClient.mode;
  armRealButton.disabled = robotClient.mode !== 'operational-disarmed' || !parkConfirm.checked;
  powerOffButton.disabled = !['operational-armed', 'operational-disarmed'].includes(robotClient.mode);
  if (message) modeResult.textContent = message;
}

function commandedCountsFromBackend() {
  const commands = robotClient.state?.commands ?? {};
  const counts = Object.fromEntries(modelSpec.joints.map((joint) => [joint.id, commands[Number(joint.id.slice(1)) - 1]]));
  return Object.values(counts).every(Number.isInteger) ? counts : null;
}

modeSelect.addEventListener('change', async () => {
  try {
    if (modeSelect.value === 'simulation') {
      robotClient.returnToSimulation();
      renderOperatingMode('Virtual-only mode. No robot API commands can be sent.');
      return;
    }
    if (robotClient.mode === 'simulation' || robotClient.mode === 'fault-stopped') {
      await robotClient.connectReadOnly();
      const commanded = commandedCountsFromBackend();
      if (commanded) tryApplyTicks(commanded);
    }
    if (modeSelect.value === 'operational-disarmed') robotClient.enterOperationalDisarmed();
    renderOperatingMode(robotClient.mode === 'connected-read-only'
      ? `Read-only API connected (${robotClient.state.mode}). No hardware preparation or movement was requested.`
      : 'Operational mode selected but disarmed. Fresh supported PARK confirmation is still required.');
  } catch (error) {
    modeSelect.value = 'simulation';
    robotClient.returnToSimulation();
    renderOperatingMode(error.message);
  }
});

parkConfirm.addEventListener('change', () => renderOperatingMode());
armRealButton.addEventListener('click', async () => {
  try {
    armRealButton.disabled = true;
    modeResult.textContent = 'Backend is enabling PARK and moving to HOME…';
    await robotClient.arm({ parkConfirmed: parkConfirm.checked });
    const commanded = commandedCountsFromBackend();
    if (commanded) tryApplyTicks(commanded, spec.visualReferences.home.s5OffsetDegrees, 'home');
    renderOperatingMode('Operational mode ARMED. Only verified, single-servo controller playback is permitted.');
  } catch (error) { renderOperatingMode(`Arming refused: ${error.message}`); }
});

powerOffButton.addEventListener('click', async () => {
  try {
    await robotClient.powerOff();
    parkConfirm.checked = false;
    modeSelect.value = 'connected-read-only';
    renderOperatingMode('Backend returned commanded pose to PARK and disabled outputs.');
  } catch (error) { renderOperatingMode(`Power-off failed: ${error.message}. Use the physical servo power switch.`); }
});

document.querySelector('#emergency-stop').addEventListener('click', async () => {
  trajectoryPlayer.stop();
  if (robotClient.mode === 'simulation') {
    planResult.textContent = 'Virtual playback stopped.';
    return;
  }
  try { await robotClient.emergencyStop(); modeSelect.value = 'simulation'; renderOperatingMode('Emergency stop requested; backend outputs disabled. Verify the physical servo power switch.'); }
  catch (error) { renderOperatingMode(`Stop request failed: ${error.message}. Use the physical servo power switch now.`); }
});

renderOperatingMode();

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
  if (physicsWorld?.ready) {
    physicsWorld.step();
    for (const [id, transform] of Object.entries(physicsWorld.cubeTransforms())) {
      const mesh = cubeMeshes.get(id);
      mesh.position.set(...transform.position);
      mesh.quaternion.set(...transform.rotation);
    }
  }
  orbit.update();
  renderer.render(scene, camera);
  requestAnimationFrame(render);
}

updateModel();
render();

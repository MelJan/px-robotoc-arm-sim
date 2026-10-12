// Page glue: rendering, joint control, tasks, scene editing, live link, calibration.

import * as THREE from 'three';
import { createView } from './render.js';
import { SERVOS, MAT, CAMERA, CUBE, COLOURS, PLAN } from './config.js';
import { fk, anglesFromCounts, countsFromAngles, ikTopDown, guardViolations, camToBoard, toolYawAndTilt, Unreachable } from './kinematics.js';
import { armShapes } from './collision.js';
import { SimController, ControllerError } from './controller.js';
import { World } from './world.js';
import { cubeTilt } from './physics.js';
import { planTransfer, verify, toMoves, parseCommand, PlanError } from './planner.js';
import { LiveLink } from './live.js';
import { makeMaterials, buildArm, buildTable, buildCamera, buildCube } from './model.js';
import { initCameraTab, defaultCalibration, applyPinhole, tableToBoard } from './camview.js';

const $ = id => document.getElementById(id);
const fmt = (v, d = 1) => v.toFixed(d).replace('-0.0', '0.0');

// ---------------- calibration (editable, saved per browser) ----------------

const CAL_KEY = 'pxsim.calibration.v1';
const DEFAULT_CAL = { servos: SERVOS.slice(0, 5).map(s => ({ qHome: s.qHome, dir: s.dir, cpd: s.cpd })), d: MAT.sToEdge };
function applyCal(cal) {
  cal.servos.forEach((c, i) => Object.assign(SERVOS[i], c));
  MAT.sToEdge = cal.d;
}
try { const saved = JSON.parse(localStorage.getItem(CAL_KEY)); if (saved?.servos?.length === 5) applyCal(saved); } catch { /* defaults */ }
const currentCal = () => ({ servos: SERVOS.slice(0, 5).map(s => ({ qHome: s.qHome, dir: s.dir, cpd: s.cpd })), d: MAT.sToEdge });

// ---------------- sim state ----------------

const ctl = new SimController();
const world = new World();
const DEFAULT_SCENE = [['orange', 10.8, 14.0, 0, 3], ['blue', -8, 12, 0, 0], ['yellow', 3, 10, 0, 0], ['green', -4, 17, 0, 10]];
function resetScene() {
  world.cubes = []; world.holding = null;
  for (const [c, x, y, l, yaw] of DEFAULT_SCENE) world.addCube(c, c, x, y, l, yaw);
}
resetScene();

// The team's hand-taught pick and place (docs/controller-live-2026-10-12.md).
const TAUGHT = { name: 'step1-hacks-10-direct-approach', positions: [
  ['HOME start', [320, 427, 403, 220, 250, 550]], ['Above orange, open', [440, 427, 360, 185, 340, 490]],
  ['Single approach, open', [440, 347, 360, 185, 340, 490]], ['Close gripper', [440, 347, 360, 185, 340, 590]],
  ['Lift cube', [440, 427, 360, 185, 340, 590]], ['Transfer', [230, 427, 360, 185, 340, 590]],
  ['Lower to destination', [235, 347, 360, 185, 380, 590]], ['Release cube', [235, 347, 360, 185, 380, 490]],
  ['Withdraw', [235, 407, 360, 185, 380, 490]], ['HOME finish', [320, 427, 403, 220, 250, 550]]].map(([name, counts]) => ({ name, counts })) };

// ---------------- renderer ----------------

const canvas = $('canvas');
const view = createView(canvas);
const { renderer, scene, camera, controls } = view;
camera.position.set(-30, 50, 30);
controls.target.set(0, 11, 7);
controls.update();

const M = makeMaterials();
const table = buildTable(M); scene.add(table);
const backdrop = table.getObjectByName('backdrop');
const arm = buildArm(M); scene.add(arm.root);

// ghost of the real arm (live mirror)
const ghostMat = new THREE.MeshStandardMaterial({ color: 0x18a38f, transparent: true, opacity: 0.16, depthWrite: false });
const ghost = buildArm(new Proxy({}, { get: () => ghostMat }));
ghost.root.traverse(o => { o.castShadow = false; o.receiveShadow = false; });
for (const c of ghost.root.children) if (c.isMesh) c.visible = false;   // the base never moves; draw only the moving links
ghost.root.visible = false; scene.add(ghost.root);

// The OAK camera: fitted to the mat markers (camview.js); the fit is kept per browser.
const CAM_KEY = 'pxsim.camera.v1';
CAMERA.calib = defaultCalibration();
try { const c = JSON.parse(localStorage.getItem(CAM_KEY)); if (c && c.R && c.f) CAMERA.calib = c; } catch { /* default fit */ }
let oak = null;
function camPose() {
  const cal = CAMERA.calib, C = cal.centre, z = cal.R[2];
  const s = -C[2] / z[2];                                   // optical axis meets the mat
  return { pos: tableToBoard(C), target: tableToBoard([C[0] + s * z[0], C[1] + s * z[1], 0]) };
}
/** Render with the OAK camera's own body hidden (it would block its view). */
const hideForCamera = fn => { const v = oak.visible; oak.visible = false; fn(); oak.visible = v; };
function placeMatAndCamera() {
  table.userData.placeMat(MAT.sToEdge);
  if (oak) scene.remove(oak);
  const p = camPose(); oak = buildCamera(M, p.pos, p.target); scene.add(oak);
}
placeMatAndCamera();

// cubes
const cubeMeshes = new Map();
function syncCubeMeshes() {
  for (const [id, m] of cubeMeshes) if (!world.cube(id)) { scene.remove(m); cubeMeshes.delete(id); }
  for (const c of world.cubes) {
    let m = cubeMeshes.get(c.id);
    if (!m) { m = buildCube(c.colour); cubeMeshes.set(c.id, m); scene.add(m); }
    m.position.set(...c.p);
    if (c.R) m.setRotationFromMatrix(new THREE.Matrix4().set(c.R[0], c.R[1], c.R[2], 0, c.R[3], c.R[4], c.R[5], 0, c.R[6], c.R[7], c.R[8], 0, 0, 0, 0, 1));
    else { m.rotation.set(0, 0, c.yaw * Math.PI / 180); }
  }
}

// collision shape overlay
const shapeGroup = new THREE.Group(); scene.add(shapeGroup);
const shapeMeshes = new Map();
const wireOk = new THREE.LineBasicMaterial({ color: 0x0e8a77, transparent: true, opacity: 0.7 });
const wireHit = new THREE.LineBasicMaterial({ color: 0xe0281e });
const unitBox = new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1));
function updateShapes(shapes, hitNames, showAll) {
  const used = new Set();
  for (const s of shapes) {
    const hit = hitNames.has(s.name);
    if (!showAll && !hit) continue;
    used.add(s.name);
    let m = shapeMeshes.get(s.name);
    if (!m) {
      if (s.kind === 'box') m = new THREE.LineSegments(unitBox, wireOk);
      else {
        const len = Math.hypot(s.b[0] - s.a[0], s.b[1] - s.a[1], s.b[2] - s.a[2]);
        m = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.CapsuleGeometry(s.r, len, 4, 10), 30), wireOk);
      }
      shapeMeshes.set(s.name, m); shapeGroup.add(m);
    }
    m.visible = true;
    m.material = hit ? wireHit : wireOk;
    if (s.kind === 'box') {
      m.position.set(...s.c); m.scale.set(2 * s.h[0], 2 * s.h[1], 2 * s.h[2]);
      const R = s.R;
      m.setRotationFromMatrix(new THREE.Matrix4().set(R[0], R[1], R[2], 0, R[3], R[4], R[5], 0, R[6], R[7], R[8], 0, 0, 0, 0, 1));
    } else {
      m.position.set((s.a[0] + s.b[0]) / 2, (s.a[1] + s.b[1]) / 2, (s.a[2] + s.b[2]) / 2);
      m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(s.b[0] - s.a[0], s.b[1] - s.a[1], s.b[2] - s.a[2]).normalize());
    }
  }
  for (const [n, m] of shapeMeshes) if (!used.has(n)) m.visible = false;
}

// planned path
const pathLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xff7a1a }));
pathLine.frustumCulled = false; scene.add(pathLine);
const pathDots = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ color: 0xff7a1a, size: 0.45 }));
pathDots.frustumCulled = false; scene.add(pathDots);
function showPath(steps, start) {
  const pts = [], dots = [];
  let prev = start;
  for (const s of steps) {
    for (let k = 1; k <= 6; k++) {
      const c = prev.map((v, i) => v + (s.counts[i] - v) * k / 6);
      pts.push(...fk(anglesFromCounts(c)).grasp);
    }
    dots.push(...fk(anglesFromCounts(s.counts)).grasp);
    prev = s.counts;
  }
  pathLine.geometry.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
  pathDots.geometry.setAttribute('position', new THREE.Float32BufferAttribute(dots, 3));
  pathLine.geometry.computeBoundingSphere();
  needsRender = true;
}

// ---------------- views ----------------

const VIEWS = {
  orbit: () => { camera.fov = 36; camera.position.set(-30, 50, 30); controls.target.set(0, 11, 7); },
  top: () => { camera.fov = 36; camera.position.set(0, 15.5, 95); controls.target.set(0, 15.5, 0); },
  side: () => { camera.fov = 36; camera.position.set(75, 12, 14); controls.target.set(0, 12, 12); },
  oak: () => { oakView = true; controls.enabled = false; applyOakView(); },
};
let oakView = false;
function applyOakView() {
  const r = canvas.getBoundingClientRect();
  applyPinhole(camera, CAMERA.calib, r.width, r.height);
  controls.target.set(...camPose().target);
}
function leaveOakView() {
  if (!oakView) return;
  oakView = false; controls.enabled = true;
  camera.fov = 36; camera.updateProjectionMatrix();
  document.querySelectorAll('[data-view]').forEach(x => x.classList.toggle('on', x.dataset.view === 'orbit'));
}
document.querySelectorAll('[data-view]').forEach(b => b.onclick = () => {
  document.querySelectorAll('[data-view]').forEach(x => x.classList.toggle('on', x === b));
  if (b.dataset.view !== 'oak') { leaveOakView(); VIEWS[b.dataset.view](); camera.updateProjectionMatrix(); controls.update(); }
  else VIEWS.oak();
  needsRender = true;
});
// dragging in the OAK view leaves it and orbits from where the camera is
canvas.addEventListener('pointerdown', () => { if (oakView) { leaveOakView(); controls.update(); } }, true);

function resize() { view.resize(); if (oakView) applyOakView(); needsRender = true; }
new ResizeObserver(resize).observe(canvas.parentElement);

// ---------------- resizable sidebar ----------------

{
  const KEY = 'pxsim.panelWidth', panel = $('panel'), split = $('splitter');
  const setW = w => { const px = Math.round(Math.max(300, Math.min(innerWidth * 0.75, w))); document.documentElement.style.setProperty('--panel-w', px + 'px'); return px; };
  try { const w = Number(localStorage.getItem(KEY)); if (w) setW(w); } catch { /* default width */ }
  split.addEventListener('pointerdown', e => {
    try { split.setPointerCapture(e.pointerId); } catch { /* synthetic events */ }
    split.classList.add('drag'); document.body.classList.add('resizing');
    const move = ev => setW(innerWidth - ev.clientX - split.offsetWidth / 2);
    const up = () => {
      split.removeEventListener('pointermove', move); split.classList.remove('drag'); document.body.classList.remove('resizing');
      try { localStorage.setItem(KEY, String(panel.getBoundingClientRect().width)); } catch { /* not saved */ }
    };
    split.addEventListener('pointermove', move);
    split.addEventListener('pointerup', up, { once: true });
  });
  split.addEventListener('dblclick', () => { document.documentElement.style.removeProperty('--panel-w'); try { localStorage.removeItem(KEY); } catch { /* ignore */ } });
}

// ---------------- log and toast ----------------

function log(text, cls = '') {
  const d = document.createElement('div');
  d.className = cls;
  d.textContent = `${new Date().toLocaleTimeString()}  ${text}`;
  $('log').prepend(d);
  while ($('log').childNodes.length > 300) $('log').lastChild.remove();
}
let toastTimer = null;
function toast(text, ms = 4000) {
  $('toast').textContent = text; $('toast').hidden = false;
  clearTimeout(toastTimer); toastTimer = setTimeout(() => { $('toast').hidden = true; }, ms);
}
ctl.on((kind, detail) => {
  if (kind === 'stop') log(`STOP: ${detail}`, 'err');
  else if (kind === 'power') log(`power ${detail}`);
});

// ---------------- joints panel ----------------

const jointRows = [];
for (const s of SERVOS) {
  const row = document.createElement('div');
  row.className = 'joint';
  row.innerHTML = `<div class="name">${s.name}<small>${s.joint}</small></div>
    <input type="range" min="${s.low}" max="${s.high}" step="1">
    <input type="number" min="${s.low}" max="${s.high}" step="1">
    <button title="jog −">−</button><button title="jog +">+</button><div class="deg"></div>`;
  const [range, num] = row.querySelectorAll('input');
  const [minus, plus] = row.querySelectorAll('button');
  const r = { range, num, deg: row.querySelector('.deg'), dragging: false };
  range.addEventListener('pointerdown', () => { r.dragging = true; });
  range.addEventListener('input', () => { num.value = range.value; });
  range.addEventListener('change', () => { r.dragging = false; command('move', s.ch, Number(range.value)); });
  num.addEventListener('change', () => command('move', s.ch, Math.round(Number(num.value))));
  minus.onclick = () => command('jog', s.ch, -1);
  plus.onclick = () => command('jog', s.ch, 1);
  $('joints').append(row);
  jointRows.push(r);
}
$('step').onchange = () => { ctl.settings.step_size = Math.max(1, Math.round(Number($('step').value))); };

function updateJointPanel(counts) {
  const a = anglesFromCounts(counts);
  const deg = [a.s1, a.s2, a.s3, a.s4, a.s5];
  counts.forEach((c, i) => {
    const r = jointRows[i];
    if (!r.dragging && document.activeElement !== r.num) { r.range.value = Math.round(c); r.num.value = Math.round(c); }
    r.deg.textContent = i < 5 ? `model angle ${fmt(deg[i])}°` : `gap ${fmt(world.gap, 2)} cm${world.holding ? ` · holding ${world.holding.id}` : ''}`;
  });
}

/** True when the joint panel and pose buttons drive the real arm. */
const liveControl = () => link && link.state && $('live-enable').checked;

/** Sim check of one joint move before sending it anywhere: does the swept path hit something? */
function moveIsClear(from, to) {
  const snap = world.snapshot();
  let hit = null;
  for (let k = 1; k <= 12 && !hit; k++) {
    world.step(0, from.map((v, i) => v + (to[i] - v) * k / 12));
    if (world.contacts.length) hit = world.contacts[0];
  }
  world.restore(snap);
  world.step(0, from);
  return hit;
}

async function command(kind, ch, value) {
  try {
    if (liveControl()) {
      const cur = LiveLink.counts(link.state);
      const target = kind === 'jog' ? cur[ch] + value * ctl.settings.step_size : value;
      const to = cur.slice(); to[ch] = target;
      const hit = moveIsClear(cur, to);
      if (hit) throw new Error(`sim predicts a collision: ${hit.a} × ${hit.b} — not sent`);
      await link.move(ch, target);
      log(`arm: ${SERVOS[ch].name} → ${target}`, 'warn');
    } else if (kind === 'jog') ctl.jog(ch, value);
    else ctl.move(ch, value);
  } catch (e) { toast(e.message); log(e.message, 'err'); }
}

const poseCmd = name => async () => {
  try {
    if (liveControl()) {
      const cur = LiveLink.counts(link.state), to = link.state.joints.map(j => j[name.toLowerCase()]);
      const hit = moveIsClear(cur, to);
      if (hit) throw new Error(`sim predicts a collision on the way to ${name}: ${hit.a} × ${hit.b} — not sent`);
      await link.goPose(name); log(`arm: go ${name}`, 'warn');
    } else ctl.go_pose(name);
  } catch (e) { toast(e.message); log(e.message, 'err'); }
};
$('go-home').onclick = poseCmd('HOME');
$('go-park').onclick = poseCmd('PARK');
$('power-on').onclick = () => { ctl.rearm(); lastContacts = new Set(world.contacts.map(k => `${k.a} × ${k.b}`)); };
$('estop').onclick = async () => {
  ctl.stop('emergency stop button');
  if (link) { try { await link.emergencyStop(); log('arm: STOP sent', 'err'); } catch (e) { log(`arm stop failed: ${e.message}`, 'err'); } }
};

// ---------------- task panel ----------------

let current = null; // { plan, report, moves, start, mode }

const startCounts = () => (liveControl() ? LiveLink.counts(link.state) : ctl.commands.slice());

function describe(report) {
  const parts = [];
  parts.push(report.ok ? '<span class="ok">✓ checked in simulation</span>' : '<span class="bad">✗ does not work in simulation</span>');
  parts.push(`${fmt(report.duration, 0)} s`);
  if (report.contacts.length) parts.push(`<span class="bad">${report.contacts.map(c => `${c.a} × ${c.b} (${fmt(-c.sep * 10, 1)} mm, at "${c.step}")`).join('; ')}</span>`);
  const warns = report.events.filter(e => e.kind === 'warn').map(e => e.text);
  if (warns.length) parts.push(`<span class="warn">${warns.join('; ')}</span>`);
  if (!report.contacts.length && !report.ok) parts.push(`<span class="bad">cube ends ${fmt(report.offset, 2)} cm from the target</span>`);
  return parts.join(' · ');
}

function renderPlan() {
  const { plan, report, mode, notes } = current;
  const grasp = report.events.find(e => e.kind === 'grasp');
  const rows = plan.steps.map((s, i) => `<tr data-i="${i}"><td>${s.label}</td>${s.counts.map(c => `<td>${c}</td>`).join('')}</tr>`).join('');
  $('plan-out').innerHTML = `<div>${plan.steps.length} poses, hover ${plan.info.hover} cm${plan.info.vias ? `, ${plan.info.vias} transit via${plan.info.vias > 1 ? 's' : ''} around stacks` : ''}, ${mode === 'moves' ? `${current.moves.length} single-joint moves` : 'demo playback'}.</div>
    <div>${describe(report)}</div>
    ${grasp ? `<div class="note">${grasp.text}</div>` : ''}
    ${notes?.length ? `<div class="warn">${notes.join('; ')}</div>` : ''}
    <div class="steps"><table><tr><th>Step</th>${SERVOS.map(s => `<th>${s.name}</th>`).join('')}</tr>${rows}</table></div>`;
  $('run-sim').disabled = false;
  $('export-plan').disabled = false;
  $('send-live').disabled = !(liveControl() && report.ok && mode === 'moves');
  $('send-live').title = mode !== 'moves' ? 'Choose "Single-joint moves": the HTTP API moves one joint at a time' : 'Needs a live connection with control enabled';
}

function makePlan() {
  try {
    const mode = $('exec-mode').value;
    const speed = Math.max(1, Math.min(100, Number($('demo-speed').value) || 20));
    const start = startCounts();
    const parsed = parseCommand($('cmd').value, world);
    const plan = planTransfer(world, parsed, start, { home: ctl.poses.HOME, step: mode === 'moves' ? 0.25 : PLAN.descentStep });
    let moves = null, notes = null;
    if (mode === 'moves') ({ moves, notes } = toMoves(world, plan, start));
    const report = verify(world, plan, start, { mode, speed, moves });
    current = { plan, report, moves, notes, start, mode, speed };
    renderPlan();
    showPath(plan.steps, start);
    log(`plan "${$('cmd').value}": ${plan.steps.length} poses, ${report.ok ? 'OK' : 'FAILS'} in sim`, report.ok ? 'ok' : 'err');
  } catch (e) {
    current = null;
    $('plan-out').innerHTML = `<span class="bad">${e.message}</span>`;
    $('run-sim').disabled = $('send-live').disabled = $('export-plan').disabled = true;
    if (!(e instanceof PlanError)) console.error(e);
  }
}
$('plan-btn').onclick = makePlan;
$('cmd').addEventListener('keydown', e => { if (e.key === 'Enter') makePlan(); });
$('exec-mode').onchange = () => { if (current) makePlan(); };

$('run-sim').onclick = () => {
  if (!current) return;
  try {
    if (ctl.commands.some((c, i) => c !== current.start[i])) { makePlan(); if (!current) return; }
    if (current.mode === 'moves') ctl.runMoves(current.moves, current.moves.map(m => m.label));
    else ctl.runPoses(current.plan.steps.map(s => s.counts), current.speed, current.plan.steps.map(s => s.label));
    world.events = [];
  } catch (e) { toast(e.message); }
};

$('export-plan').onclick = async () => {
  if (!current) return;
  const json = JSON.stringify({ name: $('cmd').value, units: 'PCA9685_counts', nominal_frequency_hz: 60,
    positions: current.plan.steps.map(s => ({ name: s.label, counts: Object.fromEntries(s.counts.map((c, i) => [String(i), c])) })) }, null, 1);
  try { await navigator.clipboard.writeText(json); toast('Plan copied as demo JSON'); } catch { console.log(json); toast('Clipboard blocked; JSON printed to the console'); }
};

$('send-live').onclick = async () => {
  if (!current || !liveControl()) return;
  // Re-plan from the arm's real commanded pose right before sending.
  $('exec-mode').value = 'moves'; makePlan();
  if (!current?.report.ok) { toast('The plan does not pass the simulation check; nothing sent'); return; }
  if (!confirm(`Send ${current.moves.length} single-joint moves to the real arm at ${link.base || location.host}?\nStay at the emergency stop.`)) return;
  $('live-follow').checked = true;
  log(`arm: sending ${current.moves.length} moves`, 'warn');
  try {
    await link.runMoves(current.moves, { onStep: (i, m) => { $('hud').textContent = `arm: ${i + 1}/${current.moves.length} ${m.label}`; } });
    log('arm: plan finished', 'ok');
  } catch (e) { log(`arm: ${e.message}`, 'err'); toast(e.message); }
};

$('goto-xyz').onclick = () => {
  try {
    const s5 = anglesFromCounts(ctl.commands).s5;
    const a = ikTopDown(Number($('gx').value), Number($('gy').value), Number($('gz').value), { s5Near: s5 });
    const c = countsFromAngles(a, ctl.commands[5]);
    const bad = guardViolations(c); if (bad.length) throw new Error(bad.join(', '));
    ctl.runPoses([c], Math.max(1, Number($('demo-speed').value) || 20), ['go to x y z']);
  } catch (e) { toast(e instanceof Unreachable ? `Unreachable: ${e.message}` : e.message); }
};

// taught demo button
{
  const b = document.createElement('button');
  b.textContent = "Play the team's taught demo";
  b.title = 'step1-hacks-10-direct-approach, hand-taught on the controller (docs/controller-live-2026-10-12.md)';
  b.onclick = () => { try { ctl.demo_load(TAUGHT.name, TAUGHT.positions); ctl.settings.demo_speed = Number($('demo-speed').value) || 40; ctl.demo_run(); showPath(TAUGHT.positions, ctl.commands.slice()); } catch (e) { toast(e.message); } };
  $('export-plan').after(b);
}

// ---------------- scene panel ----------------

let placing = null;
function renderCubes() {
  $('cubes').innerHTML = '';
  for (const c of world.cubes) {
    const row = document.createElement('div');
    row.className = 'cube-row';
    const lvl = world.levelOf(c);
    row.innerHTML = `<span class="dot" style="background:#${new THREE.Color(COLOURS[c.colour] ?? 0x999999).getHexString()}"></span>
      <span><b>${c.id}</b> <span class="pos">x ${fmt(c.p[0])} · y ${fmt(c.p[1])} · ${c.state === 'held' ? 'in gripper' : c.p[2] < -0.2 ? 'off the board' : cubeTilt(c) > 20 ? 'toppled' : `level ${lvl}`}${c.state === 'awake' ? ' · moving' : ''}</span></span>
      <button title="Click on the mat to drop it there" class="${placing === c.id ? 'on' : ''}">⌖</button><button title="Remove">×</button>`;
    const [pick, rm] = row.querySelectorAll('button');
    pick.onclick = () => { placing = placing === c.id ? null : c.id; renderCubes(); };
    rm.onclick = () => { if (world.holding?.id === c.id) world.holding = null; world.cubes = world.cubes.filter(k => k !== c); world.dirty = true; renderCubes(); needsRender = true; };
    $('cubes').append(row);
  }
}
$('add-cube').onclick = () => {
  const colour = $('new-colour').value;
  let id = colour, n = 2;
  while (world.cube(id)) id = `${colour}${n++}`;
  world.addCube(id, colour, Number($('nx').value), Number($('ny').value), 0, 0);
  world.cube(id).p[2] = 30;
  renderCubes();
};
$('reset-scene').onclick = () => { resetScene(); renderCubes(); needsRender = true; };

const ray = new THREE.Raycaster();
let downAt = null;
canvas.addEventListener('pointerdown', e => { downAt = [e.clientX, e.clientY]; });
canvas.addEventListener('pointerup', e => {
  if (!placing || !downAt || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 4) return;
  const r = canvas.getBoundingClientRect();
  ray.setFromCamera(new THREE.Vector2((e.clientX - r.left) / r.width * 2 - 1, -(e.clientY - r.top) / r.height * 2 + 1), camera);
  const hit = new THREE.Vector3();
  if (!ray.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 0, 1), 0), hit)) return;
  const c = world.cube(placing);
  if (c && c.state !== 'held') { c.p = [hit.x, hit.y, 12]; c.v = [0, 0, 0]; c.w = [0.4, 0.3, 0]; c.state = 'awake'; }
  placing = null; renderCubes();
});

$('vision-import').onclick = () => {
  try {
    const data = JSON.parse($('vision-json').value);
    const list = data.cubes || data;
    world.cubes = []; world.holding = null;
    for (const v of list) {
      const [x, y] = v.board_cm || camToBoard(v.x_mm, v.y_mm, MAT.sToEdge);
      let id = v.colour, n = 2; while (world.cube(id)) id = `${v.colour}${n++}`;
      world.addCube(id, v.colour, x, y, v.level || 0, v.yaw_deg || 0);
    }
    renderCubes(); log(`imported ${list.length} cubes from the camera`, 'ok');
  } catch (e) { toast(`Import failed: ${e.message}`); }
};

// ---------------- live link ----------------

let link = null, camTimer = null;
$('live-connect').onclick = () => {
  if (link) { link.stopPolling(); link = null; clearInterval(camTimer); ghost.root.visible = false; $('live-connect').textContent = 'Connect'; $('live-status').textContent = 'not connected'; $('live-cam').hidden = true; updateLiveUi(); return; }
  link = new LiveLink($('live-url').value.trim());
  link.on((state, err) => {
    $('live-status').textContent = err ? `error: ${err}` : `${state.mode}, ${state.outputs_off ? 'outputs off' : state.armed ? 'armed' : 'not armed'}${state.busy ? ', moving' : ''}`;
    if (state && !err) {
      const c = LiveLink.counts(state);
      liveCounts = c;
      if ($('live-follow').checked && !ctl.busy) ctl.setCommands(c);
      if (state.joints) state.joints.forEach((j, i) => { ctl.poses.HOME[i] = j.home; ctl.poses.PARK[i] = j.park; });
    }
    updateLiveUi(); needsRender = true;
  });
  link.start(backend.backend === 'mock' ? 300 : 1500);   // the real controller is shared: poll gently
  $('live-connect').textContent = 'Disconnect';
  // Camera snapshots only when asked for: the real camera is shared and runs warm.
  const refreshCam = () => { if ($('live-cam-on').checked) $('live-cam').src = `${link.base}/api/camera/snapshot.jpg?t=${Date.now()}`; };
  $('live-cam').onload = () => { $('live-cam').hidden = !$('live-cam-on').checked; };
  $('live-cam-on').onchange = () => { $('live-cam').hidden = !$('live-cam-on').checked; refreshCam(); };
  refreshCam(); camTimer = setInterval(refreshCam, 10000);
};
let liveCounts = null;
function updateLiveUi() {
  const on = !!(link && link.state);
  $('live-demo').disabled = !on;
  for (const id of ['live-home', 'live-park']) $(id).disabled = !liveControl();
  $('mode-badge').textContent = liveControl() ? 'live control' : on ? 'sim + live view' : 'simulation';
  $('mode-badge').classList.toggle('live', !!liveControl());
  if (current) $('send-live').disabled = !(liveControl() && current.report.ok && current.mode === 'moves');
}
$('live-enable').onchange = updateLiveUi;
$('show-ghost').onchange = () => { updateLiveUi(); needsRender = true; };
$('live-home').onclick = poseCmd('HOME');
$('live-park').onclick = poseCmd('PARK');
$('live-demo').onclick = () => {
  const d = link.state.demo;
  try {
    ctl.demo_load(d.name, d.positions.map(p => ({ name: p.name, counts: LiveLink.counts(p) })));
    ctl.settings.demo_speed = link.state.settings?.demo_speed || 20; ctl.demo_run();
    log(`playing the controller's demo "${d.name}" in the sim`);
  } catch (e) { toast(e.message); }
};
// Who serves this page: server.mjs with the mock (connect automatically), server.mjs as a proxy to the
// real controller (read only unless started with --allow-motion; connect only when asked), or the
// controller itself (/local/info missing: read state only after Connect).
let backend = { backend: 'none', allowMotion: false };
fetch('/local/info', { cache: 'no-store' }).then(r => (r.ok ? r.json() : { backend: 'controller', allowMotion: true })).catch(() => backend).then(info => {
  backend = info;
  // The consent box starts locked in the HTML and opens only when the server allows motion.
  if (info.allowMotion) $('live-enable').disabled = false;
  else {
    $('live-enable').checked = false; $('live-enable').disabled = true;
    $('live-enable').parentElement.title = 'Read only: the page server forwards GET requests only';
    $('live-status').textContent = 'read only';
  }
  if (info.backend === 'mock') $('live-connect').click();
});

// ---------------- calibration panel ----------------

function renderCal() {
  $('calib').innerHTML = '<tr><th></th><th>angle at HOME (°)</th><th>dir</th><th>counts/°</th></tr>' +
    SERVOS.slice(0, 5).map((s, i) => `<tr><td>${s.name}</td><td><input data-i="${i}" data-k="qHome" type="number" step="0.5" value="${s.qHome}"></td>
      <td><select data-i="${i}" data-k="dir"><option value="1"${s.dir > 0 ? ' selected' : ''}>+1</option><option value="-1"${s.dir < 0 ? ' selected' : ''}>−1</option></select></td>
      <td><input data-i="${i}" data-k="cpd" type="number" step="0.01" value="${s.cpd.toFixed(3)}"></td></tr>`).join('');
  $('calib').querySelectorAll('[data-k]').forEach(el => el.onchange = () => { SERVOS[el.dataset.i][el.dataset.k] = Number(el.value); world.dirty = true; needsRender = true; current = null; });
  $('mat-d').value = MAT.sToEdge;
}
$('mat-d').onchange = () => { MAT.sToEdge = Number($('mat-d').value); placeMatAndCamera(); needsRender = true; };
$('calib-save').onclick = () => { try { localStorage.setItem(CAL_KEY, JSON.stringify(currentCal())); toast('Calibration saved in this browser'); } catch { toast('Browser storage unavailable'); } };
$('calib-reset').onclick = () => { applyCal(structuredClone(DEFAULT_CAL)); renderCal(); placeMatAndCamera(); needsRender = true; };
$('calib-export').onclick = async () => { const j = JSON.stringify(currentCal(), null, 1); try { await navigator.clipboard.writeText(j); toast('Calibration copied'); } catch { console.log(j); } };
renderCal();

// ---------------- tabs and camera tab ----------------

document.querySelectorAll('[data-tab]').forEach(b => b.onclick = () => {
  document.querySelectorAll('[data-tab]').forEach(x => x.classList.toggle('on', x === b));
  $('tab-control').hidden = b.dataset.tab !== 'control';
  $('tab-camera').hidden = b.dataset.tab !== 'camera';
  if (b.dataset.tab === 'camera') camTab.update();
});
const camTab = initCameraTab({
  scene, initial: CAMERA.calib, hideForCamera, log, toast,
  linkBase: () => (link ? link.base : ''),
  onApply: cal => {
    CAMERA.calib = cal;
    try { localStorage.setItem(CAM_KEY, JSON.stringify({ R: cal.R, t: cal.t, f: cal.f, cx: cal.cx, cy: cal.cy, centre: cal.centre, rms: cal.rms, H: cal.H })); } catch { /* not saved */ }
    placeMatAndCamera(); if (oakView) applyOakView(); needsRender = true;
  },
  useDetections: list => {
    world.cubes = []; world.holding = null;
    for (const d of list) {
      const [x, y] = tableToBoard([d.X, d.Y, 0]);
      let id = d.colour, n = 2; while (world.cube(id)) id = `${d.colour}${n++}`;
      world.addCube(id, d.colour, x, y, 0, d.yaw);
    }
    renderCubes(); needsRender = true;
    log(`${list.length} cubes from the camera put on the mat`, 'ok');
    toast(`${list.length} cubes placed. Try a task like "${list[0]?.colour || 'orange'} to 0 14".`);
  },
  // Live tracking: move the nearest sim cube of the same colour to each detection (within 6 cm,
  // and only if it moved more than 3 mm, so camera noise does not keep the physics awake);
  // add cubes the sim does not have yet. Cubes in the gripper are left alone.
  trackDetections: list => {
    let changed = false;
    for (const d of list) {
      const [x, y] = tableToBoard([d.X, d.Y, 0]);
      const same = world.cubes.filter(c => c.colour === d.colour && c.state !== 'held');
      const near = same.sort((a, b) => Math.hypot(a.p[0] - x, a.p[1] - y) - Math.hypot(b.p[0] - x, b.p[1] - y))[0];
      if (near && Math.hypot(near.p[0] - x, near.p[1] - y) < 6) {
        if (Math.hypot(near.p[0] - x, near.p[1] - y) < 0.3) continue;
        near.p = [x, y, CUBE / 2]; near.v = [0, 0, 0]; near.w = [0, 0, 0];
        near.R = [Math.cos(d.yaw * Math.PI / 180), -Math.sin(d.yaw * Math.PI / 180), 0, Math.sin(d.yaw * Math.PI / 180), Math.cos(d.yaw * Math.PI / 180), 0, 0, 0, 1];
        near.state = 'awake'; changed = true;
      } else {
        let id = d.colour, n = 2; while (world.cube(id)) id = `${d.colour}${n++}`;
        world.addCube(id, d.colour, x, y, 0, d.yaw); changed = true;
      }
    }
    if (changed) { renderCubes(); needsRender = true; }
  },
});

// ---------------- main loop ----------------

let needsRender = true, last = performance.now(), lastContacts = new Set(), eventIdx = 0, lastCubeKey = '';
controls.addEventListener('change', () => { needsRender = true; });
$('show-shapes').onchange = () => { needsRender = true; };
$('show-path').onchange = () => { pathLine.visible = pathDots.visible = $('show-path').checked; needsRender = true; };

world.step(0, ctl.position);
renderCubes();

function frame(now) {
  const real = Math.min(0.05, (now - last) / 1000); last = now;
  const dt = real * Number($('time-scale').value);
  const moving = ctl.busy || !world.settled() || !!world.holding;
  if (moving || world.dirty) {
    const n = Math.min(40, Math.max(1, Math.ceil(dt / (1 / 120))));
    for (let i = 0; i < n; i++) {
      ctl.update(dt / n);
      world.step(dt / n, ctl.position);
      // Stop on a new contact, like the real arm should be stopped.
      const keys = new Set(world.contacts.map(k => `${k.a} × ${k.b}`));
      // Cubes are physical now: the arm may push them (logged). Stop only for the arm hitting itself or the board.
      const fresh = [...keys].filter(k => !lastContacts.has(k) && !k.endsWith(' cube'));
      lastContacts = keys;
      if (fresh.length && ctl.busy && $('stop-on-contact').checked) { ctl.stop(`collision: ${fresh.join(', ')}`); toast(`Collision: ${fresh.join(', ')} — stopped`); break; }
    }
    world.dirty = false;
    needsRender = true;
  }
  // world events -> log
  for (; eventIdx < world.events.length; eventIdx++) { const e = world.events[eventIdx]; log(e.text, e.kind === 'warn' ? 'warn' : e.kind === 'grasp' ? 'ok' : ''); }
  if (eventIdx > world.events.length) eventIdx = world.events.length;

  if (!oakView && controls.update()) needsRender = true;
  if (needsRender) {
    const a = anglesFromCounts(ctl.position);
    arm.setPose(a, world.gap);
    // The ghost shows the real arm only where it differs from the simulated one.
    const showGhost = !!(link && link.state && liveCounts && $('show-ghost').checked && liveCounts.some((c, i) => Math.abs(c - ctl.position[i]) > 2));
    ghost.root.visible = showGhost;
    if (showGhost) ghost.setPose(anglesFromCounts(liveCounts), world.gap);
    syncCubeMeshes();
    const hitNames = new Set(world.contacts.flatMap(k => [k.a, k.b]));
    updateShapes(armShapes(world.frames, world.gap), hitNames, $('show-shapes').checked);
    $('contacts').hidden = !world.contacts.length;
    $('contacts').textContent = world.contacts.map(k => `${k.a} × ${k.b} ${fmt(-k.sep * 10, 1)} mm`).join(' · ');
    updateJointPanel(ctl.position);
    const f = world.frames, t = toolYawAndTilt(f.hand);
    $('pose-readout').textContent = `Cube-centre point x ${fmt(f.grasp[0])} · y ${fmt(f.grasp[1])} · z ${fmt(f.grasp[2])} cm · tool ${fmt(t.tilt, 0)}° off vertical`;
    $('hud').textContent = ctl.outputsOff ? `stopped: ${ctl.error}` : ctl.motion ? ctl.motion.label : 'idle';
    if (current) {
      const lbl = ctl.motion?.label;
      $('plan-out').querySelectorAll('tr[data-i]').forEach(tr => tr.classList.toggle('cur', !!lbl && current.plan.steps[tr.dataset.i]?.label === lbl.split(':')[0]));
    }
    // The backdrop wall hides itself when the camera swings behind it.
    if (backdrop) backdrop.visible = camera.position.y > backdrop.position.y + 2;
    if (oakView) hideForCamera(view.render); else view.render();
    camTab.markDirty(); camTab.render();
    needsRender = false;
    const cubeKey = world.cubes.map(c => c.id + c.state + c.p.map(v => v.toFixed(1))).join('|');
    if (cubeKey !== lastCubeKey && !placing) { lastCubeKey = cubeKey; renderCubes(); }
  }
  requestAnimationFrame(frame);
}
resize();
requestAnimationFrame(frame);

// for the console and for tests
window.sim = { ctl, world, scene, camera, controls, renderer, arm, camTab, get link() { return link; }, makePlan, THREE };

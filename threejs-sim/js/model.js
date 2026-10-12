// three.js model of the arm, board, mat, cubes and the desk around them.
// Units: cm, Z up, board frame of docs/arm-dimensions.md.
//
// Joint distances come from the schematic (docs/images/arm-dimensions.png, canon).
// Shapes follow the photos in docs/images/photos and the
// MG996R datasheet (body 4,07 x 1,97 x 3,67 cm, 5,45 cm over the mounting ears).
// Bracket, horn and claw sizes are read from the photos, not measured.
//
// Each moving link is built from many small meshes and then merged per material
// (bake), so the whole arm draws in a few dozen calls.

import * as THREE from 'three';
import { RoundedBoxGeometry } from '../vendor/RoundedBoxGeometry.js';
import { mergeGeometries } from '../vendor/jsm/utils/BufferGeometryUtils.js';
import { ARM, BOARD, CUBE, GRIPPER, MAT, COLOURS } from './config.js';
import { thetaForGap } from './kinematics.js';

const RAD = Math.PI / 180;
/** Centre of the 9 x 8,4 base plate relative to the S1 axis (cm, +Y forward). */
export const BASE_OFFSET_Y = 2.2;
const V = (x, y, z) => new THREE.Vector3(x, y, z);

// ---------------- procedural textures ----------------

function canvasTexture(w, h, draw, { srgb = true, repeat = null } = {}) {
  const c = document.createElement('canvas');
  c.width = w; c.height = h;
  draw(c.getContext('2d'), w, h);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = srgb ? THREE.SRGBColorSpace : THREE.NoColorSpace;
  t.anisotropy = 8;
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  if (repeat) t.repeat.set(...repeat);
  return t;
}

let seed = 1234567;
const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647);

/** Grey noise; `streak` adds row-wise variation (brushed metal), `blur` softens it. */
function noise(g, w, h, base, amp, { streak = 0, fibres = 0 } = {}) {
  const img = g.createImageData(w, h);
  for (let y = 0; y < h; y++) {
    const row = streak ? (rnd() - 0.5) * streak : 0;
    for (let x = 0; x < w; x++) {
      const v = Math.max(0, Math.min(255, base + (rnd() - 0.5) * amp + row));
      const i = 4 * (y * w + x);
      img.data[i] = img.data[i + 1] = img.data[i + 2] = v; img.data[i + 3] = 255;
    }
  }
  g.putImageData(img, 0, 0);
  for (let i = 0; i < fibres; i++) {
    g.strokeStyle = `rgba(${rnd() < 0.5 ? '0,0,0' : '255,255,255'},${0.05 + rnd() * 0.08})`;
    g.lineWidth = 1;
    const x = rnd() * w, y = rnd() * h, a = rnd() * Math.PI, l = 3 + rnd() * 10;
    g.beginPath(); g.moveTo(x, y); g.lineTo(x + Math.cos(a) * l, y + Math.sin(a) * l); g.stroke();
  }
}

// DICT_4X4_50 bit patterns for the mat markers 0..5 (from OpenCV), rows top to bottom, 1 = white.
const ARUCO = {
  0: ['1011', '0101', '0011', '0010'], 1: ['0000', '1111', '1001', '1010'], 2: ['0011', '0011', '0010', '1101'],
  3: ['1001', '1001', '0100', '0110'], 4: ['0101', '0100', '1001', '1110'], 5: ['0111', '1001', '1100', '1101'],
};
// Marker squares (camera table frame, mm): id -> [Xmin, Ymin], 30 mm each (calibration.json).
// The marker's top row faces +X (far edge), its right side faces -Y.
const MARKERS = { 0: [10, 95], 1: [10, -125], 2: [145, 95], 3: [145, -125], 4: [320, 95], 5: [320, -125] };

/** The printed reference mat: field 280 x 360 mm on 290 x 375 mm paper, 4 px per mm. */
function matTexture() {
  const PX = 4, PW = 290, PH = 375, ox = 5, oy = 5;
  const t = canvasTexture(PW * PX, PH * PX, (g, w, h) => {
    g.fillStyle = '#f6f6f2'; g.fillRect(0, 0, w, h);
    // Canvas top = far edge (+X), canvas right = -Y (the arm's right), as seen from the arm.
    const cx = Y => (ox + 140 - Y) * PX;
    const cy = X => h - (oy + X) * PX;
    for (let X = 0; X <= 360; X += 5) {
      g.strokeStyle = X % 50 === 0 ? '#878d94' : X % 10 === 0 ? '#c4c9ce' : '#e2e5e8';
      g.lineWidth = X % 50 === 0 ? 2.4 : 1;
      g.beginPath(); g.moveTo(cx(140), cy(X)); g.lineTo(cx(-140), cy(X)); g.stroke();
    }
    for (let Y = -140; Y <= 140; Y += 5) {
      g.strokeStyle = Y % 50 === 0 ? '#878d94' : Y % 10 === 0 ? '#c4c9ce' : '#e2e5e8';
      g.lineWidth = Y % 50 === 0 ? 2.4 : 1;
      g.beginPath(); g.moveTo(cx(Y), cy(0)); g.lineTo(cx(Y), cy(360)); g.stroke();
    }
    g.strokeStyle = '#4d535a'; g.lineWidth = 3; g.strokeRect(cx(140), cy(360), 280 * PX, 360 * PX);
    // the mat is two A4 sheets, joined at X = 180 with a slight overlap shadow
    g.fillStyle = 'rgba(0,0,0,0.07)'; g.fillRect(0, cy(180) - 2, w, 5);
    g.fillStyle = 'rgba(255,255,255,0.8)'; g.fillRect(0, cy(180) - 4, w, 2);
    g.fillStyle = '#25292d'; g.textAlign = 'center'; g.textBaseline = 'middle';
    const label = (text, X, Y, size = 7, bold = true) => { g.font = `${bold ? 'bold ' : ''}${size * PX}px Helvetica, Arial, sans-serif`; g.fillText(text, cx(Y), cy(X)); };
    for (let X = 50; X <= 350; X += 50) { label(String(X), X, 75); label(String(X), X, -75); }
    for (const Y of [-100, -50, 50, 100]) label((Y > 0 ? '+' : '') + Y, 280, Y);
    label('0', 280, 0); label('Y (mm)', 270, 0, 4.5); label('X (mm)', 340, 75, 4.5); label('X (mm)', 155, 75, 4.5);
    label('X 360 - FAR EDGE', 352, 0, 5); label('+Z UP', 330, 0, 4);
    label('TABLE REFERENCE  |  2/2  |  FAR ARM', 365, 60, 2.6, false);
    label('A4 landscape  |  Actual size / 100%  |  Single-sided', 365, -70, 2.2, false);
    for (const [id, [X0, Y0]] of Object.entries(MARKERS)) {
      const bits = ARUCO[id], cell = 30 / 6;
      g.fillStyle = '#ffffff'; g.fillRect(cx(Y0 + 33), cy(X0 + 33), 36 * PX, 36 * PX);
      for (let r = 0; r < 6; r++) for (let c = 0; c < 6; c++) {
        const white = r > 0 && r < 5 && c > 0 && c < 5 && bits[r - 1][c - 1] === '1';
        g.fillStyle = white ? '#ffffff' : '#151515';
        const X = X0 + 30 - (r + 1) * cell, Y = Y0 + 30 - c * cell;
        g.fillRect(cx(Y), cy(X + cell), cell * PX + 0.6, cell * PX + 0.6);
      }
      label(`ID ${id}  X ${X0 + 15}  Y ${Y0 + 15}`, X0 - 3.5, Y0 + 15, 2.0, false);
    }
  });
  t.wrapS = t.wrapT = THREE.ClampToEdgeWrapping;
  return t;
}

function woodTexture(hex) {
  const col = new THREE.Color(hex);
  return canvasTexture(256, 256, (g, w, h) => {
    g.fillStyle = `#${col.getHexString()}`; g.fillRect(0, 0, w, h);
    for (let i = 0; i < 70; i++) {
      const y = rnd() * h;
      g.strokeStyle = `rgba(0,0,0,${0.025 + rnd() * 0.05})`;
      g.lineWidth = 1 + rnd() * 3;
      g.beginPath(); g.moveTo(0, y);
      for (let x = 0; x <= w; x += 16) g.lineTo(x, y + Math.sin(x / 40 + i) * 4);
      g.stroke();
    }
    const d = g.getImageData(0, 0, w, h);
    for (let i = 0; i < d.data.length; i += 4) { const n = (rnd() - 0.5) * 16; d.data[i] += n; d.data[i + 1] += n; d.data[i + 2] += n; }
    g.putImageData(d, 0, 0);
  });
}

function servoLabelTexture() {
  return canvasTexture(512, 256, (g, w, h) => {
    g.fillStyle = '#121214'; g.fillRect(0, 0, w, h);
    g.fillStyle = '#f2cf1d'; g.fillRect(70, 70, 372, 120);
    g.fillStyle = '#121214'; g.fillRect(78, 78, 120, 44);
    g.fillStyle = '#f2cf1d'; g.font = 'bold 34px Arial'; g.textAlign = 'center'; g.fillText('DM', 138, 112);
    g.fillStyle = '#121214'; g.font = 'bold 50px Arial'; g.fillText('MG996R', 310, 125);
    g.font = 'bold 22px Arial'; g.fillText('DIGI HI-SPEED', 256, 172);
    g.fillStyle = '#f2cf1d'; g.font = '16px Arial'; g.fillText('TOWER-PRO  ·  13KG  ·  4.8-7.2V', 256, 220);
  });
}

// ---------------- materials ----------------

export function makeMaterials() {
  const grain = canvasTexture(256, 256, (g, w, h) => noise(g, w, h, 128, 70), { srgb: false, repeat: [3, 3] });
  const grainRough = canvasTexture(256, 256, (g, w, h) => noise(g, w, h, 150, 50), { srgb: false, repeat: [2, 2] });
  const brushed = canvasTexture(512, 64, (g, w, h) => noise(g, w, h, 205, 30, { streak: 40 }), { repeat: [1, 6] });
  const plasticGrain = canvasTexture(256, 256, (g, w, h) => noise(g, w, h, 128, 110), { srgb: false, repeat: [4, 4] });
  const boardTex = canvasTexture(512, 512, (g, w, h) => noise(g, w, h, 120, 18), { repeat: [2, 2] });
  const felt = canvasTexture(512, 512, (g, w, h) => noise(g, w, h, 205, 45, { fibres: 3000 }), { repeat: [10, 4] });
  const deskTex = canvasTexture(256, 256, (g, w, h) => noise(g, w, h, 240, 8), { repeat: [16, 8] });
  const P = (o) => new THREE.MeshPhysicalMaterial(o);
  const S = (o) => new THREE.MeshStandardMaterial(o);
  return {
    // Black anodised 2 mm aluminium brackets: satin, slightly metallic, fine grain.
    anodised: P({ color: 0x1a1c20, metalness: 0.6, roughness: 0.4, roughnessMap: grainRough, bumpMap: grain, bumpScale: 0.25, clearcoat: 0.4, clearcoatRoughness: 0.3, sheen: 1.0, sheenColor: 0x7d8da3, sheenRoughness: 0.35 }),
    // Black acrylic / anodised claw plates: a bit glossier.
    acrylic: P({ color: 0x121316, metalness: 0.35, roughness: 0.3, clearcoat: 0.6, clearcoatRoughness: 0.2, sheen: 1.0, sheenColor: 0x7d8da3, sheenRoughness: 0.35, side: THREE.DoubleSide }),
    servo: P({ color: 0x151518, roughness: 0.6, metalness: 0.0, bumpMap: plasticGrain, bumpScale: 0.35, sheen: 0.8, sheenColor: 0x5c7698, sheenRoughness: 0.5 }),
    servoCap: P({ color: 0x1b1b1f, roughness: 0.38, metalness: 0.0, clearcoat: 0.3, sheen: 0.8, sheenColor: 0x5c7698, sheenRoughness: 0.45 }),
    servoLabel: S({ map: servoLabelTexture(), roughness: 0.45 }),
    spline: S({ color: 0xd9c38a, metalness: 1, roughness: 0.3 }),
    horn: P({ color: 0xd2d5da, metalness: 1, roughness: 0.28, anisotropy: 0.5 }),
    hornBlack: P({ color: 0x1c1d21, metalness: 0.7, roughness: 0.35, clearcoat: 0.3 }),
    screw: S({ color: 0xe1e4e8, metalness: 1, roughness: 0.16 }),
    slot: S({ color: 0x24262a, metalness: 0.6, roughness: 0.6 }),
    tube: P({ color: 0xdfe2e6, metalness: 1, roughness: 0.22, map: brushed, anisotropy: 0.8 }),
    bearing: P({ color: 0xc4c8ce, metalness: 1, roughness: 0.2 }),
    pad: P({ color: 0xf3f5f6, roughness: 0.85, transparent: true, opacity: 0.85 }),
    cableBrown: P({ color: 0x5b3214, roughness: 0.45, clearcoat: 0.4 }),
    cableRed: P({ color: 0xc21d16, roughness: 0.45, clearcoat: 0.4 }),
    cableOrange: P({ color: 0xf2900f, roughness: 0.45, clearcoat: 0.4 }),
    pinkFoam: S({ color: 0xe8579f, roughness: 1 }),
    wireYellow: P({ color: 0xe7c51d, roughness: 0.45, clearcoat: 0.4 }),
    wireGreen: P({ color: 0x2f9b47, roughness: 0.45, clearcoat: 0.4 }),
    wireBlue: P({ color: 0x2d62c8, roughness: 0.45, clearcoat: 0.4 }),
    wirePurple: P({ color: 0x7b43b8, roughness: 0.45, clearcoat: 0.4 }),
    wireBlack: P({ color: 0x1a1a1a, roughness: 0.45, clearcoat: 0.4 }),
    pcbBlue: P({ color: 0x1d4f9a, roughness: 0.4, clearcoat: 0.5 }),
    pcbGreen: P({ color: 0x1f7a3a, roughness: 0.4, clearcoat: 0.5 }),
    chip: S({ color: 0x151515, roughness: 0.5 }),
    gold: S({ color: 0xe0b44c, metalness: 1, roughness: 0.25 }),
    terminal: S({ color: 0x2fa84f, roughness: 0.5 }),
    zip: P({ color: 0xf1f0ea, roughness: 0.5, transparent: true, opacity: 0.92 }),
    board: P({ color: 0x7d8186, map: boardTex, roughness: 0.55, metalness: 0.3, clearcoat: 0.15 }),
    mat: S({ map: matTexture(), roughness: 0.8 }),
    tape: P({ color: 0xffffff, roughness: 0.15, transparent: true, opacity: 0.25, clearcoat: 1 }),
    desk: S({ color: 0xf2f2ef, map: deskTex, roughness: 0.55 }),
    wall: S({ color: 0xb4b8be, map: felt, roughness: 1 }),   // light grey felt partition
    piCase: P({ color: 0xd61f4a, roughness: 0.35, clearcoat: 0.5 }),
    piWhite: S({ color: 0xf0f0f0, roughness: 0.45 }),
    metalBox: S({ color: 0x8c8f93, metalness: 0.7, roughness: 0.45 }),
    psuBody: S({ color: 0x6c7076, metalness: 0.4, roughness: 0.55 }),
    psuFront: S({ color: 0xe8e3d4, roughness: 0.6 }),
    led: S({ color: 0xff2a1a, emissive: 0xff2a1a, emissiveIntensity: 1.6 }),
    camBody: P({ color: 0x17191c, roughness: 0.4, metalness: 0.5, clearcoat: 0.4 }),
    glass: P({ color: 0x05070a, roughness: 0.05, metalness: 0.2, clearcoat: 1 }),
    sensorSilver: S({ color: 0xcfd2d6, metalness: 1, roughness: 0.3 }),
  };
}

// ---------------- geometry helpers ----------------

const mesh = (geo, mat, cast = true) => { const m = new THREE.Mesh(geo, mat); m.castShadow = cast; m.receiveShadow = true; return m; };
const at = (o, x, y, z) => { o.position.set(x, y, z); return o; };
/** Rotate o so its local X, Y, Z axes point along the given parent axes (must be right-handed). */
function orient(o, x, y, z) {
  if (x.clone().cross(y).dot(z) < 0.99) throw new Error('orient: axes are not a right-handed basis');
  o.setRotationFromMatrix(new THREE.Matrix4().makeBasis(x, y, z)); return o;
}
/** Rotate o so its local Z axis points along d. */
function aimZ(o, d) { o.quaternion.setFromUnitVectors(V(0, 0, 1), d.clone().normalize()); return o; }

const box = (sx, sy, sz, mat, r = 0) => mesh(r > 0 ? new RoundedBoxGeometry(sx, sy, sz, 2, r) : new THREE.BoxGeometry(sx, sy, sz), mat);
function cyl(r, h, mat, ax = 'z', seg = 28, r2 = r) {
  const m = mesh(new THREE.CylinderGeometry(r2, r, h, seg), mat);
  if (ax === 'z') m.rotation.x = Math.PI / 2;
  if (ax === 'x') m.rotation.z = -Math.PI / 2;
  return m;
}

function roundedRect(w, h, r) {
  const s = new THREE.Shape(), x0 = -w / 2, y0 = -h / 2;
  r = Math.min(r, w / 2 - 1e-3, h / 2 - 1e-3);
  s.moveTo(x0 + r, y0); s.lineTo(x0 + w - r, y0); s.absarc(x0 + w - r, y0 + r, r, -Math.PI / 2, 0, false);
  s.lineTo(x0 + w, y0 + h - r); s.absarc(x0 + w - r, y0 + h - r, r, 0, Math.PI / 2, false);
  s.lineTo(x0 + r, y0 + h); s.absarc(x0 + r, y0 + h - r, r, Math.PI / 2, Math.PI, false);
  s.lineTo(x0, y0 + r); s.absarc(x0 + r, y0 + r, r, Math.PI, 1.5 * Math.PI, false);
  return s;
}

/** Smooth outline around a set of circles [[x, y, r]] (convex hull of points on them). */
function hullShape(circles, n = 24) {
  const pts = [];
  for (const [x, y, r] of circles) for (let i = 0; i < n; i++) { const a = i / n * 2 * Math.PI; pts.push([x + r * Math.cos(a), y + r * Math.sin(a)]); }
  pts.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lower = [], upper = [];
  for (const p of pts) { while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) lower.pop(); lower.push(p); }
  for (const p of pts.slice().reverse()) { while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) upper.pop(); upper.push(p); }
  const hull = lower.slice(0, -1).concat(upper.slice(0, -1));
  const s = new THREE.Shape(); s.moveTo(...hull[0]); for (const p of hull.slice(1)) s.lineTo(...p); s.closePath();
  return s;
}

function addHoles(shape, holes) {
  for (const [hx, hy, hr] of holes) { const p = new THREE.Path(); p.absarc(hx, hy, hr, 0, Math.PI * 2, true); shape.holes.push(p); }
  return shape;
}

/** Extruded plate (outline in XY, thickness along Z, centred on z = 0) with a small edge bevel. */
function plateFromShape(shape, t, mat, bevel = 0.035) {
  const g = new THREE.ExtrudeGeometry(shape, { depth: t - 2 * bevel, bevelEnabled: bevel > 0, bevelThickness: bevel, bevelSize: bevel, bevelSegments: 2, curveSegments: 20 });
  g.translate(0, 0, -(t - 2 * bevel) / 2);
  return mesh(g, mat);
}
const plate = (w, h, t, mat, holes = [], r = 0.3) => plateFromShape(addHoles(roundedRect(w, h, r), holes), t, mat);

/** Horn pattern of every kit bracket (Probots drawings): 8 mm bore, 4 x 3 mm holes on a 14 mm circle. */
const hornHoles = (x, y, R = 0.4) => [[x, y, R], ...[0, 1, 2, 3].map(k => [x + 0.7 * Math.cos(k * Math.PI / 2), y + 0.7 * Math.sin(k * Math.PI / 2), 0.15])];

/** Phillips pan-head screw, head facing +Z, at the origin. */
function screw(M, r = 0.17) {
  const g = new THREE.Group();
  const head = mesh(new THREE.CylinderGeometry(r * 0.92, r, 0.07, 16), M.screw); head.rotation.x = Math.PI / 2; at(head, 0, 0, 0.035); g.add(head);
  const dome = mesh(new THREE.SphereGeometry(r * 0.92, 16, 6, 0, Math.PI * 2, 0, Math.PI / 2), M.screw);
  dome.rotation.x = Math.PI / 2; dome.scale.set(1, 0.45, 1); at(dome, 0, 0, 0.07); g.add(dome);
  for (const rz of [0, Math.PI / 2]) { const s = box(r * 1.2, 0.035, 0.05, M.slot); s.rotation.z = rz; at(s, 0, 0, 0.13); g.add(s); }
  return g;
}
/** Screw on a parent at position p with the head facing direction d. */
function screwAt(parent, M, p, d, r) { const s = screw(M, r); aimZ(s, d); s.position.copy(p); parent.add(s); return s; }
function nut(M, s = 0.28) { const n = mesh(new THREE.CylinderGeometry(s, s, 0.22, 6), M.screw); n.rotation.x = Math.PI / 2; return n; }

// ---------------- MG996R servo ----------------

/**
 * MG996R. Local frame: output shaft on +Z at the origin, body along +X (shaft 1,0 from the
 * near end), width along Y. Case top at z = -0.05, bottom at z = -3.72.
 */
function servo(M, { label = true } = {}) {
  const g = new THREE.Group();
  const L = 4.07, W = 1.97, xc = L / 2 - 1.0;
  const top = box(L, W, 0.75, M.servoCap, 0.08); at(top, xc, 0, -0.43); g.add(top);
  const mid = box(L - 0.04, W - 0.04, 2.1, M.servo, 0.04); at(mid, xc, 0, -1.87); g.add(mid);
  const bot = box(L, W, 0.8, M.servoCap, 0.08); at(bot, xc, 0, -3.32); g.add(bot);
  // mounting ears with two holes each (drawn as dark discs) and screws holding them
  for (const sx of [-1, 1]) {
    const ear = plate(0.75, W, 0.25, M.servo, [[0.05, 0.5, 0.2], [0.05, -0.5, 0.2]], 0.12);
    at(ear, xc + sx * (L / 2 + 0.37), 0, -0.92); ear.scale.x = sx; g.add(ear);
    for (const sy of [-0.5, 0.5]) screwAt(g, M, V(xc + sx * (L / 2 + 0.42), sy, -0.8), V(0, 0, 1), 0.15);
  }
  // shaft boss, gear dome and splined output
  const boss = cyl(0.72, 0.22, M.servoCap, 'z', 32); at(boss, 0, 0, 0.06); g.add(boss);
  const dome = cyl(0.5, 0.12, M.servoCap, 'z', 24); at(dome, 1.0, 0, 0.0); g.add(dome);
  const spline = cyl(0.28, 0.42, M.spline, 'z', 18); at(spline, 0, 0, 0.32); g.add(spline);
  // four case screws on the bottom
  for (const [x, y] of [[-0.75, -0.7], [-0.75, 0.7], [2.8, -0.7], [2.8, 0.7]]) screwAt(g, M, V(x, y, -3.73), V(0, 0, -1), 0.11);
  // cable exit on the far end
  const exit = box(0.5, 0.9, 0.5, M.servo, 0.08); at(exit, L - 1.0 + 0.15, 0, -2.9); g.add(exit);
  const mats = [M.cableBrown, M.cableRed, M.cableOrange];
  [-0.12, 0, 0.12].forEach((y, i) => { const w = cyl(0.06, 0.9, mats[i], 'x', 8); at(w, L - 1.0 + 0.7, y, -2.9); g.add(w); });
  if (label) {
    const lab = mesh(new THREE.PlaneGeometry(L - 0.4, 2.0), M.servoLabel, false);
    orient(lab, V(1, 0, 0), V(0, 0, 1), V(0, -1, 0)); at(lab, xc, -W / 2 - 0.004, -1.9); g.add(lab);
  }
  return g;
}

/** Round aluminium servo horn on +Z with its screw circle and centre screw. */
function horn(M, r = 1.0, mat = M.horn) {
  // Round 25T aluminium horn (Probots drawing): 20 mm disc, 2 mm thick, 9 mm boss, 4,5 mm overall,
  // 4 x M3 on a 14 mm circle, so it matches the brackets' horn pattern.
  const g = new THREE.Group();
  const holes = [0, 1, 2, 3].map(k => [0.7 * Math.cos(k * Math.PI / 2), 0.7 * Math.sin(k * Math.PI / 2), 0.15]);
  const d = plateFromShape(addHoles(roundedRect(2 * r, 2 * r, r - 0.001), holes), 0.2, mat, 0.03); at(d, 0, 0, 0.35); g.add(d);
  const hub = cyl(0.45, 0.25, mat, 'z', 24); at(hub, 0, 0, 0.125); g.add(hub);
  for (let k = 0; k < 4; k++) screwAt(g, M, V(0.7 * Math.cos(k * Math.PI / 2), 0.7 * Math.sin(k * Math.PI / 2), 0.45), V(0, 0, 1), 0.14);
  screwAt(g, M, V(0, 0, 0.45), V(0, 0, 1), 0.2);
  return g;
}

/** Bracket side plate in the YZ plane (normal X): width along Y, from z0 to z1, round ends. */
function sidePlate(M, x, z0, z1, width = 2.5, holes = [], mat = M.anodised) {
  const len = z1 - z0;
  const p = plateFromShape(addHoles(roundedRect(width, len, Math.min(width / 2, 1.25)), holes.map(([u, v, r]) => [u, v - len / 2, r])), 0.2, mat);
  orient(p, V(0, 1, 0), V(0, 0, 1), V(1, 0, 0));
  return at(p, x, 0, (z0 + z1) / 2);
}

/** Bend between two bracket plates: a quarter-round along the given axis. */
function bend(M, len, ax = 'y') { return cyl(0.2, len, M.anodised, ax, 10); }

/** Three-strand servo cable (brown, red, orange) along a smooth curve through pts. */
function cable(M, pts, { tension = 0.5 } = {}) {
  const g = new THREE.Group();
  const curve = new THREE.CatmullRomCurve3(pts.map(p => V(...p)), false, 'catmullrom', tension);
  const n = Math.max(24, pts.length * 14);
  const frames = curve.computeFrenetFrames(n, false);
  const mats = [M.cableBrown, M.cableRed, M.cableOrange];
  [-1, 0, 1].forEach((k, i) => {
    const off = [];
    for (let j = 0; j <= n; j++) { const p = curve.getPointAt(j / n); off.push(p.add(frames.binormals[j].clone().multiplyScalar(k * 0.115))); }
    g.add(mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3(off), n, 0.058, 6, false), mats[i]));
  });
  return g;
}

/**
 * Servo cables routed from each servo to the PCA9685 on the upper arm, and the board's ribbon
 * and power leads down to the desk. Every route is a list of anchors in link frames, so the
 * cables follow the arm; between anchors they sag a little like real slack wire.
 * Rebuilt only when the pose changes.
 */
function buildCables(M, G, zS1) {
  const group = new THREE.Group();
  const L3 = ARM.L3, L4 = ARM.L4, gz = GRIPPER.gear[1];
  const pca = k => ['shoulder', [-0.55, -3.35, ARM.L2 / 2 - 0.4 - 2.75 + k * 0.254]];
  const servoWires = [M.cableBrown, M.cableRed, M.cableOrange];
  const routes = [
    { mats: servoWires, pts: [['base', [-3.77, 0, zS1 - 2.9]], ['base', [-4.6, -1.8, zS1 - 1.5]], ['turret', [-2.9, -3.8, 8.8]], ['shoulder', [-1.8, -2.9, 0.8]], pca(0)], sag: 0.6 },
    { mats: servoWires, pts: [['turret', [-1.05, -3.77, ARM.H_S2]], ['turret', [-1.6, -4.4, ARM.H_S2 + 0.9]], ['shoulder', [-1.4, -3.0, 1.2]], pca(1)], sag: 0.5 },
    { mats: servoWires, pts: [['elbow', [-1.05, 0, -3.77]], ['elbow', [-1.6, -1.6, -3.6]], ['shoulder', [-1.3, -2.9, ARM.L2 - 2.0]], pca(2)], sag: 0.7 },
    { mats: servoWires, pts: [['elbow', [-1.05, -3.77, L3]], ['elbow', [-0.9, -1.4, L3 - 3.2]], ['elbow', [-0.7, -0.85, L3 * 0.45]], ['elbow', [-1.2, -1.6, 0.4]], ['shoulder', [-1.1, -3.0, ARM.L2 - 1.6]], pca(3)], sag: 0.5 },
    { mats: servoWires, pts: [['wrist', [3.77, 0, L4 - 3.4]], ['wrist', [3.3, -1.7, 0.6]], ['elbow', [0.4, -0.85, L3 * 0.45]], ['elbow', [-0.9, -1.7, 0.6]], ['shoulder', [-0.9, -3.1, ARM.L2 - 1.2]], pca(4)], sag: 0.5 },
    { mats: servoWires, pts: [['hand', [4.62, -3.52, gz]], ['hand', [3.6, -3.8, -1.0]], ['wrist', [2.2, -2.1, 1.2]], ['elbow', [0.65, -0.75, L3 * 0.45 + 0.3]], ['elbow', [-0.7, -1.8, 0.8]], ['shoulder', [-0.7, -3.2, ARM.L2 - 0.8]], pca(5)], sag: 0.6 },
    // rainbow ribbon from the board's I2C pins to the Raspberry Pi GPIO header on the desk
    { mats: [M.wireYellow, M.wireGreen, M.wireBlue, M.wirePurple], pts: [['shoulder', [0.9, -2.6, ARM.L2 / 2 + 2.4]], ['shoulder', [1.8, -4.0, ARM.L2 / 2 + 1.0]], ['root', [-1.5, -7.5, 4.5]], ['root', [-6.4, -9.2, 1.9]]], sag: 1.6, r: 0.045 },
    // red and black power leads from the board's terminal to the bench supply
    { mats: [M.cableRed, M.wireBlack], pts: [['shoulder', [0.7, -2.8, ARM.L2 / 2 + 2.25]], ['shoulder', [1.6, -4.2, ARM.L2 / 2 + 3.0]], ['root', [-8, -6, 3]], ['root', [-22, -6, -0.6]], ['root', [-28, -0.5, 2.5]]], sag: 2.0, r: 0.08 },
  ];
  const v = new THREE.Vector3();
  const objs = routes.map(r => r.mats.map(m => { const me = new THREE.Mesh(new THREE.BufferGeometry(), m); me.castShadow = true; group.add(me); return me; }));
  let lastKey = '';
  const update = () => {
    G.root.updateMatrixWorld(true);
    const inv = G.root.matrixWorld.clone().invert();
    const key = ['turret', 'shoulder', 'elbow', 'wrist', 'hand'].map(k => G[k].matrixWorld.elements.map(x => x.toFixed(3)).join()).join('|');
    if (key === lastKey) return;
    lastKey = key;
    routes.forEach((r, ri) => {
      const pts = r.pts.map(([g, p]) => v.set(...p).applyMatrix4(G[g].matrixWorld).applyMatrix4(inv).clone());
      const pathPts = [pts[0]];
      for (let i = 1; i < pts.length; i++) {
        const a = pts[i - 1], b = pts[i], d = a.distanceTo(b);
        pathPts.push(a.clone().lerp(b, 0.5).add(new THREE.Vector3(0, 0, -Math.min(r.sag, 0.12 * d))));
        pathPts.push(b);
      }
      const curve = new THREE.CatmullRomCurve3(pathPts, false, 'centripetal');
      const n = Math.min(160, Math.max(24, Math.round(curve.getLength() * 3)));
      const frames = curve.computeFrenetFrames(n, false);
      const k = r.mats.length, spacing = (r.r || 0.058) * 2;
      objs[ri].forEach((me, si) => {
        const off = [];
        for (let j = 0; j <= n; j++) off.push(curve.getPointAt(j / n).add(frames.binormals[j].clone().multiplyScalar((si - (k - 1) / 2) * spacing)));
        me.geometry.dispose();
        me.geometry = new THREE.TubeGeometry(new THREE.CatmullRomCurve3(off), n, r.r || 0.058, 6, false);
      });
    });
  };
  return { group, update };
}

/** White nylon zip tie around a tube of radius r (axis Z), with its tail. */
function zipTie(M, r) {
  const g = new THREE.Group();
  const ring = mesh(new THREE.TorusGeometry(r + 0.05, 0.05, 6, 28), M.zip); ring.scale.z = 2.2; g.add(ring);
  const head = box(0.3, 0.25, 0.3, M.zip, 0.04); at(head, r + 0.15, 0, 0); g.add(head);
  const tail = box(2.6, 0.05, 0.22, M.zip); at(tail, r + 1.5, 0.12, 0); tail.rotation.z = 0.35; g.add(tail);
  return g;
}

/** Merge all static meshes under `group` (not inside child groups marked as joints) per material. */
function bake(group) {
  group.updateMatrixWorld(true);
  const inv = group.matrixWorld.clone().invert();
  const buckets = new Map();
  const victims = [];
  const visit = o => {
    for (const c of o.children) {
      if (c.userData.joint) continue;
      if (c.isMesh) {
        let geo = c.geometry.index ? c.geometry.toNonIndexed() : c.geometry.clone();
        for (const k of Object.keys(geo.attributes)) if (!['position', 'normal', 'uv'].includes(k)) geo.deleteAttribute(k);
        if (!geo.attributes.uv) geo.setAttribute('uv', new THREE.Float32BufferAttribute(new Float32Array(geo.attributes.position.count * 2), 2));
        geo.applyMatrix4(new THREE.Matrix4().multiplyMatrices(inv, c.matrixWorld));
        const key = `${c.material.uuid}|${c.castShadow}`;
        if (!buckets.has(key)) buckets.set(key, { mat: c.material, cast: c.castShadow, geos: [] });
        buckets.get(key).geos.push(geo);
        victims.push(c);
      }
      visit(c);
    }
  };
  visit(group);
  for (const v of victims) v.parent.remove(v);
  for (const { mat, cast, geos } of buckets.values()) group.add(mesh(mergeGeometries(geos), mat, cast));
}

const joint = g => { g.userData.joint = true; return g; };

// ---------------- the arm ----------------

/**
 * Builds the arm. Returns { root, setPose(angles, gap), groups }.
 * root is in the board frame; angles follow kinematics.js (degrees).
 */
export function buildArm(M) {
  const root = new THREE.Group();
  root.name = 'arm';

  // ---- static base: two "U beam" brackets, 90 x 42 x 25 mm (docs/images/web/bracket-u-beam-base-dimensions.jpg),
  // lying on their sides with the webs back to back in the middle: legs form the 9 x 8,4 top and
  // bottom plates, the meeting webs form the centre groove, the open sides face front and back.
  const H = 2.5;
  const base = new THREE.Group(); root.add(base);
  // The plate is not centred on S1: the S1 servo stands at its back, the plate reaches forward
  // over the mat (photos/base-shoulder-closeup.jpg, photos/setup-top-view-base.jpg).
  const plates = new THREE.Group(); plates.position.y = BASE_OFFSET_Y; base.add(plates);
  const legHoles = sy => [-3.475, 0, 3.475].flatMap(x => hornHoles(x, sy * 0.908))
    .concat([[-1.375, sy * 1.608, 0.15], [1.375, sy * 1.608, 0.15], [-4.0, sy * 1.608, 0.15], [4.0, sy * 1.608, 0.15]]);
  for (const sy of [-1, 1]) {
    const yc = sy * 2.12;
    for (const z of [0.1, H - 0.1]) { const p = plate(9.0, 4.0, 0.2, M.anodised, legHoles(sy), 0.5); at(p, 0, yc, z); plates.add(p); }
    const web = plate(9.0, H - 0.3, 0.2, M.anodised, [-3.475, 0, 3.475].map(x => [x, 0, 0.4]), 0.05);
    orient(web, V(1, 0, 0), V(0, 0, 1), V(0, -1, 0)); at(web, 0, sy * 0.13, H / 2); plates.add(web);
    for (const z of [0.15, H - 0.15]) { const bd = bend(M, 9.0, 'x'); at(bd, 0, sy * 0.2, z); plates.add(bd); }
    for (const x of [-4.0, 4.0]) {
      const so = cyl(0.2, H - 0.4, M.screw, 'z', 6); at(so, x, yc + sy * 1.4, H / 2); plates.add(so);
      for (const z of [-0.02, H + 0.02]) screwAt(plates, M, V(x, yc + sy * 1.4, z), V(0, 0, z > 0 ? 1 : -1), 0.17);
    }
  }

  // S1 servo upright in a multi-function bracket on the base, shaft on the S1 axis.
  const zS1 = H + 3.92;   // S1 case on the bracket plate, shaft on top
  const s1 = servo(M); s1.rotation.z = Math.PI; at(s1, 0, 0, zS1); base.add(s1);
  const mb = plate(6.1, 2.6, 0.2, M.anodised, [[-2.4, 0.7, 0.16], [-2.4, -0.7, 0.16], [2.4, 0.7, 0.16], [2.4, -0.7, 0.16], [0, 0, 0.6]], 0.25);
  at(mb, -1.03, 0, H + 0.1); base.add(mb);
  for (const sx of [-1, 1]) {
    const wall = plate(2.6, zS1 - 1.1 - H, 0.2, M.anodised, [[0, 0.3, 0.3]], 0.15);
    orient(wall, V(0, 1, 0), V(0, 0, 1), V(1, 0, 0)); at(wall, -1.03 + sx * 3.0, 0, (H + zS1 - 1.1) / 2); base.add(wall);
    const lip = plate(0.8, 2.6, 0.2, M.anodised, [[0, 0.5, 0.15], [0, -0.5, 0.15]], 0.1); at(lip, -1.03 + sx * 2.7, 0, zS1 - 1.05); base.add(lip);
  }
  for (const [x, y] of [[-3.43, 0.7], [-3.43, -0.7], [1.37, 0.7], [1.37, -0.7]]) screwAt(base, M, V(x, y, H + 0.2), V(0, 0, 1), 0.16);

  // ---- turret: S1 horn and the U bracket holding S2 (shaft +X at z = 9,6), body toward the back
  const turret = joint(new THREE.Group()); root.add(turret);
  const h1 = horn(M); at(h1, 0, 0, zS1 + 0.35); turret.add(h1);
  const zT = zS1 + 0.75;
  const tp = plate(5.4, 4.6, 0.2, M.anodised, hornHoles(0, 0).concat([[-1.9, 1.7, 0.15], [1.9, 1.7, 0.15], [-1.9, -1.7, 0.15], [1.9, -1.7, 0.15]]), 0.3);
  at(tp, 0, -0.6, zT); turret.add(tp);
  for (const sx of [-1, 1]) {
    turret.add(at(sidePlate(M, sx * 2.5, zT, ARM.H_S2 + 1.3, 4.6, [[0, ARM.H_S2 - zT, 0.5], [-1.4, 0.8, 0.15], [1.4, 0.8, 0.15]]), sx * 2.5, -0.6, (zT + ARM.H_S2 + 1.3) / 2));
    const b = bend(M, 4.6, 'y'); at(b, sx * 2.4, -0.6, zT + 0.1); turret.add(b);
  }
  const s2 = orient(servo(M), V(0, -1, 0), V(0, 0, -1), V(1, 0, 0)); at(s2, 1.85, 0, ARM.H_S2); turret.add(s2);
  const pin = cyl(0.42, 0.5, M.bearing, 'x', 24); at(pin, -2.55, 0, ARM.H_S2); turret.add(pin);

  // ---- upper arm (shoulder frame: Z along the link, pivots around X)
  // Two long U brackets (52 x 25 x 62,5 mm, horn 50 mm up the leg) bolted base to base: horn to
  // horn 2 x 5,0 + 2 x 0,2 = 10,4, which matches the measured 10,5 (photos/elbow-s3-forearm-closeup.jpg).
  const shoulder = joint(new THREE.Group()); shoulder.position.z = ARM.H_S2; turret.add(shoulder);
  const h2 = horn(M); orient(h2, V(0, 1, 0), V(0, 0, 1), V(1, 0, 0)); at(h2, 2.05, 0, 0); shoulder.add(h2);
  const zMid = ARM.L2 / 2;
  const legHolesU = (zh, dir) => [...hornHoles(0, 0), [0, dir * 1.86, 0.15], [0, dir * 3.547, 0.15]].map(([u, v, r]) => [u, v + zh, r]);
  for (const sx of [-1, 1]) {
    // lower U: round end at S2, legs up to the middle; upper U: from the middle to the round end at S3
    shoulder.add(sidePlate(M, sx * 2.6, -1.25, zMid - 0.2, 2.5, legHolesU(1.25, 1)));
    shoulder.add(sidePlate(M, sx * 2.6, zMid + 0.2, ARM.L2 + 1.25, 2.5, legHolesU(ARM.L2 + 1.25 - zMid - 0.2, -1)));
    for (const z of [zMid - 0.3, zMid + 0.3]) { const b = bend(M, 2.5, 'y'); at(b, sx * 2.5, 0, z); shoulder.add(b); }
  }
  for (const z of [zMid - 0.1, zMid + 0.1]) { const p = plate(5.0, 2.5, 0.2, M.anodised, [...hornHoles(0, 0), [-1.86, 0, 0.15], [1.86, 0, 0.15]], 0.05); at(p, 0, 0, z); shoulder.add(p); }
  for (const x of [-1.86, 1.86]) { screwAt(shoulder, M, V(x, 0, zMid + 0.2), V(0, 0, 1), 0.16); const n = nut(M, 0.2); n.rotation.set(0, 0, 0); at(n, x, 0, zMid - 0.3); shoulder.add(n); }
  const pin3 = cyl(0.42, 0.45, M.bearing, 'x', 24); at(pin3, -2.4, 0, ARM.L2); shoulder.add(pin3);
  // PCA9685 servo driver on pink foam on the back of the upper arm (photos/base-s2-shoulder-side.jpg)
  const pca = new THREE.Group();
  pca.add(at(box(2.7, 0.6, 6.2, M.pinkFoam, 0.18), 0, 0, 0));
  pca.add(at(box(2.5, 0.16, 6.2, M.pcbBlue, 0.03), 0, -0.38, 0));
  for (let i = 0; i < 4; i++) pca.add(at(box(0.85, 0.12, 0.25, M.chip), 0.55, -0.5, -1.0 + i * 0.5));          // resistor arrays
  pca.add(at(box(0.9, 0.18, 0.9, M.chip), -0.35, -0.52, 1.3));                                               // PCA9685 chip
  pca.add(at(box(0.9, 0.7, 0.8, M.terminal, 0.05), 0.5, -0.75, 2.65));                                        // power terminal
  for (const x of [0.25, 0.75]) screwAt(pca, M, V(x, -1.1, 2.65), V(0, -1, 0), 0.12);
  pca.add(at(box(0.15, 0.1, 0.1, M.led), 0.95, -0.5, 1.9));
  for (let k = 0; k < 16; k++) {                                                                              // 16 x 3 servo header
    const z = -2.75 + k * 0.254;
    pca.add(at(box(0.75, 0.25, 0.22, M.chip), -0.75, -0.6, z));
    if (k < 6) pca.add(at(box(0.78, 1.45, 0.26, M.chip, 0.03), -0.75, -1.3, z));                              // plugged servo connectors S1..S6
    else for (let r = 0; r < 3; r++) pca.add(at(box(0.064, 0.6, 0.064, M.gold), -1.0 + r * 0.254, -0.95, z));
  }
  at(pca, 0.2, -1.65, zMid - 0.4); shoulder.add(pca);

  // ---- forearm (elbow frame). The S3 servo rides on the forearm: its body sits in a bracket
  // behind the elbow, the upper arm's U bracket is screwed to its horn (photos/elbow-s3-forearm-closeup.jpg).
  const elbow = joint(new THREE.Group()); elbow.position.z = ARM.L2; shoulder.add(elbow);
  const h3 = horn(M); orient(h3, V(0, 1, 0), V(0, 0, 1), V(1, 0, 0)); at(h3, 2.05, 0, 0); elbow.add(h3);
  const s3 = orient(servo(M), V(0, 0, -1), V(0, 1, 0), V(1, 0, 0)); at(s3, 1.85, 0, 0); elbow.add(s3);
  for (const sy of [-1, 1]) {
    const side = plate(4.6, 4.9, 0.2, M.anodised, [[-1.2, 1.5, 0.15], [1.2, 1.5, 0.15], [0, -0.9, 0.4]], 0.15);
    orient(side, V(1, 0, 0), V(0, 0, 1), V(0, -1, 0)); at(side, 0, sy * 1.12, -0.8); elbow.add(side);
  }
  const back = plate(4.6, 2.44, 0.2, M.anodised, [[-1.5, 0, 0.15], [1.5, 0, 0.15]], 0.1); at(back, 0, 0, -3.35); elbow.add(back);
  const cap = plate(4.6, 2.44, 0.2, M.anodised, [...hornHoles(0, 0), [-1.6, 0, 0.15], [1.6, 0, 0.15]], 0.1); at(cap, 0, 0, 1.55); elbow.add(cap);
  for (const x of [-1.6, 1.6]) screwAt(elbow, M, V(x, 0, 1.65), V(0, 0, 1), 0.15);
  const tubeLen = ARM.L3 - 4.45;
  const tube = cyl(0.6, tubeLen, M.tube, 'z', 40); at(tube, 0, 0, 1.85 + tubeLen / 2); elbow.add(tube);
  for (const z of [2.05, ARM.L3 - 2.8]) {
    const fl = cyl(1.0, 0.18, M.bearing, 'z', 32); at(fl, 0, 0, z); elbow.add(fl);
    const col = cyl(0.78, 0.45, M.bearing, 'z', 32); at(col, 0, 0, z + (z < 5 ? 0.3 : -0.3)); elbow.add(col);
    for (let k = 0; k < 3; k++) screwAt(elbow, M, V(0.72 * Math.cos(k * 2.09), 0.72 * Math.sin(k * 2.09), z + (z < 5 ? 0.1 : -0.1)), V(0, 0, z < 5 ? 1 : -1), 0.11);
    const grub = cyl(0.09, 0.3, M.screw, 'x', 8); at(grub, 0.85, 0, z + (z < 5 ? 0.3 : -0.3)); elbow.add(grub);
  }
  const zt = zipTie(M, 0.6); at(zt, 0, 0, ARM.L3 * 0.45); elbow.add(zt);
  const cap2 = plate(5.1, 2.4, 0.2, M.anodised, [[0, 0, 0.62], [-1.6, 0, 0.15], [1.6, 0, 0.15]], 0.2); at(cap2, 0, 0, ARM.L3 - 2.65); elbow.add(cap2);
  for (const sx of [-1, 1]) elbow.add(sidePlate(M, sx * 2.45, ARM.L3 - 2.75, ARM.L3 + 1.2, 2.4, [...hornHoles(0, 2.75)]));
  const s4 = orient(servo(M), V(0, -1, 0), V(0, 0, -1), V(1, 0, 0)); at(s4, 1.85, 0, ARM.L3); elbow.add(s4);
  const pin4 = cyl(0.42, 0.4, M.bearing, 'x', 24); at(pin4, -2.25, 0, ARM.L3); elbow.add(pin4);

  // ---- wrist (wrist frame): U bracket from the S4 horn down to the S5 servo
  const wrist = joint(new THREE.Group()); wrist.position.z = ARM.L3; elbow.add(wrist);
  const h4 = horn(M); orient(h4, V(0, 1, 0), V(0, 0, 1), V(1, 0, 0)); at(h4, 2.05, 0, 0); wrist.add(h4);
  for (const sx of [-1, 1]) wrist.add(sidePlate(M, sx * 2.75, -1.25, 2.8, 2.5, [...hornHoles(0, 1.25)]));
  const wp = plate(5.7, 2.5, 0.2, M.anodised, [[-1.6, 0, 0.18], [1.6, 0, 0.18], [0, 0, 0.5]], 0.2); at(wp, 0, 0, 2.9); wrist.add(wp);
  for (const sx of [-1, 1]) { const b = bend(M, 2.5, 'y'); at(b, sx * 2.65, 0, 2.8); wrist.add(b); }
  const s5 = servo(M); at(s5, 0, 0, ARM.L4 - 0.48); wrist.add(s5);
  for (const sx of [-1, 1]) wrist.add(sidePlate(M, 1.03 + sx * 2.95, 3.0, ARM.L4 - 1.0, 2.2, [[0, 1.5, 0.15]]));

  // ---- hand (flange frame, Z = tool axis)
  const hand = joint(new THREE.Group()); hand.position.z = ARM.L4; wrist.add(hand);
  const gripper = buildGripper(M); hand.add(gripper.group);

  for (const g of [root, turret, shoulder, elbow, wrist, hand, ...gripper.joints]) bake(g);

  const setPose = (a, gap) => {
    turret.rotation.z = -a.s1 * RAD;
    shoulder.rotation.x = -a.s2 * RAD;
    elbow.rotation.x = -a.s3 * RAD;
    wrist.rotation.x = -a.s4 * RAD;
    hand.rotation.z = a.s5 * RAD;
    gripper.setGap(gap);
  };
  const groups = { root, base, turret, shoulder, elbow, wrist, hand };
  const cables = buildCables(M, groups, zS1);
  root.add(cables.group);
  const setPoseAndCables = (a, gap) => { setPose(a, gap); cables.update(); };
  return { root, setPose: setPoseAndCables, groups };
}

/** Claw finger outline in finger-local coordinates (u outward, v down from the gear-link end). */
function fingerShape() {
  const tu = GRIPPER.tip[0], tv = GRIPPER.tip[1];
  const pts = [[-0.35, -0.38], [0.45, -0.42], [1.8, 0.3], [1.9, 0.95], [1.7, 2.6], [0.95, 4.6], [tu + 0.95, tv + 0.72],
               [tu + 0.3, tv + 0.72], [tu + 0.28, tv - 0.62], [tu + 0.42, tv - 1.4], [-0.25, 3.6], [-0.55, 1.6], [-0.52, 0.3]];
  const s = new THREE.Shape();
  s.moveTo(pts[0][0], -pts[0][1]);
  for (const [u, v] of pts.slice(1)) s.lineTo(u, -v);
  s.closePath();
  addHoles(s, [[0, 0, 0.13], [1.35, -0.5, 0.13]]);
  return s;
}

function linkPlate(M, len, w = 0.65, t = 0.2) {
  const r = w / 2, s = new THREE.Shape();
  s.moveTo(-r, 0); s.lineTo(-r, -len); s.absarc(0, -len, r, Math.PI, 2 * Math.PI, false);
  s.lineTo(r, 0); s.absarc(0, 0, r, 0, Math.PI, false);
  return plateFromShape(s, t, M.acrylic, 0.03);
}

function gearShape(r, teeth = 14) {
  const s = new THREE.Shape();
  for (let i = 0; i <= teeth * 4; i++) {
    const a = i / (teeth * 4) * Math.PI * 2;
    const rr = (i % 4 === 1 || i % 4 === 2) ? r + 0.1 : r - 0.1;
    if (i === 0) s.moveTo(rr * Math.cos(a), rr * Math.sin(a)); else s.lineTo(rr * Math.cos(a), rr * Math.sin(a));
  }
  return addHoles(s, [[0, 0, 0.2]]);
}

/**
 * Gripper in the hand frame: u = X (jaw axis), v = +Z (down the tool), depth = Y.
 * Back plate at y = -0.45, gears and front plate in front, links and fingers in two layers.
 */
function buildGripper(M) {
  const group = new THREE.Group();
  const joints = [];
  const { gear, outer, link } = GRIPPER;
  const fl = horn(M, 1.0, M.hornBlack); fl.rotation.x = Math.PI; at(fl, 0, 0, 0.3); group.add(fl);
  // base plates: rounded outline around the gears, the outer pivots and the flange
  const outline = () => addHoles(hullShape([[-gear[0], gear[1], 1.35], [gear[0], gear[1], 1.35], [-outer[0], outer[1], 0.5], [outer[0], outer[1], 0.5], [0, 0.45, 0.9]]),
    [[-gear[0], gear[1], 0.2], [gear[0], gear[1], 0.2], [-outer[0], outer[1], 0.13], [outer[0], outer[1], 0.13]]);
  for (const y of [-0.5, 0.62]) {
    const p = plateFromShape(outline(), 0.2, M.acrylic, 0.03);
    orient(p, V(1, 0, 0), V(0, 0, 1), V(0, -1, 0)); at(p, 0, y, 0); group.add(p);
  }
  for (const sx of [-1, 1]) {
    // standoffs between the plates and screws on both faces
    const so = cyl(0.17, 1.0, M.screw, 'y', 6); at(so, sx * outer[0], 0.06, outer[1]); group.add(so);
    for (const y of [-0.62, 0.74]) screwAt(group, M, V(sx * outer[0], y, outer[1]), V(0, Math.sign(y), 0), 0.14);
    screwAt(group, M, V(sx * gear[0], 0.74, gear[1]), V(0, 1, 0), 0.16);
  }
  // S6 servo behind the back plate, shaft driving the right gear, body out to the side
  const s6 = orient(servo(M), V(1, 0, 0), V(0, 0, -1), V(0, 1, 0)); at(s6, gear[0], -0.62, gear[1]); group.add(s6);
  // gears between the plates
  const gears = [];
  for (const sx of [-1, 1]) {
    const holder = joint(new THREE.Group()); holder.position.set(sx * gear[0], 0.3, gear[1]);
    const gm = plateFromShape(gearShape(0.8), 0.25, M.acrylic, 0.02); holder.add(gm);
    group.add(holder); gears.push({ holder, sx }); joints.push(holder);
  }
  // links and fingers per side
  const sides = [];
  for (const sx of [-1, 1]) {
    const gl = joint(new THREE.Group()), ol = joint(new THREE.Group()), finger = joint(new THREE.Group());
    gl.position.set(sx * gear[0], 0, gear[1]); ol.position.set(sx * outer[0], 0, outer[1]);
    for (const y of [-0.3, 0.35]) {
      const a = linkPlate(M, link); orient(a, V(1, 0, 0), V(0, 0, -1), V(0, 1, 0)); a.position.y = y; gl.add(a);
      const b = linkPlate(M, link); orient(b, V(1, 0, 0), V(0, 0, -1), V(0, 1, 0)); b.position.y = y; ol.add(b);
    }
    for (const lk of [gl, ol]) for (const y of [-0.45, 0.5]) screwAt(lk, M, V(0, y, link), V(0, Math.sign(y), 0), 0.13);
    for (const y of [-0.42, 0.42]) {
      const fm = plateFromShape(fingerShape(), 0.2, M.acrylic, 0.03);
      orient(fm, V(sx, 0, 0), V(0, 0, -1), V(0, sx, 0)); at(fm, 0, y, 0); finger.add(fm);
    }
    // spacers between the two finger plates, and the screws at the joints
    for (const [u, v] of [[0.95, 2.8], [GRIPPER.tip[0] + 0.7, GRIPPER.tip[1] - 0.9]]) {
      const sp = cyl(0.16, 0.65, M.screw, 'y', 6); at(sp, sx * u, 0, v); finger.add(sp);
      for (const y of [-0.55, 0.55]) screwAt(finger, M, V(sx * u, y, v), V(0, Math.sign(y), 0), 0.13);
    }
    const pad = box(GRIPPER.pad.t, GRIPPER.pad.w, GRIPPER.pad.h, M.pad, 0.06);
    at(pad, sx * (GRIPPER.tip[0] + GRIPPER.pad.t / 2), 0, GRIPPER.tip[1]); finger.add(pad);
    group.add(gl, ol, finger);
    sides.push({ sx, gl, ol, finger });
    joints.push(gl, ol, finger);
  }

  const setGap = gap => {
    const t = thetaForGap(Math.max(0, gap)) * RAD;
    for (const { sx, gl, ol, finger } of sides) {
      gl.rotation.y = sx * t; ol.rotation.y = sx * t;
      finger.position.set(sx * (gear[0] + link * Math.sin(t)), 0, gear[1] + link * Math.cos(t));
    }
    for (const { holder, sx } of gears) holder.rotation.set(Math.PI / 2, 0, 0), holder.rotateZ(-sx * t * 1.0);
  };
  setGap(4.5);
  return { group, setGap, joints };
}

// ---------------- the desk around the arm ----------------

export function buildTable(M) {
  const g = new THREE.Group();
  // Grey aluminium-composite board 30 x 38, top at z = 0, S1 axis 1,5 from its back edge.
  const yc = BOARD.d / 2 - BOARD.axisToBackEdge;
  const board = mesh(new RoundedBoxGeometry(BOARD.w, BOARD.d, BOARD.t, 3, 0.12), M.board); at(board, 0, yc, -BOARD.t / 2); g.add(board);
  // Printed mat: canvas top = far edge (+y), canvas right = +x, so the plane needs no rotation.
  const matGroup = new THREE.Group(); g.add(matGroup);
  g.userData.placeMat = d => placeMat(M, matGroup, d);
  placeMat(M, matGroup, MAT.sToEdge);
  const desk = mesh(new THREE.PlaneGeometry(240, 130), M.desk, false); at(desk, 0, 15, -BOARD.t - 0.01); g.add(desk);
  // Felt partition behind the desk; faces the arm (+Y) and is hidden by the app when the camera goes behind it.
  const wall = mesh(new THREE.PlaneGeometry(240, 90), M.wall, false); wall.rotation.set(Math.PI / 2, Math.PI, 0);
  wall.name = 'backdrop'; at(wall, 0, -24, 44); g.add(wall);

  const pi = buildPi(M); pi.rotation.z = Math.PI; at(pi, -4, -11, -BOARD.t); g.add(pi);
  // network and USB cables leaving toward the partition
  for (const [x, mat, r] of [[-0.8, M.piWhite, 0.32], [-4.85, M.wireBlack, 0.28]]) {
    const c = new THREE.CatmullRomCurve3([V(x, -8.0, 0.9), V(x + 0.3, -12, 1.2), V(x + 1.5, -18, 0.0), V(x + 4, -22, -0.7)]);
    g.add(mesh(new THREE.TubeGeometry(c, 40, r, 10, false), mat));
  }
  // bench supply beside the board (photos/setup-desk-overview.jpg)
  const psu = buildPSU(M); psu.rotation.z = 0.25; at(psu, -34, -10, -BOARD.t); g.add(psu);

  // HC-SR04 ultrasonic sensor on the desk behind the arm.
  const us = new THREE.Group();
  us.add(at(box(4.5, 2.0, 0.16, M.pcbBlue), 0, 0, 0.1));
  for (const x of [-1.3, 1.3]) { const c = cyl(0.8, 1.2, M.sensorSilver, 'z', 28); at(c, x, 0, 0.8); us.add(c); }
  us.rotation.z = 0.4; at(us, 15, -12, -BOARD.t + 0.05); g.add(us);
  return g;
}

/**
 * The printed mat (290 x 375 mm paper, field near edge at y = d). Where the paper reaches past
 * the board's back edge it folds down over the edge instead of floating in the air.
 */
function placeMat(M, group, d) {
  group.clear();
  const PW = 29, PH = 37.5, y0 = d - 0.5, edge = -BOARD.axisToBackEdge;
  const strip = (ya, yb, va, vb) => {
    const geo = new THREE.PlaneGeometry(PW, yb - ya);
    const uv = geo.attributes.uv;
    for (let i = 0; i < uv.count; i++) uv.setY(i, uv.getY(i) > 0.5 ? vb : va);
    return mesh(geo, M.mat, false);
  };
  const flat0 = Math.max(y0, edge), vCut = (flat0 - y0) / PH;
  const top = strip(flat0, y0 + PH, vCut, 1); at(top, 0, (flat0 + y0 + PH) / 2, 0.012); top.name = 'mat'; group.add(top);
  if (y0 < edge) {
    const over = edge - y0;
    const flap = strip(0, over, 0, vCut);
    flap.rotation.x = Math.PI / 2; at(flap, 0, edge - 0.012, -over / 2 + 0.01); group.add(flap);   // hanging down the back edge
  }
  for (const x of [-PW / 2 + 0.5, PW / 2 - 0.5]) {
    const t = mesh(new THREE.PlaneGeometry(1.8, y0 + PH - flat0), M.tape, false); at(t, x, (flat0 + y0 + PH) / 2, 0.02); group.add(t);
  }
}

/** Raspberry Pi 4 Model B in the red base of the official case, no lid (photos/raspberry-pi-case.jpg). Origin: case bottom centre. */
function buildPi(M) {
  const g = new THREE.Group();
  const pcbTex = canvasTexture(1024, 680, (c, w, h) => {
    c.fillStyle = '#1d7a3c'; c.fillRect(0, 0, w, h);
    c.strokeStyle = 'rgba(210,190,90,0.35)'; c.lineWidth = 2;               // traces
    for (let i = 0; i < 60; i++) { c.beginPath(); let x = rnd() * w, y = rnd() * h; c.moveTo(x, y); for (let k = 0; k < 4; k++) { x += (rnd() - 0.5) * 160; y += rnd() < 0.5 ? 0 : (rnd() - 0.5) * 160; c.lineTo(x, y); } c.stroke(); }
    c.fillStyle = '#f0f0f0'; c.font = 'bold 30px Arial'; c.fillText('Raspberry Pi 4 Model B', 300, 600); c.font = '22px Arial'; c.fillText('© Raspberry Pi 2018', 330, 635);
    c.font = '18px Arial'; for (const [t, x, y] of [['CAMERA', 600, 300], ['DISPLAY', 120, 520], ['GPIO', 120, 70], ['RUN', 900, 120], ['POE', 800, 90]]) c.fillText(t, x, y);
    for (const [x, y] of [[40, 40], [w - 40, 40], [40, h - 40], [w - 40, h - 40]]) { c.fillStyle = '#d8c060'; c.beginPath(); c.arc(x, y, 22, 0, 7); c.fill(); c.fillStyle = '#111'; c.beginPath(); c.arc(x, y, 12, 0, 7); c.fill(); }
  });
  const pcbMat = new THREE.MeshPhysicalMaterial({ map: pcbTex, roughness: 0.45, clearcoat: 0.5 });
  // case: red tray with rounded walls and the vent slot
  const tray = roundedRect(9.4, 6.6, 0.6); tray.holes.push(roundedRect(8.8, 6.0, 0.4));
  g.add(mesh(new THREE.ExtrudeGeometry(tray, { depth: 2.0, bevelEnabled: true, bevelSize: 0.05, bevelThickness: 0.05, bevelSegments: 2 }), M.piCase));
  g.add(at(box(9.4, 6.6, 0.2, M.piCase, 0.1), 0, 0, 0.1));
  const pcb = mesh(new THREE.BoxGeometry(8.5, 5.6, 0.16), [M.pcbGreen, M.pcbGreen, M.pcbGreen, M.pcbGreen, pcbMat, M.pcbGreen]); at(pcb, 0, 0, 1.0); g.add(pcb);
  // SoC, RAM, USB controller under heat sinks; Wi-Fi can
  for (const [x, y, w] of [[-1.2, 0.4, 1.4], [0.6, 0.6, 1.0], [-1.6, -1.4, 0.8]]) {
    g.add(at(box(w * 0.95, w * 0.95, 0.12, M.chip), x, y, 1.14));
    const hs = new THREE.Group();
    hs.add(at(box(w, w, 0.12, M.metalBox), 0, 0, 0));
    for (let i = 0; i < 5; i++) hs.add(at(box(w, 0.05, 0.38, M.metalBox), 0, -w / 2 + 0.08 + i * (w - 0.16) / 4, 0.24));
    at(hs, x, y, 1.26); g.add(hs);
  }
  g.add(at(box(1.1, 0.9, 0.18, M.sensorSilver, 0.03), 2.2, 1.3, 1.17));
  // ports along the short edge: Ethernet, 2 x USB 3 (blue), 2 x USB 2
  g.add(at(box(1.6, 2.1, 1.35, M.metalBox, 0.05), -3.2, -3.0, 1.75));
  g.add(at(box(1.2, 0.05, 0.9, M.chip), -3.2, -4.06, 1.75));
  for (const [x, blue] of [[-1.15, true], [0.85, false]]) {
    g.add(at(box(1.45, 2.1, 1.6, M.metalBox, 0.05), x, -3.0, 1.88));
    for (const z of [1.5, 2.25]) g.add(at(box(1.2, 0.05, 0.28, blue ? M.wireBlue : M.chip), x, -4.06, z));
  }
  // USB-C power, 2 x micro HDMI, audio on the long edge
  for (const [y, w] of [[2.4, 0.9], [0.9, 0.65], [-0.4, 0.65]]) g.add(at(box(0.7, w, 0.33, M.metalBox, 0.05), 4.05, y, 1.25));
  const jack = cyl(0.3, 0.7, M.chip, 'x', 16); at(jack, 4.05, -1.9, 1.4); g.add(jack);
  // 2 x 20 GPIO header with gold pins, ribbon connector on four of them
  g.add(at(box(5.1, 0.5, 0.25, M.chip), 0.4, 2.45, 1.21));
  for (let i = 0; i < 20; i++) for (const r of [0, 1]) g.add(at(box(0.064, 0.064, 0.6, M.gold), 0.4 - 2.41 + i * 0.254, 2.33 + r * 0.254, 1.5));
  g.add(at(box(1.05, 0.55, 0.85, M.chip, 0.04), -1.2, 2.45, 1.9));
  g.add(at(box(0.25, 0.4, 0.1, M.zip), -1.2, 2.45, 2.35));
  // camera and display FFC connectors, LEDs
  for (const [x, y, rz] of [[0.9, -0.8, 0], [-3.6, 0.6, Math.PI / 2]]) { const c = at(box(2.2, 0.35, 0.3, M.piWhite), x, y, 1.25); c.rotation.z = rz; g.add(c); }
  g.add(at(box(0.18, 0.12, 0.08, M.led), 4.0, 2.7, 1.12));
  g.add(at(box(0.18, 0.12, 0.08, new THREE.MeshStandardMaterial({ color: 0x24ff5a, emissive: 0x24ff5a, emissiveIntensity: 1.2 })), 3.75, 2.7, 1.12));
  // yellow tape over the jumper wires (photo)
  const tape = mesh(new THREE.CylinderGeometry(0.22, 0.22, 2.2, 12), new THREE.MeshStandardMaterial({ color: 0xe8d23a, roughness: 0.4 }));
  tape.rotation.z = Math.PI / 2 + 0.2; at(tape, -1.6, 2.0, 2.15); g.add(tape);
  return g;
}

/**
 * Bench power supply (the team's Eventek unit, photos/setup-desk-overview.jpg): vented grey
 * steel case, cream front panel with voltage and current read-outs, two knobs, banana sockets,
 * rocker switch and rubber feet. Origin: centre of the bottom, front facing +Y.
 */
function buildPSU(M) {
  const g = new THREE.Group();
  const Wd = 13, D = 24, Hh = 15;
  const body = new THREE.MeshPhysicalMaterial({ color: 0x6a6e74, metalness: 0.55, roughness: 0.5, clearcoat: 0.2 });
  g.add(at(box(Wd, D, Hh, body, 0.25), 0, 0, Hh / 2 + 0.4));
  // vent slots on both sides and on top
  const slot = new THREE.MeshStandardMaterial({ color: 0x1b1c1f, roughness: 0.8 });
  for (const sx of [-1, 1]) for (let i = 0; i < 9; i++) for (let j = 0; j < 2; j++) g.add(at(box(0.05, 2.8, 0.35, slot), sx * (Wd / 2 + 0.01), -7 + j * 7, 5 + i * 0.9));
  for (let i = 0; i < 8; i++) g.add(at(box(6, 0.35, 0.05, slot), 0, -8 + i * 1.6, Hh + 0.41));
  for (const [x, z] of [[-5.8, 1.1], [5.8, 1.1], [-5.8, 14.4], [5.8, 14.4]]) screwAt(g, M, V(x, D / 2 - 0.4, z), V(0, 1, 0), 0.18);
  // front panel texture: displays and labels
  const panelTex = canvasTexture(640, 740, (c, w, h) => {
    c.fillStyle = '#e9e3d2'; c.fillRect(0, 0, w, h);
    c.fillStyle = '#4b4f55'; c.font = 'bold 34px Arial'; c.fillText('EVENTEK', 40, 70);
    c.font = '20px Arial'; c.fillText('DC POWER SUPPLY  KPS305D', 40, 100);
    const disp = (y, text, label) => {
      c.fillStyle = '#141414'; c.fillRect(40, y, 300, 110);
      c.fillStyle = '#ff3b2a'; c.shadowColor = '#ff3b2a'; c.shadowBlur = 18; c.font = 'bold 86px "Courier New", monospace'; c.fillText(text, 60, y + 88); c.shadowBlur = 0;
      c.fillStyle = '#4b4f55'; c.font = 'bold 22px Arial'; c.fillText(label, 350, y + 65);
    };
    disp(150, '05.2', 'V'); disp(300, '0.54', 'A');
    c.font = '18px Arial'; c.fillText('VOLTAGE', 450, 200); c.fillText('CURRENT', 450, 360);
    c.fillText('COARSE   FINE', 440, 520); c.fillText('+          -', 450, 690); c.fillText('POWER', 60, 690);
  });
  const panel = box(Wd - 0.4, 0.3, Hh - 0.4, M.psuFront, 0.1); at(panel, 0, D / 2 + 0.15, Hh / 2 + 0.4); g.add(panel);
  // printed face: texture right = -X (the viewer's right when facing the front), up = +Z
  const face = mesh(new THREE.PlaneGeometry(Wd - 0.6, Hh - 0.6), new THREE.MeshStandardMaterial({ map: panelTex, roughness: 0.55 }), false);
  orient(face, V(-1, 0, 0), V(0, 0, 1), V(0, 1, 0)); at(face, 0, D / 2 + 0.31, Hh / 2 + 0.4); g.add(face);
  // knobs with knurled skirts
  const knobMat = new THREE.MeshStandardMaterial({ color: 0xd9d5cc, roughness: 0.45 });
  for (const [x, z, r] of [[-3.7, 11.2, 0.95], [-3.7, 7.6, 0.95], [-2.3, 4.4, 0.6], [-4.9, 4.4, 0.6]]) {   // viewer's right = -X
    const k = cyl(r, 1.1, knobMat, 'y', 32); at(k, x, D / 2 + 0.85, z); g.add(k);
    const sk = cyl(r * 1.12, 0.3, knobMat, 'y', 24); at(sk, x, D / 2 + 0.45, z); g.add(sk);
    const ln = box(0.08, 0.05, r * 0.8, M.chip); at(ln, x, D / 2 + 1.42, z + r * 0.45); g.add(ln);
  }
  // banana sockets (red +, black -) and the rocker switch
  for (const [x, mat] of [[-2.4, M.cableRed], [-4.9, M.wireBlack]]) {
    const b = cyl(0.42, 0.9, mat, 'y', 20); at(b, x, D / 2 + 0.6, 1.9); g.add(b);
    const h = cyl(0.16, 0.95, M.chip, 'y', 12); at(h, x, D / 2 + 0.62, 1.9); g.add(h);
  }
  g.add(at(box(1.2, 0.5, 1.6, M.wireBlack, 0.1), 4.6, D / 2 + 0.4, 1.9));
  g.add(at(box(0.9, 0.2, 0.6, M.cableRed, 0.05), 4.6, D / 2 + 0.7, 2.2));
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) { const ft = cyl(0.8, 0.4, M.wireBlack, 'z', 16); at(ft, sx * 5, sy * 10, 0.2); g.add(ft); }
  return g;
}

/** The OAK camera on its clamp arm, at the fitted position, looking at `target`. */
export function buildCamera(M, pos, target) {
  const g = new THREE.Group();
  const head = new THREE.Group();
  head.add(box(9.1, 2.6, 2.2, M.camBody, 0.45));
  for (const x of [-3.75, 0, 3.75]) { const l = cyl(x === 0 ? 0.55 : 0.38, 0.25, M.glass, 'y', 24); at(l, x, 1.3, 0); head.add(l); }
  head.position.set(...pos);
  head.quaternion.setFromUnitVectors(V(0, 1, 0), V(...target).sub(V(...pos)).normalize());
  g.add(head);
  // articulated stand down to the desk
  const base = V(pos[0] + 4, pos[1] + 9, -BOARD.t), mid = V(pos[0] + 5, pos[1] + 6, pos[2] * 0.55);
  for (const [a, b] of [[base, mid], [mid, V(...pos).add(V(0, 1.8, -1.2))]]) {
    const len = a.distanceTo(b);
    const rod = mesh(new THREE.CylinderGeometry(0.5, 0.5, len, 16), M.camBody);
    rod.position.copy(a).add(b).multiplyScalar(0.5);
    rod.quaternion.setFromUnitVectors(V(0, 1, 0), b.clone().sub(a).normalize());
    g.add(rod);
    const j = mesh(new THREE.SphereGeometry(0.85, 20, 14), M.camBody); j.position.copy(b); g.add(j);
  }
  g.add(at(box(5, 6, 1.2, M.camBody, 0.2), base.x, base.y, base.z + 0.6));
  return g;
}

/** A wooden test cube with slightly rounded edges. */
const woodCache = new Map();
export function buildCube(colour) {
  const hex = COLOURS[colour] ?? 0x999999;
  if (!woodCache.has(hex)) woodCache.set(hex, new THREE.MeshPhysicalMaterial({ map: woodTexture(hex), roughness: 0.62, clearcoat: 0.15, clearcoatRoughness: 0.6 }));
  const m = mesh(new RoundedBoxGeometry(CUBE, CUBE, CUBE, 3, 0.1), woodCache.get(hex));
  m.name = colour;
  return m;
}

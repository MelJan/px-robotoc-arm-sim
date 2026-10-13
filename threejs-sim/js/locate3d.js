// Cubes in 3D from two or more calibrated cameras: no assumption that they lie on the mat, so stacked
// cubes and a cube in the gripper are found at their real height.
//
// Each camera's cube detector (detect.js) gives a colour blob and its image centroid. For each colour the
// blobs are paired across cameras, their viewing rays intersected (least squares; the gap between the rays
// says how well they meet), and the result moved from the centroid of the visible faces to the cube's centre.
// Frames: calibrations in the camera table frame (mm), as calib.js; results also in the board frame (cm).

import { CUBE } from './config.js';
import { triangulate, tableToBoardPt } from './armtrack.js';
import { solve } from './calib.js';

const H_MM = CUBE * 10 / 2;   // half the cube side, mm

/** Camera centre and the unit ray through pixel uv, table frame. */
export function ray(cal, uv) {
  const d = [(uv[0] - cal.cx) / cal.f, (uv[1] - cal.cy) / cal.f, 1];
  const w = [0, 1, 2].map(i => cal.R[0][i] * d[0] + cal.R[1][i] * d[1] + cal.R[2][i] * d[2]);
  const n = Math.hypot(...w);
  const C = cal.centre || [0, 1, 2].map(i => -(cal.R[0][i] * cal.t[0] + cal.R[1][i] * cal.t[1] + cal.R[2][i] * cal.t[2]));
  return { C, d: w.map(v => v / n) };
}

/** Shortest distance between two rays (mm) and the midpoint of the closest points. */
export function rayGap(a, b) {
  const w0 = a.C.map((v, i) => v - b.C[i]);
  const A = a.d.reduce((s, v, i) => s + v * a.d[i], 0), B = a.d.reduce((s, v, i) => s + v * b.d[i], 0), Cc = b.d.reduce((s, v, i) => s + v * b.d[i], 0);
  const D = a.d.reduce((s, v, i) => s + v * w0[i], 0), E = b.d.reduce((s, v, i) => s + v * w0[i], 0);
  const den = A * Cc - B * B;
  const sa = den > 1e-12 ? (B * E - Cc * D) / den : 0, sb = den > 1e-12 ? (A * E - B * D) / den : E / Cc;
  const pa = a.C.map((v, i) => v + sa * a.d[i]), pb = b.C.map((v, i) => v + sb * b.d[i]);
  return { gap: Math.hypot(...pa.map((v, i) => v - pb[i])), mid: pa.map((v, i) => (v + pb[i]) / 2) };
}

/** The point closest to all rays { C, d (unit) } in the least-squares sense. */
export function intersectRays(rays) {
  const A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], b = [0, 0, 0];
  for (const { C, d } of rays) for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
    const m = (i === j ? 1 : 0) - d[i] * d[j];
    A[i][j] += m; b[i] += m * C[j];
  }
  return solve(A, b);
}

/**
 * Offset (mm) from a cube's centre to the centroid of its faces as seen from direction v (unit vector from
 * the cube to the camera): faces weighted by their projected area. Axis-aligned cube turned by yaw (deg).
 */
export function visibleCentroidOffset(v, yaw = 0, { top = true } = {}) {
  const c = Math.cos(yaw * Math.PI / 180), s = Math.sin(yaw * Math.PI / 180);
  const normals = [[c, s, 0], [-c, -s, 0], [-s, c, 0], [s, -c, 0]];   // the bottom face is never seen,
  if (top) normals.push([0, 0, 1]);                                     // the top one not under another cube
  let w = 0; const o = [0, 0, 0];
  for (const n of normals) { const k = n[0] * v[0] + n[1] * v[1] + n[2] * v[2]; if (k > 0) { w += k; for (let i = 0; i < 3; i++) o[i] += k * n[i] * H_MM; } }
  return w > 0 ? o.map(x => x / w) : [0, 0, 0];
}

/**
 * Locate cubes from several cameras. views: [{ cal, detections: detect.js results }].
 * Returns [{ colour, X (table mm), board (cm), level, gap (mm), cameras, yaw }] for every colour blob
 * seen by at least two cameras; blobs of one colour are paired by the smallest ray gap.
 */
export function locateCubes(views, { maxGap = 25 } = {}) {
  const usable = views.filter(v => v.cal && v.detections?.length);
  if (usable.length < 2) return [];
  const colours = [...new Set(usable.flatMap(v => v.detections.map(d => d.colour)))];
  const out = [];
  for (const colour of colours) {
    // blobs of this colour per camera, each with its ray
    const per = usable.map(v => v.detections.filter(d => d.colour === colour && d.uv).map(d => ({ d, cal: v.cal, r: ray(v.cal, d.uv), used: false })));
    // greedy pairing between the first two cameras that have this colour, others join by smallest gap
    const cams = per.filter(p => p.length);
    if (cams.length < 2) continue;
    const pairs = [];
    for (const a of cams[0]) for (const b of cams[1]) pairs.push({ a, b, ...rayGap(a.r, b.r) });
    pairs.sort((p, q) => p.gap - q.gap);
    for (const p of pairs) {
      if (p.a.used || p.b.used || p.gap > maxGap) continue;
      p.a.used = p.b.used = true;
      const members = [p.a, p.b];
      for (const more of cams.slice(2)) {
        const best = more.filter(m => !m.used).map(m => ({ m, ...rayGap(p.a.r, m.r) })).sort((x, y) => x.gap - y.gap)[0];
        if (best && best.gap < maxGap) { best.m.used = true; members.push(best.m); }
      }
      // intersect the rays through the blob centroids (the centre of the visible faces, not of the cube)
      const tri = triangulate(members.map(m => ({ cal: m.cal, uv: m.d.uv })));
      out.push({ colour, raw: tri.X, members, gap: p.gap, rms: tri.rms, cameras: members.length, yaw: members[0].d.yaw ?? 0 });
    }
  }
  // Shift each to its centre: the visible faces' centroid as seen from each camera. A cube with another
  // one right on top shows no top face; two passes, since "on top" needs the centres.
  // Each camera's ray passes through ITS visible-face centroid, centre + offset(camera): shift every ray
  // back by its own offset and intersect the shifted rays (least squares).
  const centre = (c, top) => intersectRays(c.members.map(m => {
    const v = m.r.C.map((x, i) => x - c.raw[i]), n = Math.hypot(...v);
    const off = visibleCentroidOffset(v.map(x => x / n), c.yaw, { top });
    return { C: m.r.C.map((x, i) => x - off[i]), d: m.r.d };
  }));
  for (const c of out) c.X = centre(c, true);
  for (const c of out) {
    const covered = out.some(o => o !== c && Math.hypot(o.X[0] - c.X[0], o.X[1] - c.X[1]) < H_MM && Math.abs(o.X[2] - c.X[2] - 2 * H_MM) < H_MM);
    if (covered) c.X = centre(c, false);
  }
  for (const c of out) {
    c.board = tableToBoardPt(c.X);
    c.level = Math.max(0, Math.round((c.X[2] - H_MM) / (2 * H_MM)));
    delete c.members; delete c.raw;
  }
  return out.sort((a, b) => a.level - b.level || a.colour.localeCompare(b.colour));
}

// The arm's pose from its tags in all cameras, without the page: the Camera tab runs it directly ("Track the
// arm"), live tracking runs it in js/trackworker.js so the page never waits for it.
//   findTags(image, cal, angles, opts)   the arm tags in one camera frame, searched around where `angles` puts them
//   solveArm(views, opts)                triangulated tags, each rigid part alone, wrist + gripper, the joint angles

import { ARM_TAGS, ARM_PARTS } from './config.js';
import { fk } from './kinematics.js';
import { detectArmTags, triangulate, fitArmPose, tagPoints, projectBoard, tableToBoardPt } from './armtrack.js';
import { fitPartPose } from './rigidtrack.js';
import { fitJointPair } from './jointfit.js';

/**
 * Tags in one frame (ImageData-like), in the calibration's pixel frame (w x h). The search window is the tags'
 * place for `angles`, padded: wide with a full-frame search as well, or (quick) tight and only that window.
 */
export function findTags(img, cal, angles, { quick = false, w = img.width, h = img.height } = {}) {
  const sx = w / img.width, sy = h / img.height;
  const uvs = ARM_TAGS.map(t => projectBoard(cal, tagPoints(t, angles).centre)).map(([u, v]) => [u / sx, v / sy]);
  const pad = (quick ? 0.06 : 0.18) * img.width;
  const roi = [Math.min(...uvs.map(p => p[0])) - pad, Math.min(...uvs.map(p => p[1])) - pad, Math.max(...uvs.map(p => p[0])) + pad, Math.max(...uvs.map(p => p[1])) + pad];
  const found = detectArmTags(img, undefined, { roi, full: !quick });
  return Object.fromEntries(Object.entries(found).map(([id, t]) => [id, { corners: t.corners.map(([x, y]) => [x * sx, y * sy]), centre: [t.centre[0] * sx, t.centre[1] * sy] }]));
}

/**
 * views: [{ cal, tags }] (cameras that saw a tag). commanded: the joint panel's angles; last: the last tracked
 * pose (or null). Returns { tri, parts, pair, result, notes }; result is null with too few tag views.
 */
export function solveArm(views, { commanded, last = null, useCorners = true } = {}) {
  const notes = [];
  // a tag seen by two or more cameras: its 3D position, independent of the arm model
  const tri = {};
  for (const t of ARM_TAGS.filter(x => x.id != null)) {
    const v = views.filter(x => x.tags[t.id]).map(x => ({ cal: x.cal, uv: x.tags[t.id].centre }));
    if (v.length >= 2) { const r = triangulate(v); tri[t.id] = { board: tableToBoardPt(r.X), rms: r.rms }; }
  }
  // each rigid part on its own, from all its tags in all cameras (rigidtrack.js), started from the commanded pose
  const f0 = fk(commanded), parts = {};
  for (const link of Object.keys(ARM_PARTS)) {
    try { const pf = fitPartPose(link, views, { init: f0[link] }); if (pf) parts[link] = pf; } catch (e) { notes.push(`${link}: ${e.message}`); }
  }
  const shifts = Object.assign({}, ...Object.values(parts).map(p => p.shifts));
  // wrist + gripper together: wrist pose and S5 (7 numbers) from the tags of both parts
  let pair = null;
  try { pair = fitJointPair('s5', views, { init: { parent: f0.wrist, child: f0.hand } }); } catch { pair = null; }
  let result = null;
  try {
    if (views.length >= 2 || views.reduce((n, x) => n + Object.keys(x.tags).length, 0) >= 2) {
      // from the last tracked pose and from the joint panel; the better fit wins (either can be far from the real arm)
      const fitFrom = init => fitArmPose(views, { init, useCorners, shifts });
      result = [last, commanded].filter(Boolean).map(fitFrom).sort((x, y) => x.rms - y.rms)[0];
      // both far off (an arm moved by hand): also try the wrist turned in 45 degree steps
      if (result.rms > 6) for (let s5 = -180; s5 < 180; s5 += 45) { const r = fitFrom({ ...result.angles, s5 }); if (r.rms < result.rms) result = r; }
      result.triangulated = tri;
    }
  } catch (e) { notes.push(e.message); }
  return { tri, parts, pair, result, notes };
}

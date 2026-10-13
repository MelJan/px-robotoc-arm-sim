// Live arm tracking off the page's main thread. The page keeps one of these workers per camera, so the cameras'
// frames are fetched, decoded and searched for tags at the same time on separate CPU cores; the joint fit, which
// needs all cameras, then runs in one of them.
//   { type: 'tags', cam: { id, url, cal }, start, quick, w, h }  one snapshot (GET only) and the arm tags in it
//   { type: 'solve', views: [{ id, cal, tags }], commanded, last, useCorners }  js/armsolve.js solveArm
// Every message carries the page's current tag layout and mat offset (both can change in the page).

import { ARM_TAGS, MAT } from './config.js';
import { findTags, solveArm } from './armsolve.js';

self.onmessage = async ({ data: q }) => {
  const t0 = performance.now();
  for (const t of q.layout) Object.assign(ARM_TAGS.find(x => x.key === t.key) || {}, t);
  MAT.sToEdge = q.d;
  if (q.type === 'tags') {
    const c = q.cam;
    try {
      const r = await fetch(c.url, { cache: 'no-store' });
      if (!r.ok) throw new Error(`HTTP ${r.status}`);
      const blob = await r.blob();
      const bmp = await createImageBitmap(blob);
      const canvas = new OffscreenCanvas(bmp.width, bmp.height), g = canvas.getContext('2d', { willReadFrequently: true });
      g.drawImage(bmp, 0, 0); bmp.close();
      const img = g.getImageData(0, 0, canvas.width, canvas.height);
      const t1 = performance.now();
      const tags = findTags(img, c.cal, q.start, { quick: q.quick, w: q.w, h: q.h });
      self.postMessage({ seq: q.seq, id: c.id, blob, tags, ms: { frame: Math.round(t1 - t0), tags: Math.round(performance.now() - t1) } });
    } catch (e) { self.postMessage({ seq: q.seq, id: c.id, blob: null, tags: {}, error: e.message, ms: {} }); }
  } else if (q.type === 'solve') {
    const solved = solveArm(q.views, { commanded: q.commanded, last: q.last, useCorners: q.useCorners });
    self.postMessage({ seq: q.seq, solved, ms: { fit: Math.round(performance.now() - t0) } });
  }
};

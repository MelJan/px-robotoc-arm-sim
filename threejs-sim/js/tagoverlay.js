// Overlay that shows which tags sit on which rigid part of the arm and what moves that part
// (config.js ARM_TAGS, ARM_PARTS): tag outlines in the part's colour, a line joining the tags of one part
// (they act as one object), a line to the joint axis that moves the part, the axis itself, and labels.

import * as THREE from 'three';
import { ARM_TAGS, ARM_PARTS } from './config.js';
import { fk, point } from './kinematics.js';
import { tagPoints } from './armtrack.js';

function label(text, colour) {
  const c = document.createElement('canvas'); c.width = 1024; c.height = 64;
  const g = c.getContext('2d');
  g.font = 'bold 30px system-ui';
  const w = Math.min(1020, g.measureText(text).width + 24);
  g.fillStyle = 'rgba(20,24,30,0.78)'; g.beginPath(); g.roundRect(0, 6, w, 52, 10); g.fill();
  g.fillStyle = '#' + new THREE.Color(colour).getHexString(); g.fillRect(0, 6, 8, 52);
  g.fillStyle = '#fff'; g.fillText(text, 16, 42);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: t, depthTest: false, transparent: true }));
  s.scale.set(0.9 * 1024 / 64, 0.9, 1); s.center.set(0, 0.5); s.renderOrder = 10;   // 0,9 cm tall text band
  return s;
}

/** Builds the overlay group; update(angles) moves it with the arm. */
export function buildTagOverlay() {
  const group = new THREE.Group();
  group.name = 'tag-overlay';
  const lineMat = c => new THREE.LineBasicMaterial({ color: c, depthTest: false, transparent: true, opacity: 0.95 });
  const items = ARM_TAGS.map(t => {
    const colour = ARM_PARTS[t.link].colour;
    const outline = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints([0, 1, 2, 3].map(() => new THREE.Vector3())), lineMat(colour));
    const toAxis = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), lineMat(colour));
    toAxis.material.opacity = 0.5;
    const txt = label(`tag ${t.id ?? '(reprint)'}: ${t.on}`, colour);
    for (const o of [outline, toAxis, txt]) { o.renderOrder = 10; o.frustumCulled = false; group.add(o); }
    return { t, outline, toAxis, txt };
  });
  const parts = Object.entries(ARM_PARTS).map(([link, p]) => {
    const join = new THREE.Line(new THREE.BufferGeometry().setFromPoints(ARM_TAGS.filter(t => t.link === link).map(() => new THREE.Vector3())), lineMat(p.colour));
    join.material.opacity = 0.7;
    const axis = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), lineMat(p.colour));
    const txt = label(`${p.name} — moved by ${p.joints}, turns about ${p.last}`, p.colour);
    for (const o of [join, axis, txt]) { o.renderOrder = 10; o.frustumCulled = false; group.add(o); }
    return { link, p, join, axis, txt };
  });

  function update(angles) {
    const f = fk(angles), V = a => new THREE.Vector3(...a);
    const axisPoint = link => point(f[ARM_PARTS[link].axis[0]], ARM_PARTS[link].axis[1]);
    for (const it of items) {
      const tp = tagPoints(it.t, angles, f);
      it.outline.geometry.setFromPoints(tp.corners.map(V));
      it.toAxis.geometry.setFromPoints([V(tp.centre), V(axisPoint(it.t.link))]);
      const n = V(point(f[it.t.link], it.t.centre.map((c, i) => c + 2.5 * it.t.normal[i])));
      it.txt.position.copy(n);
    }
    for (const pt of parts) {
      const tags = ARM_TAGS.filter(t => t.link === pt.link).map(t => V(tagPoints(t, angles, f).centre));
      pt.join.geometry.setFromPoints(tags);
      // the joint axis that moves the part: S4 (along the elbow frame's X), S5 (along the wrist frame's Z)
      const fr = f[pt.p.axis[0]], c = pt.p.axis[1];
      const dir = pt.link === 'wrist' ? [1, 0, 0] : [0, 0, 1], h = pt.link === 'wrist' ? 4 : 2.5;
      const a = point(fr, c.map((v, i) => v - h * dir[i])), b = point(fr, c.map((v, i) => v + h * dir[i]));
      pt.axis.geometry.setFromPoints([V(a), V(b)]);
      pt.txt.position.copy(V(b)).add(new THREE.Vector3(0, 0, 2));
    }
  }
  return { group, update };
}

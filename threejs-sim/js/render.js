// Renderer, lights and environment. One fast path, no post-processing.
// Look: a bright office desk lit by a window (key light with soft shadows), a ceiling fill,
// and two rim lights from behind that outline the black arm against the desk; the arm's
// materials add a cool sheen at grazing angles (model.js) so its silhouette stays readable.

import * as THREE from 'three';
import { OrbitControls } from '../vendor/OrbitControls.js';

/** A studio-like room for reflections: softboxes and a window strip on a grey box. */
function studioEnvironment(renderer) {
  const env = new THREE.Scene();
  env.add(new THREE.Mesh(new THREE.BoxGeometry(60, 60, 60), new THREE.MeshBasicMaterial({ color: 0x5a5d62, side: THREE.BackSide })));
  const panel = (w, h, intensity, pos, look, tint = 0xffffff) => {
    const m = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshBasicMaterial({ color: new THREE.Color(tint).multiplyScalar(intensity), side: THREE.DoubleSide }));
    m.position.set(...pos); m.lookAt(...look); env.add(m);
  };
  panel(24, 10, 6, [0, 28, 6], [0, 0, 0]);                 // ceiling softbox (Y is up in this scene)
  panel(10, 10, 4, [-18, 22, -10], [0, 0, 0]);
  panel(40, 14, 3.5, [0, 10, -29], [0, 10, 0], 0xf4f8ff);   // window wall
  panel(14, 20, 1.6, [29, 8, 8], [0, 8, 8], 0xfff3e6);      // warm bounce
  panel(60, 60, 0.9, [0, -29, 0], [0, 0, 0], 0xeeeeee);     // desk bounce from below
  const pmrem = new THREE.PMREMGenerator(renderer);
  const tex = pmrem.fromScene(env, 0.035).texture;
  pmrem.dispose();
  return tex;
}

function gradientBackground() {
  const c = document.createElement('canvas'); c.width = 4; c.height = 512;
  const g = c.getContext('2d');
  const grad = g.createLinearGradient(0, 0, 0, 512);
  grad.addColorStop(0, '#eef1f4'); grad.addColorStop(0.6, '#f7f8f9'); grad.addColorStop(1, '#ffffff');
  g.fillStyle = grad; g.fillRect(0, 0, 4, 512);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

export function createView(canvas) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.0;

  const scene = new THREE.Scene();
  scene.background = gradientBackground();
  scene.environment = studioEnvironment(renderer);
  scene.environmentRotation.set(Math.PI / 2, 0, 0);   // env built Y-up, world is Z-up
  scene.environmentIntensity = 0.9;

  const camera = new THREE.PerspectiveCamera(36, 1, 0.5, 1500);
  camera.up.set(0, 0, 1);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.dampingFactor = 0.1;
  controls.minDistance = 6;
  controls.maxDistance = 240;
  controls.maxPolarAngle = Math.PI * 0.495;    // never below the desk
  controls.screenSpacePanning = true;

  // Key light: the window, high and to the arm's front left; the only shadow caster.
  const key = new THREE.DirectionalLight(0xfff6ea, 2.4);
  key.position.set(-30, 45, 70); key.target.position.set(0, 12, 0);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  Object.assign(key.shadow.camera, { left: -34, right: 34, top: 34, bottom: -34, near: 20, far: 200 });
  key.shadow.bias = -0.0003; key.shadow.normalBias = 0.03;
  scene.add(key, key.target);
  const hemi = new THREE.HemisphereLight(0xf2f6ff, 0xb9b4aa, 0.55); hemi.position.set(0, 0, 100); scene.add(hemi);
  // Rim lights from behind, left cool and right warm: bright edges on the black brackets.
  const rimL = new THREE.DirectionalLight(0xcfe0ff, 2.4); rimL.position.set(-35, -55, 45); scene.add(rimL);
  const rimR = new THREE.DirectionalLight(0xffe4c8, 1.6); rimR.position.set(45, -40, 35); scene.add(rimR);
  const fill = new THREE.DirectionalLight(0xffffff, 0.45); fill.position.set(60, 40, 25); scene.add(fill);

  const resize = () => {
    const r = canvas.parentElement.getBoundingClientRect();
    const w = Math.max(1, r.width), h = Math.max(1, r.height);
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  };
  const render = () => renderer.render(scene, camera);
  return { renderer, scene, camera, controls, render, resize };
}

# PX robot arm simulator — Three.js branch

This is the `feat/threejs-vk` branch of the PX robot-arm simulator repository.
It contains the browser-based Three.js implementation developed independently
from the team's Blender, MuJoCo and Unreal simulator experiments.

The application is a visual and kinematic digital twin. It renders a simplified
S1-S6 robot arm, accepts servo counts through sliders, animates reference poses
and reports the tool-centre-point position and board clearance.

It is simulation-only and cannot connect to or command the physical robot.

## What is included

- Generated Three.js geometry for the board, base, links and gripper
- A hierarchical S1-S6 forward-kinematic chain
- HOME, PARK and straight-up reference poses
- Servo-count sliders with provisional count-to-angle conversion
- Animated HOME-to-PARK transition
- TCP position, board-clearance and gripper-gap readouts
- Orbit, zoom, camera reset and joint-axis helpers
- Responsive desktop and tablet interface

## Run the simulator

Requirements: Node.js 20 or newer, npm and a WebGL-capable browser.

```sh
cd web
npm install
npm run dev
```

Open the URL printed by Vite, normally <http://localhost:5173>. Stop the local
server with `Ctrl+C`. After the first installation, future launches only need
`npm run dev`.

Build the static production version with:

```sh
cd web
npm run build
```

## Branch layout

| Path | Purpose |
|------|---------|
| `web/src/main.js` | User interface, Three.js scene graph, joint updates and animation |
| `web/src/style.css` | Responsive visual design |
| `web/public/arm-spec.json` | Browser-readable dimensions, poses and provisional calibration |
| `web/README.md` | Installation, controls and usage guide |
| `docs/threejs-implementation.md` | Detailed architecture and implementation notes |

## Calibration status

Physical dimensions are based on the measurements recorded by the team. Servo
zero points, directions and some usable ranges remain provisional. The model is
appropriate for visualization and forward-kinematic experiments, but it must
not yet be treated as an exact predictor of the physical arm.

The intended next step is to generate `web/public/arm-spec.json` from the
repository's canonical arm specification so every simulator consumes the same
measurements and calibration values.

See [the web usage guide](web/README.md) to operate the simulator and
[the Three.js implementation document](docs/threejs-implementation.md) for the
complete technical design.

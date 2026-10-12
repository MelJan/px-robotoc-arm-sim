# PX-robo-arm-simulator

Interactive Three.js digital twin of the measured PX robot arm. It renders a
simplified S1-S6 kinematic chain in the browser and lets a user explore servo
counts, reference poses, tool-centre-point position and board clearance.

This application is deliberately offline. It does not import the physical-arm
API and cannot send commands to the robot.

## Requirements

- Node.js 20 or newer
- npm
- A browser with WebGL support

## Run locally

From the repository root:

```sh
cd web
npm install
npm run dev
```

Open the URL printed by Vite, normally <http://localhost:5173>. Keep the
terminal running while using the simulator. Stop it with `Ctrl+C`.

`npm install` is only necessary after the first checkout or when dependencies
change. On later runs, `npm run dev` is enough.

## Controls

- Drag in the viewport to orbit the camera.
- Scroll or pinch to zoom.
- Use the S1-S6 sliders to change commanded servo counts.
- Select **HOME**, **PARK** or **Straight-up** to animate to a reference pose.
- Select **Play HOME -> PARK** to demonstrate a complete pose transition.
- Toggle **Joint axes** to show or hide the world and joint-axis helpers.
- Select **Reset view** to restore the initial camera position.

The TCP readout reports the gripper tool-centre point in centimetres. Clearance
is measured from the board's top plane. The S6 readout displays the estimated
gap between the gripper pads rather than an angle.

## Production build

```sh
cd web
npm run build
```

The static production site is written to `web/dist/`. Preview that build with:

```sh
npm run preview
```

## Project layout

```text
web/
├── index.html                 Browser entry point and page title
├── package.json              npm scripts and Three.js dependency
├── public/
│   └── arm-spec.json         Browser-readable dimensions and calibration
└── src/
    ├── main.js               UI, Three.js scene and kinematic updates
    └── style.css             Responsive application styling
```

For the implementation details, coordinate conventions and extension points,
see [`../docs/threejs-digital-twin.md`](../docs/threejs-digital-twin.md).

## Calibration status

`public/arm-spec.json` currently contains measured geometry and provisional
tick-to-angle calibration. The interface labels this explicitly. Do not treat
the rendered pose as an exact prediction until every servo zero, direction and
usable range has been verified against the physical arm.

The repository's long-term source of truth is `spec/arm.toml`. The JSON file
should eventually be generated from that TOML specification during the build,
rather than maintained independently.

## Safety boundary

The web simulator has no network calls except loading its own static JSON file.
It contains no WebSocket, serial, GPIO or robot-control client. Adding live
robot connectivity should be a separate, explicitly enabled adapter with
limits, an emergency stop and clear simulated/live mode indicators.

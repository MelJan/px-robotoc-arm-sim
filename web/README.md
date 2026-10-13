# PX-robo-arm-simulator

The browser application is the Three.js implementation of the PX robot-arm
digital twin. It combines a photo-informed visual model with millimetre-based
kinematics, inverse kinematics, collision checking, trajectory planning and a
Rapier3D physics world. It uses Vite and plain JavaScript; no frontend framework
or Blender runtime is involved.

Simulation is the default. Merely opening the page never prepares or commands
the physical arm.

## Run

Requirements: Node.js 20 or newer and a WebGL-capable browser.

```sh
cd web
npm install
npm run dev
```

Open the URL printed by Vite, normally <http://localhost:5173>. Other commands:

```sh
npm test
npm run build
npm run preview
```

Rapier's compatibility package embeds its WebAssembly runtime. It is loaded as
a separate lazy chunk, so Vite reports a non-fatal large-chunk warning during a
production build.

## What is implemented

- detailed generated geometry for the board, raised two-plate fixture,
  brackets, horns, servos, fasteners, cable runs and parallel-link claw;
- four dynamic 26.5 mm solid cubes;
- pure millimetre-based S1-S6 forward kinematics and numerical IK;
- capsule and box collision checks for the board, fixture, workpieces and
  non-adjacent robot parts;
- a deterministic optimizing planner with direct/staged seeds, RRT*-style
  search, shortcutting and STOMP-style smoothing;
- weighted motion cost covering clearance, changed servos, count travel,
  estimated duration, acceleration/jerk proxy and TCP distance;
- text pick/place tasks with approach, grasp, lift, place, release and retreat
  waypoints;
- collision verification of every sampled path segment;
- coordinated virtual playback and controller-compatible one-servo-at-a-time
  playback;
- Rapier fixed board/fixture colliders, kinematic arm colliders, dynamic cubes
  and simulated grasp/release;
- HOME, observed commissioning PARK and straight-up references; and
- read-only gripper-marker tracking that compares planned, simulated and camera-observed TCP positions; and
- simulation, connected read-only, operational-disarmed, operational-armed and
  fault/stopped modes.

The planner optimizes the configured engineering cost. It does not claim a
mathematical proof of the globally shortest path or literally solve a
continuous least-action variational problem.

## Controls

The first card contains **HOME**, **PARK** and **STRAIGHT UP** positioning.
Each transition is planned and collision-verified before virtual playback.
Reference-pose planning includes the live cube positions. For task playback,
the return to HOME uses an additional 6 mm detailed-model clearance margin and
inserts a raised transit when the direct return passes too close to a stack.
STRAIGHT UP is a provisional open-gripper modelling pose and is intentionally
simulation-only. PARK is modelled as supported contact on the raised fixture
plate, whose provisional photo-derived top is 25 mm above the board. Joint
sliders below show servo counts and provisional model angles.

The Task card includes a preset dropdown for the verified default-scene
examples. Selecting an entry only fills the task command: use **Plan** to
calculate and collision-check it, inspect the result, and then use **Run in
sim**. Reset the Scene before treating each preset as an independent example.
Reset replaces the Rapier world atomically, so stale cube transforms cannot be
written back while the new world loads.

Enter a task such as:

```text
Move the red cube to the blue zone
Move the orange cube to (80, 240) mm
```

Select **Plan**, inspect the path and verification result, then select **Run in
sim**. The planner first scores its millimetre RRT*/smoothed candidate against
the detailed bracket/gripper collision model. If that candidate is rejected,
the UI uses the photo-model-aware fallback only after both collision models
verify it. The playback selector switches between simultaneous joint
interpolation and a sequence in which only one servo command changes at a time.

The guarded real-mode controls are under **Tools → Mode & robot API** so the
restored dark split-pane layout, camera tools and scene controls remain intact.

## Gripper marker tracking

The Camera tab recognizes the dedicated 30 mm OpenCV `DICT_4X4_50` marker ID 6.
It estimates the marker pose in the mat frame, applies the operator-measured
marker-to-TCP offset, and reports planned, simulated and observed TCP positions
in millimetres. A printable SVG is included at
`public/gripper-marker-id6.svg`. See
[the setup guide](../docs/gripper-marker-tracking.md) before attaching it.

Saved/local images work offline. **Connected camera snapshot** uses the guarded
gateway's read-only `/api/camera/snapshot.jpg` route. It remains opt-in and does
not enable robot motion. Connected continuous tracking is limited to 1 Hz.

## Operational mode

The optional adapter targets the existing `lutich/robot-arm` FastAPI contract:

- `GET /api/state` for a read-only connection;
- `POST /api/power_on` only after explicit supported-PARK confirmation;
- `POST /api/move` for verified single-servo steps;
- `POST /api/power_off` to return to PARK and disable outputs; and
- `POST /api/stop` for an emergency stop.

### Connection commands

Build before running the production gateway:

| Purpose | Command | POST policy |
| --- | --- | --- |
| Simulation only | `npm run serve` | no controller configured |
| Inspect real state/camera | `npm run serve -- --controller http://ROBOT_HOST:8765` | camera/state GET and `/api/stop` pass |
| Attended operation | `npm run serve -- --controller http://ROBOT_HOST:8765 --allow-motion` | only stop, power and single-joint move routes pass |

The gateway binds to `127.0.0.1:8770` by default. Use `--host` and `--port` only
when required. `GET /connection-info` reports the active gateway policy without
contacting the controller.

The controller requires same-origin POSTs. `server.mjs` supplies that boundary
and keeps motion disabled unless `--allow-motion` is present. The browser still
requires: read-only connection → operational/disarmed selection → fresh PARK
confirmation → arming. Starting the gateway never connects, powers or moves the
arm.

The client rejects malformed or non-JSON state, requires six channels, polls
connection health after an explicit connection, validates targets against the
backend session bounds, confirms every commanded count and attempts `/api/stop`
if operational playback fails.

The physical arm has no position feedback. Connected values are always labelled
as **commanded pose**, never measured pose. Operational execution additionally
requires an exact match between the plan's first waypoint and the backend's six
held commands. Server-side bounds and guards remain authoritative.

### Operator checklist

1. Keep a person at the physical power switch and clear the workspace.
2. Start the gateway without `--allow-motion`; verify read-only state first.
3. Restart with `--allow-motion` only for an attended test.
4. Confirm the physical arm is supported in PARK before arming.
5. Re-plan whenever backend commanded counts or the scene change.
6. Use the physical power switch if browser stop cannot be confirmed.

## Specification status

`public/arm-spec.json` schema version 2 uses millimetres throughout. It keeps
measured dimensions, provisional assumptions, observed commissioning poses and
historical unvalidated bounds in separate provenance categories. It contains
no validated safe limits.

The physical HOME and PARK records both use S6=650. The S6 490 open count is a
gripper modelling endpoint, not a replacement HOME command. PARK remains an
observed commissioning reference and is not declared motion-validated or safe
for unattended hardware execution.

See [the implementation document](../docs/threejs-implementation.md) for the
architecture and validation details. See
[the photo-reference guide](../docs/arm-photo-reference.md) for the physical-arm
evidence map and reproducible overlay command.

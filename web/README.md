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
- simulation, connected read-only, operational-disarmed, operational-armed and
  fault/stopped modes.

The planner optimizes the configured engineering cost. It does not claim a
mathematical proof of the globally shortest path or literally solve a
continuous least-action variational problem.

## Controls

Joint sliders show servo counts and provisional model angles. **Go HOME** and
**Go PARK** plan and verify the requested transition before playback. PARK is
modelled as supported contact on the raised fixture plate, whose provisional
photo-derived top is 25 mm above the board.

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

## Operational mode

The optional adapter targets the existing `lutich/robot-arm` FastAPI contract:

- `GET /api/state` for a read-only connection;
- `POST /api/power_on` only after explicit supported-PARK confirmation;
- `POST /api/move` for verified single-servo steps;
- `POST /api/power_off` to return to PARK and disable outputs; and
- `POST /api/stop` for an emergency stop.

The hardware backend requires POST requests to originate from the same origin.
Therefore a Vite page on another port cannot control it. The production app
must be served by, or reverse-proxied through, the robot backend before the
connected modes can work. The UI validates that `/api/state` is a real robot
state document and never falls through to Vite's HTML page.

The physical arm has no position feedback. Connected values are always labelled
as **commanded pose**, never measured pose. Operational execution additionally
requires an exact match between the plan's first waypoint and the backend's six
held commands. Server-side bounds and guards remain authoritative.

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
architecture and validation details.

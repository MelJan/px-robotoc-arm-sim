# Three.js implementation

Status: modular simulation and planning baseline, 2026-10-12.

This document describes the browser implementation in `web/`. It remains
independent of Blender, MuJoCo and Unreal. Three.js renders the scene, the pure
core modules calculate and verify motion, and Rapier3D supplies contact and
rigid-body simulation.

## 1. Scope and safety

The application supports virtual forward/inverse kinematics, collision-aware
planning, task playback and dynamic cube interaction. Simulation is always the
startup mode. Loading the page performs no robot API call.

An optional operational adapter uses the existing `lutich/robot-arm` HTTP API.
It never accesses PCA9685, GPIO or serial hardware directly. The state machine
is:

```text
simulation (default)
    -> connected read-only
    -> operational disarmed
    -> operational armed
    -> connected read-only after power-off

any connected state -> fault/stopped after emergency stop or connection fault
```

Read-only connection is a GET. Arming needs a separate operational selection,
a fresh supported-PARK checkbox and backend acceptance of `power_on`. Arbitrary
coordinated browser trajectories are never sent to hardware: operational
playback is converted to collision-verified, one-servo-at-a-time `move`
requests. Before the first request, all six plan-start counts must equal the
backend's held commands. The backend still owns power sequencing, limits,
motion admission, cancellation and output shutdown.

The physical servos do not report position. UI and documentation therefore say
**commanded pose**, not actual or measured pose.

## 2. Architecture

```text
index.html                          preserved dark split-pane application shell
style.css                           preserved responsive visual design
js/app.js                           active UI, camera/model and runtime integration
js/model/                           detailed robot, fixture, mat and cable rendering
js/render.js                        preserved camera, lighting and renderer
js/advanced.js                      canonical-mm/photo-model calibration adapter
js/kinematics.js                    photo-fitted visual kinematics (centimetres)
js/collision.js, js/planner.js      detailed-geometry verification/fallback planning
src/config/spec.js                  canonical schema validation
src/core/math.js                    dependency-free vector/transform helpers
src/core/kinematics.js              count conversion, FK, numerical IK
src/core/collision.js               capsules, boxes and sampled path checks
src/core/planning.js                tasks, costs, search and path conversion
src/core/controller.js              smooth virtual trajectory playback
src/core/physics.js                 lazy Rapier3D world adapter
src/operational/client.js           guarded robot API/state-machine adapter
src/operational/gateway.js          proxy endpoint allowlist
server.mjs                          production assets + guarded same-origin proxy
public/arm-spec.json                versioned millimetre specification
test/*.test.js                      Node test suite
```

Core modules contain no Three.js objects. They operate on plain arrays and
servo-count records, so tests and future simulator comparisons can execute
without WebGL. `js/advanced.js` derives planner calibration anchors from the
preserved photo-fitted render hierarchy; this prevents the millimetre planner
from silently describing a differently posed arm. Joint counts and provenance
metadata are not scaled.

Rapier is dynamically imported. This keeps its embedded WebAssembly runtime in
a separate build chunk and allows the UI/core planner to remain independently
loadable.

There is one application shell: `index.html`, `style.css` and `js/app.js`.
The superseded `src/main.js`/`src/style.css` shell was removed; `src/core` and
`src/operational` remain because the active shell imports them directly.

## 3. Units and coordinates

`arm-spec.json` and the canonical kinematics, collision and planning modules
use millimetres. Rapier receives metres at its adapter boundary because it
assumes SI-style scene scale. The preserved detailed renderer uses centimetres;
the adapter converts its fitted link geometry and current cube positions at the
canonical boundary. This deliberate bridge keeps the imported model, camera
and lighting byte-for-byte stable while avoiding mixed units in planning.

- origin: board top below S1;
- +Z: up;
- +Y: from the back edge toward the front of the board; and
- +X: across the 300 mm board width.

The camera uses `camera.up.set(0, 0, 1)`. Readouts convert Three.js metres to
centimetres.

## 4. Specification and provenance

Schema version 2 validates these distinctions:

- measured dimensions;
- provisional geometry/calibration assumptions;
- observed commissioning HOME/PARK counts;
- historical, unvalidated reference bounds; and
- validated safe limits (currently empty).

The physical reference records are:

```text
HOME: S1 320, S2 427, S3 403, S4 220, S5 480, S6 650
PARK: S1 330, S2 540, S3 580, S4 195, S5 480, S6 650
```

S6=490 remains the visual fully-open endpoint. It is not substituted into the
HOME record. PARK remains an observed supported commissioning pose, not a
validated transition, physical angle zero or proof of safe limits.

## 5. Visual model

The renderer creates geometry from Three.js boxes, cylinders, tubes and edge
outlines. It includes:

- a 300 × 380 × 10 mm calibration mat/board with procedural grid and fiducials;
- lower fixture plate fixed to the board;
- four standoffs and a raised upper plate with a provisional 25 mm top;
- open U-brackets, black servo cases, silver horns/collars and fasteners;
- photo-informed orange, black and pink cable runs;
- paired gripper plates, actuator, drive teeth, link bars, pivots and pads; and
- four solid 26.5 mm colored workpieces.

The cosmetic detail is photo-informed, not surveyed CAD. It does not silently
replace the measured joint-origin translations.

| Evidence | Useful for | Not proof of |
| --- | --- | --- |
| Mat-calibrated photo overlay | pose, camera and gross link alignment | safe clearance or exact part dimensions |
| Dimensioned drawing | bracket and link dimensions | assembled tolerances or cable envelope |
| Close-up photograph | part presence, orientation and cable routing | metric scale without a reference |
| Collision test | implemented envelope behaviour | real safety without surveyed geometry |

The visualization is the preserved `codex/threejs-unified` presentation: dark
split-pane desktop layout, resizable control panel, Perspective/Top/Side/OAK
camera views and the existing photo-informed robot. Below 980 px the layout
becomes a single column without replacing or rebuilding the scene.

## 6. Kinematics

The transform chain is:

```text
board -> S1 Z yaw -> 96 mm -> S2 -X pitch -> 105 mm
      -> S3 -X pitch -> 126 mm -> S4 -X pitch -> 67 mm
      -> S5 Z roll -> S6 interpolated tool length -> TCP
```

Linear count conversion is:

```text
degrees = direction * (tick - zero) / ticks_per_degree
```

S2-S4 use piecewise-linear photo anchors where present. These anchors remain
provisional. The PARK S4 anchor is fitted to the raised support surface and is
tagged `PARK-photo-raised-fixture-contact-fit`.

S6 linearly interpolates closure, pad gap and tool length between counts 490
and 650. This is a visual linkage approximation.

Inverse kinematics uses deterministic multi-start coordinate refinement in
count space. It solves TCP position for S1-S4 while retaining S5 and selecting
the requested S6 state. Every returned result includes residual error and a
reachable flag based on the configured millimetre tolerance. Orientation IK is
not yet claimed.

## 7. Collision model

The analytical collision layer uses:

- capsules for pedestal and link envelopes;
- two separated thin capsules for the front/back gripper plates;
- boxes for the fixture and solid cubes;
- board-plane clearance;
- selected non-adjacent capsule pairs for self-collision; and
- a pedestal body box checked against the claw.

Using separated claw plates is significant: a single fat center capsule falsely
passed through the physical gap and reported a severe PARK/pedestal overlap.
The revised shapes describe the actual paired plates rather than adding a PARK
exception.

PARK is a special *contact state*, not free-space clearance. A gripper/upper
plate contact inside the 3 mm provisional modelling margin is reported as zero
clearance. Penetration beyond that margin remains invalid. The current PARK
endpoint verifies at zero clearance on the upper plate.

Paths are sampled according to maximum servo-count delta. Any collision makes
the segment invalid. Pick/place verification allows only gripper contact with
the selected cube. After grasp, that cube is removed from its static location
and a carried box is checked against the board, fixture, other cubes and arm.
At placement, board support inside the modelling margin is zero-clearance
contact; after release the box is checked at its destination.

These primitives are conservative approximations. They are not a replacement
for measured CAD envelopes or physical safety validation.

## 8. Planning and cost

The planner follows this sequence:

1. Try a direct verified joint-space edge.
2. Try a raised/open clearance seed for folded poses.
3. If needed, run deterministic RRT*-style sampling with parent selection and
   local rewiring.
4. Apply random shortcutting with a seeded generator.
5. Apply repeated local smoothing inspired by STOMP/trajectory optimization.
6. Re-verify every output segment.

The cost includes:

- reciprocal clearance penalty;
- count of servos changed per segment;
- total absolute count travel;
- duration estimate;
- acceleration/jerk proxy;
- TCP path length; and
- collision as an invalid/infinite-cost condition.

This is a practical least-effort/least-adjustment engineering objective. The
implementation does not claim a proof of the globally shortest path and does
not literally solve Hamilton's principle of least action with a calibrated
mass/energy model.

Controller-compatible conversion subdivides coordinated edges into small
samples, then greedily selects the collision-free single-servo change with the
greatest clearance. Conversion fails closed if no valid ordering exists.

## 9. Text tasks

The parser recognizes a cube color and either a named color zone or explicit
board coordinates:

```text
Move the red cube to the blue zone
Move orange to (80, 240) mm
```

A pick/place task expands into:

```text
approach pick -> pick -> grasp -> lift
-> approach place -> place -> release -> retreat
```

Each Cartesian point is solved by IK from the preceding pose. Each joint-space
phase is independently optimized and collision verified. Plan results display
waypoint count, servo adjustments, count travel, TCP length, minimum clearance
and total cost.

## 10. Rapier physics

Rapier contains fixed board/fixture colliders, position-based kinematic arm
capsules and four dynamic cube rigid bodies with friction and low restitution.
It runs at a fixed 1/60 s step. During virtual grasp the selected cube becomes
kinematic and follows the modeled tool attachment point; release restores a
dynamic body so gravity and contact settle it.

Physics validates contact behavior and object motion. It does not choose the
route. The optimizing planner remains responsible for collision-free path
selection, which avoids conflating dynamics with search.

## 11. Operational API boundary

The adapter matches the existing FastAPI routes and its same-origin rule.
`server.mjs` serves the built application and proxies a deliberately small
allowlist. Without `--allow-motion`, only `GET /api/state` and emergency
`POST /api/stop` pass. With the flag, power-on, power-off and single-joint move
are additionally available. Starting the server or opening the page performs
no controller request.

The app rejects HTML fallbacks, malformed state, missing channels and
non-integer commands. After explicit connection it polls state every 1.5 s.
Targets must fit the backend's current per-joint session bounds, every move must
change exactly one servo, the starting commanded pose must match the plan and
the backend must confirm each target.

Operational execution waits for idle after every request. A move failure
attempts `/api/stop` and puts the client in `fault-stopped`; an unconfirmed
browser stop still requires the physical servo power switch.

## 12. Validation

`npm test` runs 27 tests covering:

- schema units, fixture height, provenance and physical HOME S6;
- linear/anchored count conversion, deterministic FK and reachable IK;
- collision-valid HOME and supported-contact PARK;
- verified HOME-to-PARK planning;
- single-servo playback conversion;
- text task parsing;
- strict robot-state parsing and explicit-PARK operational guards;
- gateway read-only/motion allowlists;
- backend bounds, single-servo paths, command confirmation and stop-on-failure;
- schema failure cases, deterministic planning and motion-cost ordering; and
- Rapier world/collider initialization.

Manual browser validation covers task planning/playback, cube grasp/drop,
HOME, PARK, API refusal on a non-backend origin, console output and mobile
layout. `npm run build` succeeds. Vite warns that Three.js and especially the
separate embedded Rapier WebAssembly chunk exceed 500 kB; this is non-fatal.

## 13. Remaining validation work

Before unattended operational use, the team still needs surveyed collision
envelopes, validated angular zeros/directions, current-position sensing or an
equivalent homing strategy, real safe limits, continuous swept-volume testing,
orientation-aware IK, physically measured mass/friction parameters and
hardware-supervised trajectory trials. None of those are implied by this
simulation baseline.

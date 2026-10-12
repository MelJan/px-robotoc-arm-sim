# Three.js implementation

Status: initial offline implementation, 2026-10-12.

The implementation is under `web/`. It is a browser-based visual and kinematic
twin of the PX robot arm. It is independent of Blender, MuJoCo and Unreal, so
the team can compare the four simulator approaches without coupling their
runtimes.

## 1. Scope

The current version provides:

- simplified geometry for the mounting board, base, links, servos and gripper;
- a parented S1-S6 kinematic chain;
- servo-count controls and provisional count-to-angle conversion;
- HOME, PARK and straight-up reference poses;
- animated pose interpolation;
- an orbitable WebGL view with lights, shadows and axis helpers;
- world-space tool-centre-point and board-clearance readouts; and
- a responsive control panel suitable for a laptop or tablet.

It does not yet provide inverse kinematics, collision detection, dynamics,
torque simulation, URDF import, a Blender model import or physical-robot
connectivity.

## 2. Runtime architecture

The application uses Vite for development and production bundling and Three.js
for rendering. There is no frontend framework.

```text
index.html
    |
    v
src/main.js --------------------> src/style.css
    |
    +---- fetch('/arm-spec.json')
    |
    +---- build HTML controls
    +---- build Three.js scene graph
    +---- map servo counts to rotations
    +---- render continuously with requestAnimationFrame
```

`src/main.js` uses top-level `await` to load `public/arm-spec.json` before it
creates the controls or arm. A missing or invalid specification therefore
stops startup instead of silently rendering with arbitrary values.

## 3. Units and coordinate system

All geometry in `arm-spec.json` is expressed in metres. Displayed TCP and
clearance values are multiplied by 100 and shown in centimetres.

The world convention matches `docs/arm-dimensions.md`:

- origin: board top directly below the S1 rotation axis;
- +Z: upward from the board;
- +Y: from the robot toward the front of the board; and
- +X: across the 30 cm width of the board.

The Three.js camera normally treats +Y as up. The implementation changes
`camera.up` to `(0, 0, 1)` so the viewport follows the robot's Z-up convention.

## 4. Scene construction

The scene uses generated Three.js primitives rather than imported mesh files.
This keeps the initial twin reproducible and makes all kinematic dimensions
easy to audit.

The helper functions are:

- `material()` for consistent standard materials;
- `box()` for boards, brackets, links and gripper parts;
- `cylinder()` for simplified servo and joint housings;
- `link()` for a beam with two visible side rails; and
- `jointMarker()` for orange joint-centre markers.

Meshes cast and receive shadows. A hemisphere light supplies ambient fill,
while two directional lights establish shape and depth. `OrbitControls` adds
camera rotation, zoom and damping.

The geometry is intentionally schematic. Its purpose is to validate joint
origins, link lengths and motion before cosmetic CAD detail is introduced.

## 5. Kinematic hierarchy

Each moving joint is a `THREE.Group`. Child joints are positioned at the end of
their parent link, so changing a parent rotation automatically moves every
downstream joint.

```text
Scene
├── Board
├── Board_grid
├── World_axes
└── ARM_Root
    ├── Base_plate
    └── S1_joint                       rotate about Z
        ├── S1_servo / Pedestal
        └── S2_joint                   rotate about -X
            ├── Upper_arm
            └── S3_joint               rotate about -X
                ├── Forearm
                └── S4_joint           rotate about -X
                    ├── Wrist
                    └── S5_joint       rotate about Z
                        └── S6_gripper
                            ├── Palm and fingers
                            └── TCP
```

The local translations between joint groups are the measured link lengths:

| Translation | Specification value |
|-------------|---------------------|
| Board to S2 | `shoulderHeight` |
| S2 to S3 | `upperArm` |
| S3 to S4 | `forearm` |
| S4 to S5 | `wrist` |
| S5 to TCP | interpolated gripper length |

S6 is not modelled as another revolute joint. It changes finger spacing and
effective tool length based on its servo count.

## 6. Servo-count representation

`public/arm-spec.json` defines each joint with an ID, display name, slider
limits, HOME and PARK counts, provisional zero, ticks per degree, direction and
axis.

For S1-S5, `tickToDegrees()` applies:

```text
angle_degrees = direction * (tick - zero_tick) / ticks_per_degree
```

The angle is converted to radians immediately before it is assigned to a
Three.js rotation. S1 and S5 update local Z rotation. S2-S4 update local X
rotation with the sign required by the documented `-X` convention.

The values are provisional because the physical zero and direction of every
servo have not all been measured. Slider limits mirror the control interface's
reference range; they are not proof that every position is mechanically safe.

### S6 gripper mapping

The initial gripper model treats 490 ticks as fully open and 650 as closed:

```text
closure = clamp((tick - 490) / (650 - 490), 0, 1)
```

Linear interpolation then produces the pad gap and flange-to-tip length from
the measured open and closed dimensions. This is a visual approximation of the
real linkage, not a linkage-mechanism simulation.

## 7. Pose and animation flow

`currentTicks` is the application pose state. All UI changes follow the same
path:

```text
slider or preset
      |
      v
currentTicks
      |
      +--> update slider outputs
      +--> update S1-S5 rotations
      +--> update S6 geometry
      +--> update world matrices
      +--> update TCP and clearance
```

`animateTo()` interpolates each servo count from the current pose to a target
pose. It uses an ease-out cubic curve and rounds intermediate values to whole
servo counts. Animation is disabled automatically when the operating system's
`prefers-reduced-motion` setting is active.

HOME and PARK come from `arm-spec.json`. Straight-up uses each joint's
provisional zero, clamped to the configured slider range, and opens S6.

## 8. TCP and clearance calculation

`TCP` is an empty `THREE.Object3D` parented to the gripper. Its local Z position
tracks the interpolated gripper length. After joint rotations and gripper
geometry are updated, the application calls `scene.updateMatrixWorld(true)`
and obtains the TCP world position with `getWorldPosition()`.

Because the board top is world Z = 0, clearance currently equals the TCP world
Z value. A negative value is displayed as below the board. This is a point
clearance only; it does not check links or fingers for collision with the board.

## 9. Rendering and resizing

The renderer uses antialiasing, sRGB output and soft PCF shadows. Pixel ratio is
limited to 2 to avoid excessive GPU cost on high-density displays.

A `ResizeObserver` watches the viewport container. It updates the renderer size
and camera aspect ratio whenever the layout changes. The render loop updates
the damped orbit controls and draws the scene using `requestAnimationFrame()`.

## 10. Safety boundary

The application is simulation-only:

- no robot-arm API module is imported;
- no serial, GPIO, WebSocket or remote endpoint is opened;
- slider changes affect only local JavaScript objects; and
- the only fetch loads `/arm-spec.json` from the same web application.

Live control must not be added by reusing the slider handler directly. A future
live adapter should require an explicit mode switch, validate limits server
side, rate-limit commands, expose connection state and provide an emergency
stop. Simulation must remain the default mode.

## 11. Validation

Run the development version:

```sh
cd web
npm install
npm run dev
```

Then check:

1. The app loads without browser-console errors.
2. HOME and PARK set all six displayed counts to the specification values.
3. Straight-up aligns the pitch links vertically according to provisional zeros.
4. Each S1-S5 slider moves only its joint and downstream children.
5. S6 changes pad gap and TCP length without rotating the wrist.
6. The TCP coordinates update for every joint.
7. Joint axes can be hidden and the camera can be reset.
8. The layout remains usable at desktop and narrow viewport widths.

Verify the production bundle with:

```sh
cd web
npm run build
```

## 12. Recommended next steps

1. Generate `web/public/arm-spec.json` from `spec/arm.toml` so all simulators
   consume the same calibrated data.
2. Replace provisional zero ticks, directions and limits after physical
   calibration.
3. Add automated forward-kinematics test vectors shared with the Python and
   Blender implementations.
4. Add swept-volume or mesh collision checks for the board and robot links.
5. Add inverse kinematics only after the forward model agrees with measured
   poses.
6. Optionally import a lightweight GLTF cosmetic model while retaining the
   generated joint groups as the authoritative kinematic hierarchy.

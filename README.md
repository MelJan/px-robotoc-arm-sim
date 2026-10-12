# PX robot arm simulator — Three.js branch

This is the `feat/threejs-vk` browser implementation of the PX robot-arm
simulator. It is independent of the team's Blender, MuJoCo and Unreal work.

The application now supports full virtual simulation and planning:

- a detailed photo-informed Three.js arm, calibration mat, desk fixture,
  cables, servos, horns and linkage claw;
- a versioned millimetre `arm-spec.json` with explicit provenance;
- modular forward/inverse kinematics;
- capsule/box self, board, fixture and workpiece collision detection;
- deterministic optimizing joint-space planning and smoothing;
- text pick/place tasks with approach/lift waypoints and collision verification;
- coordinated and one-servo-at-a-time playback;
- lazy-loaded Rapier3D contact physics and dynamic solid cubes; and
- simulation plus explicitly guarded read-only/disarmed/armed operational
  modes for the existing robot-arm API.

Simulation is the default and startup performs no physical-arm request. The
hardware has no position feedback, so operational state is labelled commanded
pose. Historical bounds are not described as safe limits, and PARK remains an
observed supported commissioning reference rather than a motion-validated
pose.

## Run and verify

```sh
cd web
npm install
npm test
npm run dev
```

Open the URL printed by Vite. Build the production site with:

```sh
npm run build
```

For a same-origin, read-only robot connection, build first and use the guarded
gateway. Motion forwarding needs the separate explicit flag:

```sh
npm run serve -- --controller http://ROBOT_HOST:8765
npm run serve -- --controller http://ROBOT_HOST:8765 --allow-motion
```

The build succeeds with a non-fatal large-chunk warning for Three.js and the
separate embedded Rapier WebAssembly runtime.

## Key files

| Path | Purpose |
| --- | --- |
| `web/js/app.js` | active preserved UI, camera/model orchestration and integration |
| `web/js/model/` | detailed photo-informed robot, fixture, mat, cables and gripper |
| `web/js/advanced.js` | canonical-mm adapter for the preserved visual calibration |
| `web/src/core/` | pure kinematics, collision, planning, playback and physics adapters |
| `web/src/operational/client.js` | guarded same-origin robot API state machine |
| `web/src/operational/gateway.js` | allowlist policy for proxied robot routes |
| `web/server.mjs` | production static server and read-only-by-default API gateway |
| `web/public/arm-spec.json` | millimetre model, planner settings and provenance |
| `web/test/` | 27 model, planner, physics, connection and safety tests |
| `web/README.md` | usage and operational-mode guide |
| `docs/threejs-implementation.md` | detailed design and validation status |

The root repository may also contain separate Blender work in other checkouts.
Do not combine it with this branch without an explicit specification-integration
decision.

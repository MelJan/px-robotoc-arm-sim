# px-robotoc-arm-sim

MuJoCo sandbox for the six-servo desktop arm, driven through the same control API as the
real arm. The physics sits behind robot-arm's `ServoDriver` port, so robot-arm's `Session`,
its HTTP API, its web page and its demo playback run unchanged against the simulation.

Design: [docs/simulation-concept.md](docs/simulation-concept.md).
Measurements: [docs/arm-dimensions.md](docs/arm-dimensions.md), [docs/servo-calibration.md](docs/servo-calibration.md).

## Setup

Needs Python 3.12+, [uv](https://docs.astral.sh/uv/) and a checkout of the robot-arm repo next
to this one (or `ROBOT_ARM_PATH` pointing at it). Tested against robot-arm commit `3792d64`.

```sh
uv venv --python 3.13 .venv
uv pip install -p .venv/bin/python -e ".[dev]"
.venv/bin/python -m pytest          # about two minutes; the HTTP tests run a real server
```

## Run

```sh
.venv/bin/python -m armsim.server                 # sandbox API on http://127.0.0.1:8765
.venv/bin/mjpython -m armsim.server --viewer      # same, with the MuJoCo viewer window (macOS needs mjpython)
.venv/bin/python -m armsim.cli                    # text interface against the sandbox
.venv/bin/python -m armsim.cli --target http://raspberrypi.local:8765   # the same shell against the real arm
.venv/bin/python -m armsim.cli scripts/pick_red.txt                     # scripted session
```

With the server running, robot-arm's own page at http://127.0.0.1:8765/ and its 3D view at
`/simulation` show and drive the sandbox, Swagger is at `/docs`, and the rendered camera is
served at `/api/camera/snapshot.jpg` like on the Pi. Sandbox-only routes: `/sim/blocks`,
`/sim/state`, `/sim/reset`, `/sim/snapshot.png?camera=oak|top|side|front`, and
`POST /api/move_pose` (six counts plus an optional speed), the action proposed to robot-arm.

Text interface commands: `help` lists them. The important ones are `power on`, `move S2 450`,
`pose <six counts>`, `home`, `pick red`, `place T2`, `run red T1`, `demo import FILE`,
`demo load NAME`, `demo run`, `snapshot oak`, `log FILE`, `replay FILE`.

## Layout

| Path | Content |
|------|---------|
| `config/robot.yaml` | Link lengths, gripper linkage, servo model, count-to-angle calibration |
| `config/scene.yaml` | Table frame, base position, blocks, slots, physics, cameras |
| `armsim/model_builder.py` | MuJoCo model generated from the two configs |
| `armsim/driver.py` | `MujocoDriver`, robot-arm's `ServoDriver` port over the physics |
| `armsim/session.py` | robot-arm `Session` plus `move_pose` |
| `armsim/server.py` | Sandbox launcher: Session, HTTP server, camera service, `/sim` routes |
| `armsim/client.py` | `HttpArmClient` and `SessionArmClient`, one interface for sandbox and real arm |
| `armsim/kinematics.py`, `armsim/tasks.py` | Closed-form IK and the pick-and-place sequence in counts |
| `armsim/replay.py` | Import of Pi demo records, pose-log replay |
| `armsim/cameras.py` | Named cameras, fit of the real camera from robot-arm's calibration, `CameraSource` |
| `armsim/cli.py` | Text interface |
| `tests/` | Calibration, model geometry, driver contract, kinematics, grasp physics, task, replay, cameras, HTTP end to end |

## Assumptions still to verify on the real arm

All in `config/robot.yaml` and `config/scene.yaml`, each a single number:

- Joint angles at the HOME counts and the direction of rising counts per servo (taken from
  robot-arm's browser model; S4 assumed 95° so that PARK rests on the base plate).
- The gripper jaws open across the arm plane at S5 = 480, and 650 counts is fully open.
- The S1 axis stands 40 mm inside the near table edge, centred.
- Block mass 12 g, pad friction 1.0.

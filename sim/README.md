# MuJoCo model and IK

Engine-independent model of the arm, as suggested in `docs/simulator-proposal.md`.
The same `arm.xml` can later be imported into Unreal through URLab.

| File | Purpose |
|------|---------|
| `arm.xml` | MJCF model: board, test cube, S1 to S5 as hinges, the gripper as two coupled slides. Values from `docs/arm-dimensions.md`, in metres |
| `ik.py` | Top-down grasp IK from `docs/pick-and-place-plan.md`, section 3, in cm and degrees |
| `check_ik.py` | Sets the IK angles in MuJoCo and checks position, gripper direction and jaw alignment |
| `pick_place.py` | Runs the full pick-and-place sequence with physics. `--view` opens the MuJoCo viewer |

## Setup (Windows)

```powershell
py -3.13 -m venv .venv
.\.venv\Scripts\python.exe -m pip install mujoco numpy
```

## Run

```powershell
.\.venv\Scripts\python.exe sim\ik.py
.\.venv\Scripts\python.exe sim\check_ik.py
.\.venv\Scripts\python.exe sim\pick_place.py --view
```

`check_ik.py` and `pick_place.py` exit with code 1 if a check fails.

## What is placeholder

Only the link lengths, the gripper gap and the cube size are measured.
Masses, link radii, joint ranges, damping, actuator gains and friction are guesses.
The fingers are slides; the real gripper is a parallel linkage whose fingers get longer as they close.
The model's zero pose is straight up, not HOME; mapping model angles to servo counts needs the joint angles at HOME (`docs/pick-and-place-plan.md`, section 7).

"""Runs the motion sequence of docs/pick-and-place-plan.md, section 4, in MuJoCo.

Moves the cube from START to TARGET with the position actuators, then reports where
the cube ended up. Pass --view to watch it in the MuJoCo viewer.

Run: .venv/Scripts/python sim/pick_place.py [--view]
"""
import math, os, sys
import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import ik

START = (5.0, 15.0)     # cube centre, cm, same as the cube in arm.xml
TARGET = (-8.0, 12.0)
HOVER = 5.0
OPEN, GRIP = 0.0, 0.011  # finger slide in m; cube contact is at 0,00925, a bit more squeezes it
MOVE_S, WAIT_S = 1.5, 0.5

model = mujoco.MjModel.from_xml_path(os.path.join(os.path.dirname(__file__), "arm.xml"))
data = mujoco.MjData(model)
ARM = ["s1", "s2", "s3", "s4", "s5"]
JOINTS = ["s1_base", "s2_shoulder", "s3_elbow", "s4_wrist_flex", "s5_wrist_rot"]


def pose(xy, hover):
    a = ik.solve(*xy, hover=hover)["mjcf"]
    return np.radians([a[j] for j in JOINTS])


HOME = np.zeros(5)  # the model's straight-up pose stands in for HOME until the home angles are measured
steps = [
    ("open gripper",      HOME,                 OPEN),
    ("above start",       pose(START, HOVER),   OPEN),
    ("descend",           pose(START, 0),       OPEN),
    ("grip",              pose(START, 0),       GRIP),
    ("lift",              pose(START, HOVER),   GRIP),
    ("above target",      pose(TARGET, HOVER),  GRIP),
    ("descend",           pose(TARGET, 0),      GRIP),
    ("release",           pose(TARGET, 0),      OPEN),
    ("lift",              pose(TARGET, HOVER),  OPEN),
    ("return",            HOME,                 OPEN),
]

for name, q in zip(JOINTS, HOME):
    data.qpos[model.joint(name).qposadr[0]] = q
mujoco.mj_forward(model, data)


def run(viewer=None):
    q_prev, g_prev = HOME.copy(), OPEN
    for label, q, g in steps:
        n_move, n_wait = int(MOVE_S / model.opt.timestep), int(WAIT_S / model.opt.timestep)
        for i in range(n_move + n_wait):
            s = min(1.0, i / n_move)
            s = s * s * (3 - 2 * s)                    # smooth start and stop
            data.ctrl[[model.actuator(a).id for a in ARM]] = q_prev + s * (q - q_prev)
            data.ctrl[model.actuator("s6").id] = g_prev + s * (g - g_prev)
            mujoco.mj_step(model, data)
            if viewer is not None:
                viewer.sync()
        q_prev, g_prev = q, g
        c = data.body("cube").xpos * 100
        print(f"{label:13s} cube at ({c[0]:6.2f}, {c[1]:6.2f}, {c[2]:5.2f}) cm")


if "--view" in sys.argv:
    import mujoco.viewer
    with mujoco.viewer.launch_passive(model, data) as v:
        run(v)
else:
    run()

c = data.body("cube").xpos * 100
err = math.hypot(c[0] - TARGET[0], c[1] - TARGET[1])
print(f"cube ended {err:.2f} cm from the target, height {c[2]:.2f} cm")
sys.exit(0 if err < 0.5 and abs(c[2] - ik.CUBE / 2) < 0.2 else 1)

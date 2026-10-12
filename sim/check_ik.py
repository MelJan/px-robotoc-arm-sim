"""Checks sim/ik.py against the MuJoCo model sim/arm.xml.

For each test position: solve the IK, set the joints in MuJoCo, and check that the
grasp site lands on the cube centre, the gripper points down and the jaws are
parallel to the board edges.

Run: .venv/Scripts/python sim/check_ik.py
"""
import math, os, sys
import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import ik

model = mujoco.MjModel.from_xml_path(os.path.join(os.path.dirname(__file__), "arm.xml"))
data = mujoco.MjData(model)
grasp = model.site("grasp").id

CASES = [(5, 15, 0), (5, 15, 5), (-8, 12, 0), (-8, 12, 5), (0, 10, 0), (12, 8, 5), (0, 20, 0), (0, 25, 0)]

ok = True
for x, y, hover in CASES:
    try:
        a = ik.solve(x, y, hover=hover)
    except ik.Unreachable as e:
        print(f"({x:4}, {y:4}) hover {hover}: unreachable, {e}")
        continue

    mujoco.mj_resetData(model, data)
    for name, deg in a["mjcf"].items():
        data.qpos[model.joint(name).qposadr[0]] = math.radians(deg)
    mujoco.mj_kinematics(model, data)

    pos = data.site_xpos[grasp] * 100                 # m -> cm
    R = data.site_xmat[grasp].reshape(3, 3)
    target = np.array([x, y, ik.CUBE / 2 + hover])
    err = np.linalg.norm(pos - target)
    down = R[:, 2] @ np.array([0, 0, -1])             # 1 = local Z points straight down
    jaw = R[:, 0]                                     # finger slide direction
    jaw_skew = math.degrees(math.atan2(abs(jaw[1]), abs(jaw[0])))
    jaw_skew = min(jaw_skew, 90 - jaw_skew)           # 0 = parallel to a board edge

    good = err < 1e-3 and down > 1 - 1e-6 and jaw_skew < 1e-3
    ok &= good
    j = "  ".join(f"{k.split('_')[0]}={v:7.2f}" for k, v in a["mjcf"].items())
    print(f"({x:4}, {y:4}) hover {hover}: err {err:.1e} cm, down {down:.6f}, jaw skew {jaw_skew:.1e}°  "
          f"{'OK' if good else 'FAIL'}   {j}")

print("all reachable cases OK" if ok else "FAILURES")
sys.exit(0 if ok else 1)

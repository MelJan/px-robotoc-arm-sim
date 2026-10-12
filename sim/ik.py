"""Top-down grasp inverse kinematics from docs/pick-and-place-plan.md, section 3.

Lengths in cm, angles in degrees. Board frame as in docs/arm-dimensions.md.
"""
import math

# ---------------- measured values (cm), docs/arm-dimensions.md ----------------
H_S2 = 9.6          # board top to S2 axis
L2 = 10.5           # S2 shoulder to S3 elbow
L3 = 12.6           # S3 elbow to S4 wrist flexion
L4 = 6.7            # S4 wrist flexion to gripper flange
G_BLOCK = 11.0      # flange to cube centre while holding the cube
TOOL = L4 + G_BLOCK # S4 to the held cube centre: 17,7
CUBE = 2.65


class Unreachable(ValueError):
    pass


def solve(x, y, z_c=CUBE / 2, hover=0.0):
    """Joint angles for the cube centre at (x, y, z_c + hover), gripper pointing down.

    Returns a dict with the plan's angles (theta1, phi2, q3, phi3, q4, q5) and
    `mjcf`, the joint values for sim/arm.xml in degrees.
    """
    theta1 = math.degrees(math.atan2(x, y))
    r = math.hypot(x, y)

    dz = z_c + hover + TOOL - H_S2
    d = math.hypot(r, dz)
    if not abs(L2 - L3) <= d <= L2 + L3:
        raise Unreachable(f"wrist point {d:.2f} cm from S2, reach is {abs(L2 - L3):.1f} to {L2 + L3:.1f}")

    c3 = (d * d - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    q3 = -math.acos(max(-1.0, min(1.0, c3)))          # elbow-up solution
    phi2 = math.atan2(dz, r) - math.atan2(L3 * math.sin(q3), L2 + L3 * math.cos(q3))
    phi3 = phi2 + q3
    q4 = -math.pi / 2 - phi3

    q5 = (-theta1 + 45.0) % 90.0 - 45.0               # cube looks the same every 90°

    deg = math.degrees
    a = dict(theta1=theta1, r=r, dz=dz, phi2=deg(phi2), q3=deg(q3), phi3=deg(phi3), q4=deg(q4), q5=q5)
    # sim/arm.xml measures S2 from vertical and tilts toward +Y for positive values.
    a["mjcf"] = dict(s1_base=theta1, s2_shoulder=90.0 - a["phi2"], s3_elbow=-a["q3"],
                     s4_wrist_flex=-a["q4"], s5_wrist_rot=q5)
    return a


def max_reach(z_c=CUBE / 2, hover=0.0):
    """Largest horizontal distance from S1 for a top-down grasp at this height."""
    dz = z_c + hover + TOOL - H_S2
    return math.sqrt((L2 + L3) ** 2 - dz * dz)


if __name__ == "__main__":
    a = solve(5, 15)
    for k in ("theta1", "r", "dz", "phi2", "q3", "phi3", "q4", "q5"):
        print(f"{k:7s} {a[k]:8.2f}")
    print(f"max reach: grasp {max_reach():.2f} cm, hover 5 {max_reach(hover=5):.2f} cm")

"""Synthetic check of vision/blocks.py with stacked cubes.

Renders the ArUco mat and a few cubes in MuJoCo, through the camera that blocks.py
fits to a real controller frame, then runs blocks.py on the render and compares the
found positions and stack levels with the truth.

Run: .venv/Scripts/python vision/test_synthetic.py [--out render.png]
"""
import argparse, math, os, sys
import cv2
import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import blocks

REAL_FRAME = os.path.join(blocks.REPO, "docs", "images", "photos", "controller-camera-2026-10-12.jpg")
CELL_MM = 5.0   # 30 mm marker = 6 x 6 cells including the black border

# (colour, rgba, x_mm, y_mm, level)
CUBES = [
    ("orange", "1 0.42 0.12 1", 150.0, -60.0, 0),
    ("blue",   "0.1 0.3 0.9 1", 150.0, -60.0, 1),
    ("green",  "0.1 0.65 0.2 1", 230.0, 40.0, 0),
    ("yellow", "0.95 0.8 0.1 1", 150.0, -60.0, 2),
]


def marker_geoms(layout):
    """Black 5 mm tiles reproducing each printed marker, in table mm."""
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    out = []
    for mid, corners in layout.items():
        bits = cv2.aruco.generateImageMarker(d, mid, 6, borderBits=1)
        tl, tr, _, bl = corners
        right, down = (tr - tl) / 6, (bl - tl) / 6       # one cell along the marker's image axes
        for r in range(6):
            for c in range(6):
                if bits[r, c] == 0:
                    p = tl + right * (c + 0.5) + down * (r + 0.5)
                    out.append(f'<geom type="box" size="{CELL_MM / 2000:.5f} {CELL_MM / 2000:.5f} 0.0001" '
                               f'pos="{p[0] / 1000:.5f} {p[1] / 1000:.5f} 0.0001" rgba="0 0 0 1"/>')
    return out


def scene_xml(layout, cam):
    s = blocks.CUBE_MM / 2000
    cubes = [f'<geom type="box" size="{s} {s} {s}" pos="{x / 1000} {y / 1000} {(k * blocks.CUBE_MM) / 1000 + s}" rgba="{rgba}"/>'
             for _, rgba, x, y, k in CUBES]
    R, C = cam["R"], cam["centre_mm"] / 1000
    xaxis, yaxis = R[0], -R[1]                       # OpenCV x right, y down -> MuJoCo x right, y up
    fovy = math.degrees(2 * math.atan(blocks.H_IMG / 2 / cam["K"][1, 1]))
    return f"""
<mujoco>
  <visual><global offwidth="{blocks.W}" offheight="{blocks.H_IMG}"/><headlight ambient="0.6 0.6 0.6" diffuse="0.4 0.4 0.4"/></visual>
  <worldbody>
    <geom type="box" size="0.18 0.14 0.001" pos="0.18 0 -0.001" rgba="0.95 0.95 0.95 1"/>
    {''.join(marker_geoms(layout))}
    {''.join(cubes)}
    <camera name="oak" pos="{C[0]} {C[1]} {C[2]}" fovy="{fovy}"
            xyaxes="{xaxis[0]} {xaxis[1]} {xaxis[2]} {yaxis[0]} {yaxis[1]} {yaxis[2]}"/>
  </worldbody>
</mujoco>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", help="write the annotated render here")
    args = ap.parse_args()

    layout = blocks.load_layout(blocks.DEFAULT_CAL)
    real = cv2.imread(REAL_FRAME)
    cam_real = blocks.fit_camera(blocks.detect_markers(real, layout), layout)

    model = mujoco.MjModel.from_xml_string(scene_xml(layout, cam_real))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with mujoco.Renderer(model, blocks.H_IMG, blocks.W) as r:
        r.update_scene(data, camera="oak")
        img = cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR)

    cam, found, report = blocks.analyse(img, layout)
    print(f"camera refit on render: {cam['rms_px']:.2f} px rms, f {cam['K'][0, 0]:.0f}, markers {cam['markers']}")
    ok = True
    for colour, _, x, y, k in CUBES:
        hits = [c for c in found if c["colour"] == colour]
        if len(hits) != 1:
            print(f"{colour:7s} FAIL: found {len(hits)}")
            ok = False
            continue
        c = hits[0]
        err = math.hypot(c["x_mm"] - x, c["y_mm"] - y)
        good = err < 3.0 and c["level"] == k and c["level_from_size"] in (None, k) and c["valid"]
        ok &= good
        print(f"{colour:7s} truth ({x:6.1f}, {y:6.1f}) L{k}   found ({c['x_mm']:6.1f}, {c['y_mm']:6.1f}) "
              f"L{c['level']} size-L{c['level_from_size']} on={c['on']}  err {err:.1f} mm  "
              f"edge {c['front_edge_mm']:.1f} mm  {'OK' if good else 'FAIL'}")
    if args.out:
        cv2.imwrite(args.out, blocks.annotate(img, cam, found))
    print("all OK" if ok else "FAILURES")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

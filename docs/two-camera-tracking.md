# Two cameras: tracking the arm and locating cubes in 3D

Since 2026-10-13 two cameras watch the mat (`images/photos/setup-two-cameras-*.jpg`):

| Camera | Where | API (controller, GET only) |
|--------|-------|----------------------------|
| OAK | on an articulated stand at the desk edge | `/api/camera/status`, `snapshot.jpg`, `stream.mjpg`, `capture` |
| Webcam | clipped to the top of the monitor beside the desk | `/api/camera2/status`, `snapshot.jpg`, `stream.mjpg`, `capture` |

The second camera's routes were read from the controller's `/openapi.json` on 2026-10-13. They are not yet in `lutich/robot-arm` on GitHub. `server.mjs --camera http://10.102.4.103:8765` forwards both, read only. Without it, the OAK falls back to the saved frame; for the webcam, load an image file.

AprilTags (family 36h11) on the arm, in two rigid parts (`images/photos/arm-tags-*.jpg`, IDs read with OpenCV). Tags on one part move together and act as one object. One camera may see one tag of a part and the other camera another, and the views still meet in one pose. Each part has one degree of freedom relative to the part before it, so few views constrain it.

| Part | Moved by | Tag | Where |
|------|----------|-----|-------|
| Wrist (S4 bracket + S5 servo) | S1–S4, turns about S4 | 103 | S5 servo face, facing along the S4 axis |
| | | (reprint) | S4 bracket side bar, same direction |
| Gripper (S6 servo + jaws) | S1–S5, turns about S5 | 100 | S6 servo, face toward the wrist |
| | | (reprint) | S6 servo, cable end |

The first prints of the two "reprint" tags have no white margin on the black servo, and one is bent over an edge, so they cannot be detected. Tags need a white margin of at least 1/8 of the black square's side and must sit flat on one face. Tags 72 and 71 from the first photos were replaced by 103 and 100.

The photos also show that the S5 servo's wide face (with tag 103) faces along the S4 axis, so the model now has the S5 servo turned 90° about the tool axis.

**Overlay.** "Tags & parts" in the 3D view, and the same checkbox in the Camera tab, colours each tag by its part, joins the tags of one part and links them to the joint axis that moves the part. The Arm tracking box lists every tag with its part, the joints that move it and the cameras that saw it.

**How big must the tags be** (live test, 2026-10-13: a strip of five tags 105–109, 18,9 mm black square with a white margin, held about 18 cm above the mat):
- Both cameras found all five, OpenCV and the browser detector alike. They appeared 23 to 39 px wide in the 1280 × 720 frames.
- Triangulated from both cameras, the spacing came out as 35,4 / 35,0 / 35,0 / 34,6 mm and the side as 18,9 ± 0,3 mm.
- The same frames found tag 103 on the arm only in the webcam, and tag 100 in neither (smaller in the image, at a steep angle).
- So: about 20 px across in the image is the lower limit. 19 mm works for tags facing the cameras; 25 to 30 mm (with a 4 mm margin) is safer on the servo faces, which leave room for that.

## How it works

All of it runs in the browser (Camera tab), in plain JS. The mat markers are ArUco 4×4; the arm tags are AprilTag 36h11 (`js/apriltag36h11.js`, all 587 codes). Both are read by the same detector, `js/aruco.js`.

1. **Calibrate each camera** from the mat markers in its own frame, as before ("Calibrate all cameras").
   - Each camera keeps its own fit, stored per browser.
   - The 3D view shows both cameras and their fields of view, and the "Webcam" view button looks through the second one.
2. **Arm tracking** ("Track the arm", `js/armtrack.js`):
   - The tags are detected in every camera's frame.
   - A tag seen by both cameras is **triangulated**: the viewing rays from both cameras are intersected (linear least squares). This is its 3D position, independent of the arm model.
   - The joint angles are then fitted so that the model's tags project onto all the tag views (Levenberg-Marquardt). S5 is fitted only when tag 71 (on the hand) is seen.
   - The tracked pose is shown as a ghost arm. The table "tracked − commanded" is what the HOME calibration still gets wrong (`HANDOVER.md`, open item 1).
3. **Cubes in 3D** ("Find cubes", `js/locate3d.js`):
   - Cubes are found by colour in every camera's frame.
   - For each colour the blobs are paired across the cameras, and their viewing rays are intersected.
   - Each ray passes through the centre of the faces that camera sees, not through the cube's centre, so every ray is moved back by that offset before intersecting. A cube under another shows no top face.
   - The result is the cube's centre with its real height: the stack level, or "between levels" for a cube in the gripper or tilted. The gap between the rays shows how well they met.
   - **Cubes are no longer assumed to lie on the mat.** With only one camera, the old mat-plane estimate is used.
   - "Put them into the sim" places the cubes at their stack levels in the shared simulation.

## Checks

| Check | Result |
|-------|--------|
| Unit tests (`test/armtrack.test.mjs`, `test/locate3d.test.mjs`) | Triangulation exact without noise, 3 mm with 0,5 px noise. Joint angles recovered to 0,2° from the two tags. Cubes on the mat, stacked (levels 1, 2) and in the air located to under 4 mm |
| Detector on the photos | Tag 72 found in all three close-ups, and the printed sheets' tags in the overviews. Tag 71 needs a small threshold window (it sits on the black servo with a thin white margin); the tracker tries four settings |
| In the page, frames rendered by the simulator from two cameras (headless Chrome) | Both cameras recovered from the mat markers to 4 to 7 mm. Tag 72 triangulated within 1 mm of the model. Joint angles within 1,3° of the commanded pose. Stacked blue-on-orange found at levels 0 and 1, a single cube to 1 mm in height |
| Known limit | A cube half hidden in one view (behind the gripper) is placed wrong; its rays then miss by several mm and it is reported as "between levels", not placed |

Real frames from both cameras have not been run yet. They need the server started with `--camera http://10.102.4.103:8765` (one snapshot per camera on request, or live video).

## Next

- Measure the tag positions and sizes on the arm. Add a third tag on the upper arm or forearm for a better-determined pose.
- Collect tracked − commanded over a few poses and fit the HOME angles and directions from them (Calibration panel).
- Add the second camera to the simulator's own robot API (`/api/camera2/*` on the simulated backend: rendered frames), so other teams can test against it.

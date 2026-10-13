# Two cameras: tracking the arm and locating cubes in 3D

Since 2026-10-13 two cameras watch the mat (`images/photos/setup-two-cameras-*.jpg`):

| Camera | Where | API (controller, GET only) |
|--------|-------|----------------------------|
| OAK | on an articulated stand at the desk edge | `/api/camera/status`, `snapshot.jpg`, `stream.mjpg`, `capture` |
| Webcam | clipped to the top of the monitor beside the desk | `/api/camera2/status`, `snapshot.jpg`, `stream.mjpg`, `capture` |

The second camera's routes were read from the controller's `/openapi.json` on 2026-10-13. They are not yet in `lutich/robot-arm` on GitHub. `server.mjs --camera http://10.102.4.103:8765` forwards both, read only. Without it, the OAK falls back to the saved frame; for the webcam, load an image file.

Two AprilTags (family 36h11) are stuck to the arm (`images/photos/arm-tags-71-72-*.jpg`, found with OpenCV):

| Tag | Where | Model (`js/config.js` `ARM_TAGS`) |
|-----|-------|-----------------------------------|
| 72 | label face of the S5 (wrist rotation) servo | wrist frame, centre (0,7, −1,0, 4,5) cm, facing −Y |
| 71 | face of the S6 (gripper) servo toward the wrist | hand frame, centre (1,9, −2,5, 0,38) cm, facing −Z |

The sizes (2,2 cm black square) and positions come from the photos only, so they are good to about 2 to 3 mm. **Measure them on the arm** and correct `ARM_TAGS`.

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

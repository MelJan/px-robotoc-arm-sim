# Changelog

Branch `feature/threejs-sim`. Newest first. Dates are when the change was committed.

## 2026-10-13: One robot API for the simulated and the real arm

**For other teams: you can now develop against the simulated arm with exactly the API of the real controller, and switch to the real arm without changing your code.**

### Start it

```powershell
git checkout feature/threejs-sim
cd threejs-sim
node server.mjs                     # Node 24, no npm install needed; http://127.0.0.1:8770/
node server.mjs --host 0.0.0.0      # reachable from other machines on the network
```

Open http://127.0.0.1:8770/ to watch the simulated arm. Every program that talks to the server sees the same arm and the same cubes.

### Use it

The API is the controller's low-level API (`lutich/robot-arm` `docs/api.md`): `GET /api/state`, `POST /api/move`, `/api/jog`, `/api/go_pose`, `/api/stop`, `/api/power_on`, `/api/settings`, `/api/limits`, `/api/demo/*`.

```python
import json, urllib.request
BASE = "http://127.0.0.1:8770"

def call(path, body=None):
    req = urllib.request.Request(f"{BASE}/api/{path}", method="POST" if body is not None else "GET",
        data=None if body is None else json.dumps(body).encode(),
        headers={"Content-Type": "application/json", "Origin": BASE})
    return json.load(urllib.request.urlopen(req))

call("move", {"channel": 0, "target": 400})     # S1 to 400 counts
print(call("state")["commands"])
```

- POSTs need `Content-Type: application/json` and `Origin: <the server's address>`, the same rules as the real controller.
- Targets outside the limits are rejected (HTTP 400), and only one motion runs at a time.
- Full reference: [`docs/arm-api.md`](docs/arm-api.md).

### Simulation or real arm

- `GET /api/backend` tells which arm answers. `POST /api/backend {"target": "sim" | "real"}` switches it for everybody; the page has the same switch in its header.
- The server always starts on the simulation. The real arm is only available when the server runs with `--controller http://10.102.4.103:8765`, and it only moves with `--allow-motion` on top.
- In real mode, every call that can move the arm is first played in the simulation. If the arm would hit itself or the board, the call is refused with **HTTP 409** and the reason. `stop` is never blocked.
- In the simulation, a collision stops the arm like an emergency stop; `POST /api/power_on` re-arms it.
- The simulation's cubes: `GET /sim/world`, and `POST /sim/scene {"cubes": [["orange", 0, 14, 0, 0], ...]}` with [colour, x, y, level, yaw] in the board frame (cm). This is not part of the robot API.

### Also new

- The page is now a client of this API.
  - Joints, HOME/PARK, planned tasks and scenarios all send API calls to the active arm.
  - The 3D view mirrors the server's arm and cubes.
  - A red banner shows when the real arm is active.
- **Scenarios:** predefined block layouts with the steps to execute, or the cubes the camera sees plus typed tasks.
  - The planner turns each task into a trajectory; stored trajectories (the team's taught pick and place) are possible too.
  - ↺ Reset puts the cubes in place; ▶ Run executes the steps on the active arm.
- **Model vs. photo check:** the camera pose and joint angles are fitted to points marked in the reference photos (`tools/fit-views.mjs`, `tools/compare.mjs`).
  - The PARK pose fitted from five photos puts the gripper pads on the base plate, as in reality.
  - Results and open model differences: [`docs/model-photo-comparison.md`](docs/model-photo-comparison.md).
- **Model:** the PCA9685 board and its pink foam sit on the upper arm's side plate, as on the real arm.
- **Tests:** 29 (`node --test threejs-sim/test/*.test.mjs`), including the API backend with a second backend playing the real controller.

### Known limits

- The angle at HOME of each servo is still not measured, so the model's PARK puts the pads 2 mm into the board. At PARK the real-mode check therefore accepts gripper contact with the base and the board.
- Calibration edits in the page are local to that browser; the server's simulation uses `threejs-sim/js/config.js`.
- One camera through the API so far (`/api/camera/*`).

### Upgrading

Restart a running `server.mjs` after pulling. `--mock` is no longer needed (it is still accepted): the simulated arm is always there.

## 2026-10-12

- three.js simulator in `threejs-sim/`: kinematics, controller emulation, collision, cube physics, planning, camera calibration from the mat markers, cube recognition, tower demo (`docs/threejs-sim.md`).
- The MuJoCo and Unreal routes were removed.
- Reference photos of the arm, and a first render-vs-photo comparison tool.

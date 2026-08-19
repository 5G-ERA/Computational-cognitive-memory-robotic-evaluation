# G1 twin — field notes, troubleshooting and scenarios

Hard-won operational knowledge about the simulator. For launching and rebuilding see
[`RUN_AND_REBUILD.md`](RUN_AND_REBUILD.md); this page is what you reach for when something
behaves oddly. (Translated from the original Spanish cheat sheet on 2026-08-07; paths updated
to the reorganised repository.)

---

## The robot model

`G1_SIM_MODEL` (default `full`):

- **`full`** — the complete G1 with a collision box matching the **real envelope**
  (0.44 m shoulder width × 1.10 m tall; the box stops below the lidar plane at z≈1.20 so the
  laser does not see itself).
- **`box`** — a simple 0.30 × 0.25 box (the old, permissive physics).

**Why this matters (03 Jul 2026):** with the old box the robot "crossed" the door while its
visual shoulders were *inside the wall* — a false success. Switching to the real envelope
exposed that the carved doorway was only 0.42 m wide; it was re-carved to the physical
dimensions (opening 0.85–1.2 m, narrowest corridor point 0.71 m) and validated with three
consecutive clean runs. **Never go back to the simple box to make results look better.**

---

## The golden rule: exactly one gzserver

Never launch a second `gazebo` / `ros2 launch` (from the VNC desktop or another exec) while one
is alive. The second gzserver dies with `exit code 255` (master port taken) and its spawn fails
with `Entity [g1] already exists`. **That pair of errors is the exact signature of "another
gzserver is alive".** Check and clean before relaunching:

```bash
docker exec g1sim bash -lc "ps aux | grep -E 'gzserver|gzclient' | grep -v grep"
docker exec g1sim kill -9 <PID>
```

Kill by PID: `pkill -f gzserver` can hang, because the pattern matches the killing process
itself.

---

## Seeing the world in 3D (the pattern that works)

```bash
# 1) clean up
docker exec g1sim bash -lc "pkill -f gzserver; pkill -f gzclient; sleep 2"
# 2) ONE headless simulation
docker exec -d g1sim bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/g1_ws/install/setup.bash && ros2 launch g1_sim lab.launch.py"
# 3) in the VNC desktop terminal, ONLY the client (it attaches to the live gzserver)
#    source /opt/ros/humble/setup.bash && source ~/g1_ws/install/setup.bash && gzclient
```

GUI notes:

- The robot may appear without meshes (the URDF uses `package://`, which gzclient does not
  always resolve) → **View → Collisions** shows it as orange boxes. The mesh fix is
  `export GAZEBO_MODEL_PATH=/home/ubuntu/g1_ws/install/g1_description/share:$GAZEBO_MODEL_PATH`
  (already in the container's `~/.bashrc`).
- `gui:=true` from a plain `docker exec` does **not** paint a window: gzclient hangs creating
  the GL context in software. X itself is fine (`xeyes` works).
- Lighter 3D alternative: `rviz2 --ros-args -p use_sim_time:=true`, Fixed Frame `odom`, plus
  LaserScan `/scan` and TF.
- To paste text into the VNC session use `http://localhost:6080/vnc.html` (the full client has
  a clipboard panel; the lite one does not) — better still, do not type in the VNC at all and
  drive everything through `docker exec`.
- **Data runs always go headless**: software rendering steals CPU from the physics.

---

## Common problems

| Symptom | Cause / fix |
|---|---|
| `command not found: python` | The virtualenv is not active |
| `SIN /odom` (no odometry) | The simulation from step 1 is not running |
| gzserver `exit code 255` + spawn `Entity [g1] already exists` | Another gzserver is alive (golden rule above): find the PID and `kill -9` before relaunching |
| The robot visually "goes through" walls | Visual body larger than the collision box; check View→Collisions. If it goes through *for real* (odometry crosses the wall), the container's URDF is stale: re-copy `sim/g1_nav.urdf` into both `src` and `install` of `g1_description` and relaunch |
| The robot presses against the door frame without advancing (ENG-GO, prog=0.00) | Legitimate physics with the 0.44 m envelope if it arrives off-centre. If it happens *every* time, measure the doorway in the world (was the re-carve applied?) |
| Connects but will not navigate: "RELOCALIZACION DUDOSA" | Stale adapter — `git pull` |
| Invisible window with `backend grafico: MacOSX` | macOS backend bug → install the Tk backend; the adapter already prefers TkAgg |
| `No module named '_tkinter'` | Install the Tk support package for your Python |
| RViz shows "no tf data" | Optional viewer; set Fixed Frame to `odom` by hand |

---

## Scenarios

### `lab` (default)
`lab.world`, **generated from the real laboratory map** (refmap walls at 2.2 m + `nav_map`
furniture at 0.8 m, same G1 frame). It uses the **real** A/B/C waypoints, the real reference
map and the real door with engagement active. The robot spawns at A, and simulated runs are
directly comparable with the robot's.

Installing updated world/launch/URDF files into a running container:

```bash
docker cp sim/g1_sim_pkg/worlds/lab.world      g1sim:/home/ubuntu/g1_ws/src/g1_sim/worlds/
docker cp sim/g1_sim_pkg/launch/lab.launch.py  g1sim:/home/ubuntu/g1_ws/src/g1_sim/launch/
docker cp sim/g1_sim_pkg/worlds/lab.world      g1sim:/home/ubuntu/g1_ws/install/g1_sim/share/g1_sim/worlds/
docker cp sim/g1_sim_pkg/launch/lab.launch.py  g1sim:/home/ubuntu/g1_ws/install/g1_sim/share/g1_sim/launch/
docker cp sim/g1_nav.urdf  g1sim:/home/ubuntu/g1_ws/src/g1_description/urdf/g1_nav.urdf
docker cp sim/g1_nav.urdf  g1sim:/home/ubuntu/g1_ws/install/g1_description/share/g1_description/urdf/g1_nav.urdf
```

**Copy into BOTH places** — forgetting `install/` leaves a ghost robot running the old URDF.
And copied files only affect the **next** simulation: a running gzserver keeps the world and
robot it started with, so restart the launch afterwards.

A preview of the generated world is in `sim/lab_world_preview.png`. Note that `waypoint` and
`sweep` commands in simulation write to `sim/nav_map_lab.json` (a copy), never to the real map.

### `room`
The synthetic 8×6 room with a pillar (v1), no door, engagement off — its own simulation
waypoints. Select it with `G1_SIM_SCENARIO=room`.

---

## What the adapter does for you (simulation defaults)

`G1_ENV=sim` + `G1_SIM_ID` for tagging · `G1_NOVIS=1` (no camera, unless the synthetic one is
enabled) · `G1_DOOR_ENGAGE=0` in the room scenario · `G1_RELOCGUARD=0` and `G1_NOGATE=1` (the
simulated pose is exact) · its own scenario files under `sim/`, so it **never touches the real
lab files** · and the same stick physics as the real robot (deadzone 0.3, calibrated strafe and
turn signs).

## Analysis after a run

```bash
python3 analysis/analyze_msm.py dataset/<run>.json    # meta states, transitions, events
python3 analysis/autopsy.py dataset/<run>.json        # full HTML report
python3 tools/summarize_runs.py                       # CSV across runs (sim rows carry env=sim)
```

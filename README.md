# G1 Unitree — Autonomous Navigation & Meta-Reasoning (consumer "Air", no EDU/DDS)

Autonomous exploration and obstacle avoidance on a **stock Unitree G1 "Air"** humanoid — a
consumer unit that exposes **only a single WebRTC session through the official app** (no ROS 2, no
DDS, no SSH, no low-level SDK). Everything here is built **on top of that one channel**, by tapping
the app's own WebView over USB.

> The robot maps and navigates while we read its LiDAR/odometry and drive it — all without EDU
> hardware — and learns from its own collisions. The longer-term goal is **meta-reasoning**: a robot
> that reasons about the reliability of its own perception and adapts.

---

## TL;DR — what this does

- **Reads the robot's SLAM live** (point cloud + odometry) from Python, over USB, by hooking the
  app's WebView with `ios-webkit-debug-proxy` (no second WebRTC session needed).
- **Drives the robot** (walk / turn / strafe) by **injecting `rt/wirelesscontroller` velocity
  commands into the app's existing WebRTC datachannel** — the only way to move it programmatically
  without a 2nd session.
- **Autonomous explore mode**: wanders a room, builds coverage, avoids obstacles fusing **LiDAR +
  camera (floor segmentation, edge, YOLOv8s, MiDaS depth) + odometry collision detection**, and
  **remembers obstacles it bumps into**.
- **Live visualizer** of the LiDAR cloud (2D/3D) + robot pose + path + camera with YOLO boxes.

This was reverse-engineered against the owner's **own** robot/app for interoperability. The APK,
keys and proprietary assets are **not** redistributed here.

---

## CURRENT PIPELINE (July 2026) — A↔B navigation + DCE governance

The project has moved from reactive exploration to a **validated A↔B door-crossing pipeline**
used as the real-robot testbed for the DCA/DCE evaluation (see the tutor's paper *Decentralised
MetaReasoning Through Capability Abstraction* and `docs/META2_GOVERNANCE.md`):

- **Global plan on the full static map** (`G1_GLOBALMAP=static`): walls = hard cost (uninflated,
  the real ~0.8 m door survives), known furniture from `nav_map.json` = soft cost + wall halo.
  Deterministic plan; the local DWA keeps using the live laser. Export/inspect the map with
  `g1_get_static_map.py` (local | webview | pcd).
- **Door ENGAGEMENT** (`G1_DOOR_ENGAGE=1`): pre-entry point on the door axis (from the static
  map), stop, rotate until |yaw−axis| ≤ 8°, cross straight with re-align on biped drift.
  Works both directions (A→B and B→A).
- **META2 governance** (`G1_META2=1` shadow / `=2` active): Meta-Reasoner 2.0 (the tutor's
  configuration-first DCE runtime, package `meta-reasoner-2.0/`) fed each tick with the robot's
  shared experience — safety←clearance, progression, **mobility** (resistance channel:
  achieved/commanded speed), reliability/uncertainty with a **few-shot historical margin**.
  Outputs KEEP/SWITCH/FALLBACK/HELP; active mode applies per-analogy speed ceilings and the
  **experience escalation** aborts the run when sustained experience says no analogy is valid.
- **Everything is measured**: per-tick dataset (`dataset/<run>.json`) with SEI metrics, META2
  decisions, collisions with pre-impact camera frames; `summarize_runs.py` → `runs_summary.csv`
  (one row per run: governance, env sim/real, times, collisions, META2 aggregates) — the raw
  results table for the paper. `autopsy.py` renders a full HTML report per run.
- **Reproducible experiments**: every change is env-revertible, validated by offline replay
  against all logged runs before touching the robot (`g1_replay.py`, `sim_globalplan.py`,
  bridge replay), and the frozen git branch **`baseline`** holds the validated navigation
  config without governance for fair comparison.

Quick start (on the Ubuntu machine driving the robot):

```bash
git pull                                   # ALWAYS first
# T1: perception server
G1_FLOORCOLOR=1 python perception_server.py --host 0.0.0.0 --port 8008 \
    --fx 300 --fy 300 --cx 160 --cy 120 --cam-h 1.10 --cam-pitch -10
# T2: navigation with governance active
G1_META2=2 G1_PERC=127.0.0.1:8008 python g1_goto.py gotoviz B
```

Key env flags: `G1_META2` (0/1/2) · `G1_M2_ABORT` (experience escalation) · `G1_M2_HIST_K`
(few-shot margin; 0 = one-shot ablation) · `G1_GLOBALMAP` (static/hard/ref/live) ·
`G1_DOOR_ENGAGE` · `G1_ENV=real|sim` + `G1_SIM_ID` (simulation campaign tagging) ·
`G1_AGGR_R`, `G1_HARDGUARD`, `G1_ESCAPE`, `G1_RELOCGUARD` (safety layers).

Project memory lives in **`HANDOFF.md`** (session-by-session state) and **`PROBLEMS.md`**
(the tutor's problem list, one fix per run, with evidence). Read those first.

---

## Why it's hard (the constraints)

The G1 Air is **not** the EDU/developer unit:

| Want | Air reality |
|------|-------------|
| ROS 2 / DDS / CycloneDDS | ❌ internal bus not exposed on the WiFi AP (EDU-only) |
| SSH / low-level SDK (`rt/arm_sdk`, `lowcmd`) | ❌ forwarded but **not actuated** |
| Raw LiDAR cloud over WebRTC | ❌ only `odom` + `slam_info` reach a 3rd-party client; the dense cloud is decoded **only inside the app's WebView** |
| Two WebRTC sessions (one app, one ours) | ❌ robot allows **one** peer — the app holds it |

So the trick is: **don't fight the app — live inside it.** We attach to the WebView's JS over USB
(immune to the robot AP's client isolation), read the decoded cloud + odom, and publish velocity on
the app's own datachannel.

---

## Architecture

```
 iPhone (Unitree app, WebView)  ──WebRTC──  G1 robot
        │   ▲
   USB  │   │  ios-webkit-debug-proxy (CDP over USB)
        ▼   │
   Mac (Python)
     ├─ Perception
     │    ├─ LiDAR clean grid  (cloud → world-frame voxel grid: near-field exclusion + persistence + decay)
     │    ├─ Camera vision     (floor-color seg · edge · YOLOv8s · MiDaS depth — class-agnostic "obstacle ahead")
     │    └─ Collision sensor  (odometry stall = bumped something no sensor saw)
     ├─ Control     (inject rt/wirelesscontroller {lx,ly,rx,ry} @20Hz, in-page driver + dead-man)
     └─ Exploration (reactive wander + coverage novelty + redirection to unexplored)
```

Coordinate note: the cloud is in **Three.js Y-up** frame (the app's renderer), the odometry is in
**ROS Z-up** frame — they are reconciled in code (`cloud_x≈odom_x`, height=idx1, `cloud_z=-odom_y`).

---

## The scripts

**Navigation / autonomy**
- `g1_nav.py` — **the main program.** Unified capture + control + exploration. Modes:
  - `watch` — live odom + cloud point count (read-only)
  - `clr` — read-only obstacle clearances around the robot (LiDAR grid)
  - `vsee` — read-only camera vision readout (floor fractions, YOLO label, MiDaS depth_ratio)
  - `forward N` / `turn DEG` / `gorel F L` / `goto X Y` — closed-loop primitives (odom feedback)
  - `nav X Y` / `navrel F L` — go to a point **with reactive obstacle avoidance**
  - `vsee` / `clr` — read-only camera & LiDAR readouts (now show **metric distance** in meters)
  - `floorcal auto` — **calibrate** the camera's metric depth against the LiDAR (no tape measure; see below)
  - `explore [secs]` — **reactive** autonomous mapping/exploration (wander + coverage novelty)
  - `frontier [secs] [viz]` — **deliberative** exploration: arcs to the nearest reachable **frontier**
    with A* path planning. `viz` opens a live window (**map + robot camera**, great for screen-recording).
    Saves `map_latest.png` + `map_latest.json` every 30 s for later inspection.
- `g1_nav_v2.py` — **experimental** fork of `g1_nav.py` (stable one untouched). Adds: stronger models for
  Apple-Silicon (`G1_YOLO=yolov8m.pt`, `G1_DEPTH=DPT_Hybrid`), **information-gain frontiers** (heads for
  wide openings/doors, not the nearest nook), **analogy from past collisions** (avoids obstacles that
  *look like* something it hit before), confidence-aware speed, and an IMU-discovery scaffold (`imu` cmd).
- `g1_inject_teleop.py` — option-C teleop injection (sniff/capture/drive) — proof that we can move the robot via the app's datachannel
- `g1_teleop.py` — direct walking via `rt/wirelesscontroller` (app closed)

**Perception / viz**
- `g1_inspector_bridge.py` — live LiDAR cloud (2D/3D) + pose + path + **camera window with YOLO boxes**
- `g1_cam_probe.py` — probe the app's `<video>` element (camera)
- `slam_g1_mapping.py` — start/stop/save SLAM from Python (app closed)
- `g1_slam_viz.py`, `g1_map_viz.py` — odometry / saved-map visualizers

**Reverse-engineering / diagnostics**
- `dump_services.py`, `discover_slam_api.py`, `query_api.py`, `g1_slaminfo_dump.py`, … — how the API was mapped

**Docs**
- `AUTONOMOUS_NAVIGATION.md` — perception/control/exploration **algorithm** + a roadmap of
  improvements (incl. the meta-reasoning direction)
- `G1_Air_SLAM_SOLVED.md` / `.pdf` — the SLAM/WebRTC reverse-engineering writeup

---

## Quick start

Prereqs (on the Mac, in a venv):
```bash
brew install ios-webkit-debug-proxy
pip install websocket-client requests numpy matplotlib pillow ultralytics timm
# (MiDaS depth pulls its weights on first run; runs on Apple-GPU/MPS automatically)
```

Bring-up:
1. iPhone: Unitree app connected to the robot, **standing**, on the **SLAM/map screen**, **camera on**.
   iPhone Web Inspector ON; **don't** open Safari's inspector on that page (one debugger per page).
2. USB-connect the iPhone to the Mac (trust).
3. Terminal 1: `ios_webkit_debug_proxy`
4. Terminal 2:
   ```bash
   python g1_nav.py watch        # confirm odometry is live (x/y/yaw change when you move the robot)
   python g1_nav.py explore 90   # autonomous exploration
   ```

**Safety:** keep the physical remote in hand as a kill switch (L2+B = damping/stop), clear 2–3 m of
space, and start with the robot freshly charged (>80%) and standing in walk mode.

---

## Learning from failure (toward meta-reasoning)

Each real collision is treated as a perception signal:
- **Memory:** the bumped obstacle (which the LiDAR couldn't see) is *injected into the obstacle grid*
  so the robot doesn't hit it again.
- **Dataset:** every collision saves a camera snapshot + a `.txt` of *what each sensor reported at
  that instant* (`crashes/`) — i.e. *why* it failed (LiDAR blind, camera saw floor, …). This is the
  raw material for improving the visual model and for the robot to reason about its own blind spots.

The repo's name — *meta reasoning* — points at where this goes: a robot that knows the LiDAR misses
tables/glass and the camera confuses same-colour walls, **weights its sensors by context**, slows
down when its perception is uncertain or degraded, and **transfers** an avoidance learned on one
chair to an identical chair elsewhere (analogy). See `AUTONOMOUS_NAVIGATION.md`.

---

## Status

Working (validated on the robot, July 2026): SLAM read, teleop injection, closed-loop motion,
**A↔B door crossing on a static global plan with door engagement** (best runs: A→B 54.9 s /
0 collisions, B→A 108 s / 0 collisions), GPU perception with floor-color channel, **DCE
governance in shadow and active modes with experience-abort escalation**, full per-run
instrumentation for the paper's evaluation. Historical exploration modes (`g1_nav.py explore/
frontier`) still work. Current state, open problems and next steps: `HANDOFF.md` + `PROBLEMS.md`;
governance architecture: `docs/META2_GOVERNANCE.md`.

## Disclaimer

Research/interoperability work on the author's own hardware. Not affiliated with Unitree. Moving a
bipedal robot autonomously is inherently risky — supervise it, keep a kill switch, use a clear space.

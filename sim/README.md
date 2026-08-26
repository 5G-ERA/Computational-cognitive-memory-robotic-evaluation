# sim/ — the G1 digital twin

> **07 Aug 2026 — the twin is now REPRODUCIBLE.** Until then the container existed only in the
> Mac's Docker: losing it meant losing the environment. There is now a complete recipe:
> **`Dockerfile`** and **`docker-compose.yml`** (the originals the container was built with,
> rescued and versioned) + the scripts **`setup_g1.sh`** and **`regen_g1_nav.sh`** (which
> generate the robot description from Unitree's public repository) +
> **[`RUN_AND_REBUILD.md`](RUN_AND_REBUILD.md)** (run, rebuild, verify, back up).
> Audit finding: `rosbridge-suite` had been installed by hand inside the container and **was
> not in the image** — without it there is no bridge on 8765. It is now pinned in the recipe.

Docker container `g1sim:humble`, built on a Tiryoh-style **docker-ros2-desktop-vnc** base:
ROS 2 Humble Desktop plus a desktop served over noVNC, with **Gazebo, Nav2, slam_toolbox,
foxglove_bridge and pointcloud_to_laserscan** preinstalled.

## Where to go next

| If you want to… | Read |
|---|---|
| Run the twin, rebuild it from scratch, or back up the image | [`RUN_AND_REBUILD.md`](RUN_AND_REBUILD.md) |
| Fix something odd, view the world in 3D, or switch scenario | [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md) |
| Follow the full test protocol | [`../docs/G1_Twin_Simulation_Test_Instructions.pdf`](../docs/G1_Twin_Simulation_Test_Instructions.pdf) |

## Access

| What | How |
|---|---|
| Desktop (browser) | `http://localhost:6080/vnc_lite.html` (light client) or `/vnc.html` — password `ubuntu` |
| Raw VNC (native client) | port 5900 (not HTTP: Safari will not open it) |
| ROS websocket bridge | port **8765** — already mapped; this is what the adapter talks to |

Check the actual mappings with `docker port g1sim`.

## What is in this folder

| Path | What it is |
|---|---|
| `Dockerfile`, `docker-compose.yml` | The environment recipe (amd64 on purpose — Gazebo Classic has no arm64 build on Humble) |
| `setup_g1.sh`, `regen_g1_nav.sh` | Turn Unitree's official description into the navigation model: rigidised joints, fixed mesh paths, `base_footprint` + LiDAR + the `planar_move` plugin |
| `g1_sim_pkg/` | The simulation package: worlds (`lab.world` = the real flat, `room.world` = synthetic), launch files, URDF |
| `waypoints_sim.json`, `ref_map_room.json`, `nav_map_lab.json` | Scenario files — the simulator **never touches the real lab files** |
| `container_dump/` | Inspection dumps taken from the live container (environment, ROS packages, workspace layout) |
| `lab_world_preview.png` | Preview of the generated world |

## Integration: `g1_sim_adapter.py` — the same code as the real robot

The adapter replaces the WebView's CDP object with a **SimCDP** that resolves the very same JS
snippets against ROS 2 over **rosbridge** (JSON/websocket, port 8765):
`window.__cmd{lx,ly,rx,ry}` → `/cmd_vel` with the same calibrated stick physics (deadzone 0.3,
`lx>0` = right → `vy<0`, `rx` → `wz ≈ −1.55·rx`, `ly 0.4` → 0.30 m/s) · `/odom` → pose ·
`/scan` 360° → a flat `location` cloud in the map frame.

Because of that, **`g1_goto.py` runs unmodified** in both worlds. Simulation defaults set by
the adapter: `G1_ENV=sim`, a per-campaign `G1_SIM_ID`, `G1_NOVIS=1` (no camera unless the
synthetic one is enabled), door engagement off in the `room` scenario, and reloc-guard off
(the simulated pose is exact).

Runs land in `dataset/` tagged `env=sim` with their `sim_id`, in the dataset, in `goto.log`
and in the summary CSV columns — **they are never mixed with real-robot runs**.

## Twin-only capabilities (both default OFF)

- **`G1_SIM_NOISE=1`** — sensor noise calibrated against the real measured distributions:
  correlated per-scan bias and bursts plus odometry drift. Plain white per-ray noise does not
  work: the persistence filter absorbs it.
- **`G1_SIM_PERC=1`** — a synthetic camera: a perception service that derives the door bearing
  from the simulator's true pose with realistic latency, dropout and angular noise. This is
  what lets the twin rehearse the full real configuration, vision included.

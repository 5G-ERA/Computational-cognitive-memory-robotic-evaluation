# G1 digital twin — run it, rebuild it, back it up

Operational guide, 2026-08-07. Companion to `README.md` (what the container is) and
`TROUBLESHOOTING.md` (symptoms, GUI viewing, scenarios). This document covers what was
missing until now: **how to rebuild the twin from scratch if the container is lost**, and how
to keep a copy so that it never matters.

---

## 0. The three pieces (and where each one lives)

| Piece | What it is | Where it lives |
|---|---|---|
| **Container** `g1sim:humble` | ROS 2 Humble + Gazebo + a desktop served over noVNC | Docker on the Mac — **now also reproducible** from `sim/Dockerfile` |
| **Package `g1_sim`** | The worlds (`lab.world` = laser scan of the real flat), launch files, URDF | Versioned in `sim/g1_sim_pkg/` ✔ |
| **Adapter** | Runs the real navigation stack against the simulation | `g1_sim_adapter.py` in the repository root ✔ |

The key design decision: `lab.world` is in the **same map frame** as the real robot, so the
real waypoints, reference map and door are reused **with no translation at all**, and
`g1_goto.py` runs **unmodified** in both worlds.

---

## 1. Running the twin (day to day)

### 1.1 Start the container
```bash
docker ps | grep g1sim || docker start g1sim
```
If port 6080 is taken by another ROS container that starts by itself:
```bash
docker stop ros2_humble
```

### 1.2 The simulation (headless — the mode used for data runs)
```bash
docker exec -d g1sim bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/g1_ws/install/setup.bash && ros2 launch g1_sim lab.launch.py gui:=false > /tmp/lab_launch.log 2>&1"
```
Check it came up (`Successfully spawned entity [g1]` must appear):
```bash
docker exec g1sim grep -c "Successfully spawned" /tmp/lab_launch.log
```

### 1.3 The websocket bridge (ALWAYS manual after restarting the container)
This is the step most often forgotten: **relaunching Gazebo does not relaunch the bridge.**
```bash
docker exec -d g1sim bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/g1_ws/install/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=8765"
```

### 1.4 Place the robot and launch a run
The adapter does **not** reposition the robot: teleport it before every manual run.
```bash
docker exec g1sim bash -c "gz model -m g1 -x 0.99 -y 0.57 -z 0.05 -Y -2.1"     # position A
docker exec g1sim bash -c "gz model -m g1 -x -4.75 -y 2.60 -z 0.05 -Y -0.9"    # position B
```
The run is launched from the Mac with the experiment's environment (see
`docs/G1_Test_Protocol_Operator_Runbook.pdf`, §1.3) and the adapter:
`g1_sim_adapter.py goto A|B`.

### 1.5 Watching the simulation (noVNC desktop)
| What | How |
|---|---|
| Desktop in the browser | `http://localhost:6080/vnc_lite.html` (light client) or `/vnc.html` |
| Password | `ubuntu` |
| Native VNC | port 5900 (not HTTP: Safari will not open it) |
| ROS bridge | port 8765 |

To actually **see** the robot move you need `gui:=true`, but data runs always go headless:
software rendering steals CPU from the physics and distorts the timings.

---

## 2. Rebuilding the twin from scratch

**Real architecture (important):** the image contains *only the environment* (ROS, Gazebo,
noVNC desktop). The **workspace is not inside the image**: it is a *bind mount* from a folder
on the Mac. Today that folder is `~/Downloads/g1_sim_docker/g1_ws` (219 MB) — meaning the live
twin depends on **a folder in Downloads**. Move it somewhere stable and point `G1_WS` at it.

### 2.1 Bring the environment up
```bash
cd "<G1 repo root>/sim"
G1_WS=~/g1_ws docker compose up -d --build
```
The recipe (`sim/Dockerfile` — the **original** from 23 Jun, rescued and versioned on 07 Aug)
starts from the base image with the noVNC desktop, rebuilds the ROS and Gazebo apt sources,
installs the simulation and navigation stack — **including `rosbridge-suite`, which was
missing** — and sets up the shell environment. `docker-compose.yml` pins what cannot be
improvised: `platform: linux/amd64` (Gazebo Classic has no arm64 build on Humble),
`seccomp:unconfined` (required by the Jammy base), ports 6080/5900/8765 and `shm_size: 1gb`.

### 2.2 Populate the workspace (starting from nothing)
```bash
mkdir -p ~/g1_ws/src && cd ~/g1_ws/src
cp -r "<G1 repo root>/sim/g1_sim_pkg" ./g1_sim              # worlds + launch files + urdf
cp "<G1 repo root>/sim/setup_g1.sh" "<G1 repo root>/sim/regen_g1_nav.sh" .
git clone --depth 1 https://github.com/unitreerobotics/unitree_ros.git   # official description
bash setup_g1.sh && bash regen_g1_nav.sh                    # builds g1_description + g1_nav.urdf
docker exec -u ubuntu g1sim bash -lc "cd /home/ubuntu/g1_ws && colcon build --symlink-install"
```
`g1_description` (110 MB, Unitree Robotics' official package) is **deliberately not
versioned**: the scripts regenerate it. If you ever want a hermetic copy, the route is Git LFS
— not done today.

### 2.3 Verification after rebuilding (don't trust it: measure it)
1. `docker exec g1sim bash -lc "ros2 pkg list | grep -E 'g1_sim|g1_description|rosbridge'"` →
   all three present.
2. Launch headless (§1.2) → `Successfully spawned entity [g1]` must appear.
3. Bring up the bridge (§1.3) and teleport to A (§1.4).
4. One A→B run with the meta level: **it must arrive in 95–115 s with 0 collisions.**
   If it lands well outside that band, the physics is not equivalent to the original container
   and **the timings are not comparable** with previous campaigns: redo the twin baseline
   before using it for anything.

---

## 3. Backing up the current image

The original container carries six weeks of adjustments; the Dockerfile reproduces it, but
**until that has been built and verified once, the only faithful copy is the image on the Mac.**

### 3.1 Local copy (fast, no credentials, does not go into git)
```bash
docker save g1sim:humble | gzip > ~/Documents/g1sim_humble_$(date +%Y%m%d).tar.gz
```
Restore with `gunzip -c <file>.tar.gz | docker load`. About 2.6 GB compressed.

### 3.2 Publishing to GitHub's registry (GHCR)
**You have to run this yourself:** Docker on this Mac is logged into work Azure registries, not
GHCR, and the credentials are yours to handle, not the assistant's.

```bash
echo $GITHUB_TOKEN | docker login ghcr.io -u <your-user> --password-stdin
docker tag g1sim:humble ghcr.io/5g-era/g1sim:humble
docker push ghcr.io/5g-era/g1sim:humble
```
The token needs `write:packages`. It uploads ~2.6 GB. Afterwards, on any machine:
`docker pull ghcr.io/5g-era/g1sim:humble`.

**Decide the visibility before publishing:** the package inherits it from the organisation's
repository, and the image contains the scan of a private home (the `lab.world` world), so if
the repository is public this becomes public too.

---

## 4. Warnings

- **Architecture**: the image is `linux/amd64` and runs emulated on Apple Silicon. That is
  deliberate — the physics is calibrated that way. Building it natively for arm64 would change
  the timings.
- **The bridge is manual**: restarting the container leaves it down even when Gazebo comes back.
- **Collision model**: the robot uses the real envelope (0.44 m shoulders × 1.10 m). With the
  old permissive box the robot "crossed" the door with its shoulders inside the wall — a false
  success documented on 03 Jul 2026. Never go back to the simple box to "improve" results.
- **The recipe is the original**, not a reconstruction: `sim/Dockerfile` and
  `sim/docker-compose.yml` are the files the container was created with on 23 Jun 2026,
  rescued from `~/Downloads/g1_sim_docker/` and versioned on 07 Aug. The only change is the
  missing `rosbridge-suite` line. Even so, **the first build must be validated** with the check
  in §2.3 — nobody has run it yet.
- **The live workspace sits in `~/Downloads`**: 219 MB outside git, in a folder that cleans
  itself. Moving it is the cheapest and most valuable pending task in this whole section.

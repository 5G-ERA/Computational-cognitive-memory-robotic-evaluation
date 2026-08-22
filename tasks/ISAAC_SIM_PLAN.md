# Isaac Sim as the second simulator tier — plan

2026-08-21 (night). Adrián's question: can we deploy Isaac Sim with the G1 and our map on the
lab station, for better realism? Feasibility checked: **yes** — driver 610.43 on 2× RTX 3090,
Ubuntu 22.04, docker with NVIDIA CDI runtime already configured, 1.2 TB free. The
`nvcr.io/nvidia/isaac-sim:4.2.0` container pulls without NGC auth (download running).

## Doctrine: two simulators, two roles — never mixed

| | Gazebo twin (existing) | Isaac Sim (new) |
|---|---|---|
| Role | **statistically calibrated** cheap runs (35 s), regression + replay validation | **structural realism**: material-aware RTX lidar, path-traced camera, articulated gait |
| Fidelity kind | matched to the REAL robot's measured statistics (c0_std, laser_noise…) | physically plausible, but its gait policy ≠ Unitree firmware — declared |
| What it can do that the other cannot | reproduce our calibrated noise contract; instant | glass that transmits, lights that switch, a body that actually walks |
| Substrate discipline | REPRODUCIBILITY.md: per-substrate baselines, results never compared across simulators | same rule from day one |

The Isaac tier exists for exactly the three walls we hit this month: **W1 physics** (tinted
glass as a transmissive material seen by a raytraced lidar), **W2/W3 illumination** (scene
lights on/off with a camera that actually darkens — the degradation model becomes testable in
sim), and **gait realism** (the effective-noise layer G1_SIM_GAIT approximates what Isaac can
produce structurally).

## Phases (background project — does NOT touch Thursday)

- **P0 — install** (running): container pulled; first boot headless + WebRTC streaming check;
  `nvidia.com/gpu` CDI passthrough.
- **P1 — G1 asset**: Unitree G1 USD (official assets / Isaac Lab). Sanity: spawn + joint
  articulation. Locomotion: Isaac Lab velocity policy for G1 (pretrained if available;
  otherwise train on the 3090s — days, acceptable). DECLARED: policy gait ≠ firmware gait.
- **P2 — the office in USD**: generate walls/floor/door from the SAME map data that built
  `lab.world` (same frame as the real robot → same waypoints, zero translation — the trick
  that made the Gazebo twin comparable). Tinted-glass panel as a transmissive material at its
  real pose.
- **P3 — bridge**: Isaac ROS2 bridge → publish /scan (RTX lidar), /odom, subscribe /cmd_vel →
  the EXISTING rosbridge websocket + `g1_sim_adapter.py` connect unchanged. The whole harness
  (goto, dataset, bitácora, replay) works day one.
- **P4 — experiments**: glass angular signature vs the real 21-Aug tandas; light-switch W2/W3
  episodes; gait-induced scan statistics vs the real IMU/scan numbers (the same table
  `compara_marcha.py` prints).

## Honest limits, stated up front

Isaac's G1 walks with OUR policy, not Unitree's controller — its gait signature will differ
from the real robot's just like the rigid twin does, only less. RTX lidar glass behaviour is a
material model, not measured IR optics of THIS tinted film. Isaac results therefore feed
DESIGN and staging rehearsal, not the calibrated claims the Gazebo twin backs. Both stay.

## P0 log (22 Aug)

- `isaac-sim:4.2.0` pulls WITHOUT NGC auth (17.6 GB) — but **segfaults on this station**:
  `omni.kit.widget.viewport → __enable_hydra_engine` dies on stage open, headless included.
  Signature of driver-too-new (610.43 vs the 535/550 era 4.2 was tested on). Not fixable by
  config without losing the RTX renderer — which is the whole point here.
- Container gotcha: the image's default entrypoint is `runheadless.native.sh` and it swallows
  any command as arguments (launches the streaming app instead). Always run with
  `--entrypoint ./python.sh`.
- Newer tags exist and are pullable: **4.5.0, 5.0.0, 5.1.0**. Moving to `5.1.0` (2025 build,
  driver matrix much closer to 610; also the version whose Unitree/humanoid asset coverage is
  best). `p1_g1_sanity.py` written dual-namespace (omni.isaac.* 4.x / isaacsim.* 5.x).

## P0/P1 blocked — precise diagnosis (22 Aug)

Both `4.2.0` and `5.1.0` segfault in the RTX renderer at startup. Root cause narrowed to
**Vulkan being unusable inside containers on this host with driver 610.43.02**:

- Graphics libs DO get injected (correct 610 versions of `libGLX_nvidia`, `rtcore`, etc.),
  the ICD json is present, `ldd` shows no missing deps, and strace shows no missing files.
- Yet the Vulkan loader fails with `Could not get 'vkCreateInstance' via
  'vk_icdGetInstanceProcAddr' for ICD libGLX_nvidia.so.0` → only llvmpipe enumerates →
  Isaac's RTX plugin dies. Reproduced in a bare ubuntu:22.04 container with
  `NVIDIA_DRIVER_CAPABILITIES=all` — **this is not an Isaac bug**.
- The ICD declares `api_version 1.4.341` (very new); the loaders available (host and
  container are both 1.3.204, Ubuntu 22.04 stock) predate the 610 series by years. The
  `nvidia-container-toolkit` on the host is **1.19.1**, also predating this driver.

### Fix candidates, in order (first two need sudo — Adrián)

1. `sudo apt-get update && sudo apt-get install --only-upgrade nvidia-container-toolkit`
   then retry (newer toolkit tracks the 610-series lib/ICD layout).
2. `sudo apt-get install vulkan-tools` on the HOST and run `vulkaninfo --summary` — if the
   host itself cannot enumerate the 3090s on Vulkan, the driver install is graphics-broken
   (fine for CUDA, broken for Vulkan) and the fix is at driver-package level.
3. Interim path without RTX (no sudo): a physics-only kit experience would unblock P1
   (articulation) and P2 (map building) — RTX only becomes essential at P3/P4
   (lidar/camera). Parked unless 1-2 fail.

Container gotchas already banked: `--entrypoint ./python.sh`, `NVIDIA_DRIVER_CAPABILITIES=all`,
cache mounts, and the 17.6+GB images pull without NGC auth.

## P1 verdict (22 Aug) - CORRECTED after Adrian pushed back

**My earlier verdict ("driver 610 incompatible, park it") was wrong.** Adrian did not buy it;
he was right. NVIDIA's own checker, shipped inside the image, says the opposite:

```
isaac-sim.compatibility_check.sh
  |-- Driver version [supported]
  |-- GPU 0 [supported]   |-- GPU 1 [supported]
  |-- GPU 0: VRAM [good]  |-- GPU 1: VRAM [good]
  |-- CPU processor [supported]   |-- Operating system [supported]
System checking result: PASSED
```

And that experience **runs to completion without crashing** (verified twice). So Isaac DOES
run on this station; what fails is one specific plugin path.

### What is actually true, measured
- Vulkan is healthy in containers (after toolkit 1.20 + the `libegl1` discovery). Isaac's own
  banner enumerates both 3090s: `Driver Version: 610.43.02 | Graphics API: Vulkan`.
- The full app loads dozens of extensions and reaches `isaacsim.core` at ~9.4 s, while a
  **worker thread dies at 50-600 ms inside `librtx.scenedb.plugin.so!carbOnPluginStartup`**
  (RT scene database). Same signature on 4.2.0, 5.0.0 and 5.1.0.
- Ruled out by test: stale/shared shader caches (fresh caches crash too), multi-GPU
  (`--gpus device=0` crashes), privileged mode, X display (`DISPLAY=:1` + socket), non-root vs
  root, default streaming entrypoint vs `python.sh`, and the lighter
  `isaacsim.exp.base.python.kit` experience.
- Prime remaining suspect, from Isaac's own startup log: **`IOMMU is enabled`** (32 groups on
  this host, BIOS/kernel default, no `intel_iommu` in cmdline). NVIDIA documents IOMMU as a
  known cause of Omniverse/Isaac instability. The compat check passes because it never builds
  the RT scene database - exactly the component that dies.

### RESOLVED AND FIXED (22 Aug) - driver branch was the blocker; P1 is DONE

Diagnosis confirmed by GitHub isaac-sim/IsaacSim#651 (Ubuntu 24.04 + driver 610 + segfault in
`rtx.scenedb.plugin`, reproduced across 4090/5080/5090/5060 Ti): **Isaac 5.1 validates driver
580.65.06; the 590/610 branch breaks the RTX renderer.** Others tried `intel_iommu=off` with no
change, so the reboot we nearly spent on that would have been wasted - our P2P health check had
pointed the same way. Note NVIDIA's own `compatibility_check` says PASSED: it is lenient and
does not verify the validated driver branch, which is why it disagreed with the crash.

**Fix applied the same day** (Adrian ran the sudo steps): `nvidia-driver-580-open` 580.178.04.
Two gotchas worth banking: the install aborted midway on a dpkg file conflict
(`libnvidia-cfg1-580` vs an in-transaction upgrade of `libnvidia-cfg1` to 610.57.04) - resolved
forward with `dpkg -i --force-overwrite` on that one package, then `apt-get -f install`, then
the driver; and the machine boots a different kernel (6.8.0-138) than the one it was running
(136) - DKMS had built and signed modules for both, so the reboot was safe. Autologin verified
BEFORE the reboot by restarting gdm alone: the session came back with no password, but as `:0`
instead of `:1` - a stale `DISPLAY=:1` in our notes would have failed silently on Thursday
(fixed; `tools/arranca_percepcion.sh` owns its own Xvfb `:99` and is immune).

Post-reboot state, all verified: driver 580.178.04 on both 3090s; torch cu124 sees 2 GPUs and
runs a GPU matmul (Thursday's perception stack intact); autologin session up; **Isaac Sim boots
clean** - `app ready` at 8.5 s, `Simulation App Startup Complete`, no crash.

## P1 DONE - the G1 is alive in Isaac

`/ws/p1_g1_sanity.py` against `Isaac/Robots/Unitree/G1/g1.usd` (NVIDIA also ships `G1_23dof`,
plus `Dex3`/`Dex5` dexterous hands):

```
JUNTAS: 43
nombres: left_hip_pitch, right_hip_pitch, waist_yaw, left_hip_roll, waist_roll,
         left_knee, right_knee, left_shoulder_pitch ...
pose z inicial 1.042 -> final 0.145 (fisica actua: SI)
=== P1 SANITY OK ===
```

43 articulated DOF and gravity acting on the body - not the rigidised block that slides in the
Gazebo twin. This is the structural answer to Adrian's "it moves like furniture".

## Next: P2 - the office in USD
Generate walls/floor/door from the SAME map data that built `lab.world`, in the same frame as
the real robot, so waypoints transfer with zero translation. Then the tinted-glass panel as a
transmissive material (the RTX material system is live: the startup log shows
`omni:rtx:material:db:flattener:transmittance_color`).

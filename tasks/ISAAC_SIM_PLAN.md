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

## The realism ladder (22 Aug, night) — channel by channel, each rung with its metric

Principle, proven by the mesh-detector bench: **"more real" means the sim elicits the same
responses from the ROBOT's sensors and stack as reality does** — never "prettier to humans".

1. **Lidar (P3)** — RTX FlatScan at G1 height vs the real laser_snapshots. FIRST RESULT at
   waypoint A: 31 common sectors, median |dr| = 0.24 m (at the 0.2 m cell resolution floor),
   p90 1.25 m. Coverage gap: real 25% of sectors vs sim 69% — the app's cloud is heavily
   filtered/decimated, so realism here means IMPOVERISHING the sim scan (model the app's
   filter), not enriching it. `sim/isaac/sim_lidar_A.py` + `analysis/compara_lidar_sim.py`.
   Gotchas banked: config must be `Example_Rotary_2D` (FlatScan refuses 3D configs), data
   lives in the ANNOTATOR (`IsaacComputeRTXLidarFlatScan` -> `linearDepthData` +
   `azimuthRange`), and it needs ~20 stepped frames before producing.
2. **Glass (W1)** — tune the transmissive material until the RTX lidar reproduces the real
   angular signature measured 21-Aug (0.44 face-on, 0.32 @30deg).
3. **Camera -> perception** — feed Isaac frames to the real perception_server; calibrate
   against the real light-by-distance chair curve (0.92@1.5 / 0.82@1.8 lit; 0.47-0.61 dark).
4. **Illumination (W2/W3)** — switchable scene lights matching measured brightness
   (~116 full / ~85 low); rehearse the DOOR_VIS +9deg bias in sim.
5. **Textures from real photos** — project real frames onto walls; cardboard texture for the
   boxes (pass criterion: the detector says "refrigerator" like reality does).
6. **Gait (P4)** — locomotion policy on the articulated G1, calibrated against real IMU sway.
7. **Full loop** — g1_goto driving Isaac's G1 through the adapter; same commands -> comparable
   trajectories vs the 133 real runs.

## Rungs 1-2 progress (23 Aug, small hours)

**Rung 1 (lidar) — filter v1 validated at two poses.** The app's cloud filter, measured over
1444 real snapshots: ~82 points/snapshot, ~62 of 180 sectors, **hard range cap at 3.7 m**
(94% under 3 m), near-biased histogram {0-1m:12%, 1-2m:45%, 2-3m:37%, 3+:6%}. Applying it to
the sim scan (`analysis/filtra_scan_app.py`): at A median |dr| 0.24->0.19 m and p90 1.25->
0.43 m; cross-validated at B with UNCHANGED parameters (`analysis/valida_filtro_B.py`):
median 0.24 m, p90 0.75. Range errors sit at the cell-resolution floor and TRANSFER.
Remaining gap is coverage shape (A: sim too rich, B: too poor) -> the fixed 62-sector budget
is wrong; v2 should use constant per-range-band retention instead.

**Rung 2 (glass) — mechanism found, dial pending.** The RTX FlatScan ignores material
transparency entirely: UsdPreviewSurface opacity 0.22 AND full OmniGlass MDL both return
31/31 sectors off the glass. What DOES work: `primvars:doNotCastRays` (returns collapsed in
the glass window when set). Design for the effective model: replace the glass slab with thin
vertical strips, ~44% marked doNotCastRays - face-on absence matches the measured 0.44, and
the angular trend (more returns oblique, 0.32@30deg) EMERGES naturally because oblique rays
cross more strips. Before dialing: the capture harness must wait for a full rotation
(current first-nonempty grab returned a partial 34-sector buffer) and calibrate the sensor's
azimuth offset once against a known wall instead of best-fit searching per scan.

## Rung 2, second pass (23 Aug) — mechanism settled the hard way

Chronicle of a falsification, kept because it teaches:
- The azimuth offset is **0 degrees**: calibrated at A against pure geometry, median |dr|
  0.07 m over 124 sectors. All earlier "best-fit offsets" (232/246/256) were overfitting on
  sparse windows. The FlatScan azimuth is world-aligned; never fit per scan again.
- `primvars:doNotCastRays` (authored in USD) does NOTHING for this lidar - proven by three
  scene generations with different strip patterns yielding IDENTICAL signatures, and the
  earlier runtime "success" was actually the sensor breaking (34-sector collapse), a false
  positive I had believed. Mechanism falsified.
- What DOES work: **USD visibility** (MakeInvisible). Isolated panel test (lone striped pane,
  no office): 5/8-invisible pattern -> **63% sector absence** where naive geometry predicted
  43% - the mechanism works and the dial responds. `sim/isaac/test_tiras_aislado.py`.
- Remaining blocker in the OFFICE scene is staging, not physics: the camera-derived box
  meshes near that wall return at 2.0-2.5 m inside the glass window, and the pane row is
  jagged (easternmost-cell-per-row of a scattered blob). For glass experiments the scene
  needs the 21-Aug staging: clear line to the specimen (relocate/remove those meshes), a
  straightened pane, and then dial the pattern against the real 44/32 signatures.
- Declared cost of visibility-based strips: the camera will see striped glass. Physical fix
  for later: RTX sensor materials with real reflectance.

## Rung 2 CLOSED (23 Aug) — glass calibrated, statistically indistinguishable from the real one

`sim/isaac/calibra_cristal.py`: a proper bench — straight pane, clear line of sight (the
21-Aug staging), 20 strip patterns swept at 0 and 30 degrees incidence in one boot.

Winner **3 visible / 5 invisible** strips of 3.33 cm, reproducible across three separate runs:
**36-40% bearing absence face-on, 32% oblique**, against the real **44%** and **32%**.

Why that is a match and not a near-miss: the real signatures come from only **25 informative
bearings each**, so their 95% intervals are **[25, 63]%** face-on and **[14, 50]%** oblique.
The simulated values sit comfortably inside both. Tuning further would be fitting noise —
declared, and deliberately not done. The angular trend (less absence when oblique, because
slanted rays cross more strips) EMERGES from geometry rather than being imposed.

**Harness gotcha, paid for twice:** sweeping many patterns in one Isaac boot degrades the RTX
lidar — signatures drift to 72%, then 100% (no returns at all). Only the first one or two
measurements of a boot are trustworthy. Verdicts now come from dedicated runs.

Still open for measuring the signature *inside the office scene* (not needed for the model
itself): the camera-derived box meshes sit in the line of sight and the reconstructed pane is
jagged. The experiment variant should stage a clear corridor as the real tanda did.

## Rung 3 (camera -> perception), first pass — meshes out, real-pixel cards in, gated

Adrián: "remove the meshes, I don't trust them; the detector and the boxes must be the same
as reality". Both done, and the second part measured rather than assumed.

**Meshes removed.** The bench already justified it (SM_Armchair -> "tv" 0.74 in OUR detector).

**Replaced by real-pixel cards.** `analysis/extrae_recortes_reales.py` finds, for each object
cluster, the real frame that produced its best detection, re-runs yolo11x to recover the box
(the stored dets carry only label/conf/bearing/range), pairs box to det by label+bearing, and
derives physical width/height from box + range. Cards are placed at the position implied by
THAT observation, standing on the floor, facing the observer.

**Four faults found and fixed by measuring, not guessing:**
1. Frames are **320x180 (16:9)**, not 640x480 — the server's fx=600/cx=320/cy=240 belong to
   4:3. Using cy=240 put the optical axis 60 px low and made every object float ~0.27 m.
2. Cards were **doubly lit**: the crop already carries the office's real illumination, and the
   scene light was applied on top, rendering them almost black. Now emissive.
3. Cards sat at the **cluster centroid** while viewed from one observation's pose — 0.93 m
   apart where the detection said 1.5 m, so the apparent size was wrong. Now at the
   observation-implied position, and the bench reproduces the robot's exact pose and yaw
   instead of "looking at the card" (framing matters to YOLO).
4. Structure blocks stood **in the line of sight**; the sight corridor is now evicted too.

**Result, honestly:** 1/18 -> 4/18. It works for **chairs** (sim 0.78-0.82 vs real 0.83-0.93)
and fails for couches and boxes. The failures look structural: couch crops are partial views,
and "refrigerator" was itself a COCO misclassification driven by whole-scene context, which a
flat card cannot reproduce. Same doctrine as the meshes now applies automatically: the office
generator reads `aprobadas.json` and places only cards that pass the bench — currently 4.

**Recommendation for the protocol:** for calibrated claims the sim vision channel should be
driven by a detection model fitted to the real curves (which we have), not by rendering.
Rendered vision stays for design and rehearsal, declared as not calibrated.

## Rung 3 RESOLVED — the sim vision channel is a calibrated emulator, not rendered pixels

Rendering was tried honestly and did not reach fidelity: library meshes make our detector say
"tv" where reality says "couch"; real-pixel cards reach only 4/18. Adrián approved switching
the channel to a **model fitted to real data**.

`sim/emulador_deteccion.py` emits detections in the server's own format
`[label, conf, bearing, range]` from robot pose + frame brightness.

**Evidence base — the only data with a clean denominator.** Run logs cannot serve: the `dets`
key exists ONLY when something was detected (88% of samples carry no key at all), so a
negative is ambiguous between "perception did not run" and "ran and saw nothing". The staged
`calib_luz` tandas do have denominators: object on a tape mark, N frames, count of hits.

**Batch labels were unreliable** (the reused-label defect). Conditions are therefore split by
**frame brightness**, the measured variable — which also settles the relabelling that was
pending for Thursday:

| distance | light (>=108) | low (<108) |
|---|---|---|
| 0.3 m | P 1.00, conf 0.94 | P 1.00, conf 0.94 |
| 1.5 m | P 0.67, conf 0.93 | P 0.78, conf 0.91 |
| **1.8 m** | P 0.95, **conf 0.82** | P 1.00, **conf 0.54** |

Light is irrelevant at 0.3 and 1.5 m and halves confidence at 1.8 m — the W2 money pair,
recovered cleanly from brightness alone.

**Two modelling faults found by validating rather than assuming:** the line-of-sight march
reported every object occluded, because the furniture sits in the occupancy map itself (the
ray now stops 0.45 m short of the target); and sampling confidence uniformly between min and
max gave 0.65 where reality gives 0.82, because the real distribution is skewed — now
triangular with the mode at the measured median.

`analysis/valida_emulador.py` replays all six staged conditions through the emulator: **all
six within tolerance**, and the W2 contrast survives (0.72 lit vs 0.54 dark).

**Declared limits:** the curve is measured for `chair` only and up to 1.8 m; beyond that the
model extrapolates conservatively and flags each emission as `extrapolado`. It is the
statistical twin of the perception server in simulation, never a replacement on the robot.

## Rung 7 reached: the G1 stack now drives ISAAC (Adrián's choice of platform)

`sim/isaac/isaac_bridge.py` makes Isaac speak the **rosbridge dialect the adapter already
uses** — the adapter needs only three things (subscribe `/odom`, subscribe `/scan`, publish
`/cmd_vel`), so the bridge implements that subset over a websocket and
`g1_sim_adapter.py` connects **unmodified** with `G1_SIM_URL=ws://localhost:8766`. This avoids
mixing ROS distros: Isaac ships Jazzy internally while the Gazebo twin runs Humble.

**First traverses inside Isaac:** A→B reached, error 0.35 m, 0 collisions, 39 s; B→A reached,
error 0.23 m, 0 collisions. 368 samples, 628 detections emitted by the calibrated emulator
(chair 194, couch 261, refrigerator 112, door 61) with median confidence 0.78.

**The laser is geometric, not RTX — declared.** Inside this bridge the RTX FlatScan annotator
returns 0 rays persistently (tested with the lidar held still, without a custom timestep, and
with the prim's original xformOps preserved), although it produces normally in dedicated runs
— the rungs 1-2 measurements stand. So `/scan` is a ray-march against the map carrying the
two calibrations that matter: the app's measured filter (result: 62-70 of 180 sectors return,
against the real robot's median of 62) and the glass as a **probabilistic pass-through** with
the bench signature (44% absence face-on, 32% at 30 degrees) — which is arguably better than
depending on the strip geometry. Restoring the RTX lidar in-bridge is left as a known task.

**Light-dependent door channel** reproduces the 21-Aug finding for rehearsal: under full light
the twin emits 2-4x more door observations, biased by the measured +8.8 degrees
(`G1_SIM_LUZ`, `G1_SIM_DOOR_BIAS`). First pair: 13 door observations lit vs 3 dark, exactly
the asymmetry seen on the real robot. Crossing stayed centred in both (+0.06) — the gate can
now be rehearsed before Thursday rather than discovered on the robot.

## Door-gate rehearsal in Isaac — INCONCLUSIVE, and a claim retracted

I claimed on one run that the twin "reproduces the real failure" (+0.138 vs the real +0.130).
With repetitions that does not hold: gate-off legs give |lateral| 0.010-0.034 m, nowhere near
the real 0.130. **The single matching run was coincidence and the claim is withdrawn.**

The diagnostic says why: on the real robot the biased bearing drove the strafe phase
**ENG-C 21 times** near the doorway; in Isaac it fires **0-1 times**. The mechanism does not
engage, so the rehearsal can neither validate nor refute the gate (measured 0.022 -> 0.048 m
with 2-3 legs per arm, which is noise). What the twin DOES reproduce: the asymmetry of the
channel itself — DOOR-VIS engages under light and not in darkness — and the gate provably
suppresses it (191/198 samples gated, luma 118 detected, `door_vis_gated` logged).

Closing the gap needs the approach geometry to match, not more repetitions.

## Isaac vs Gazebo — objective comparison against the real robot

`analysis/isaac_vs_gazebo.py`. Real runs are the gold standard; the question for each metric
is which twin sits closer to them.

| metric | REAL | GAZEBO | ISAAC | closer |
|---|---|---|---|---|
| traverse duration (s) | 56 | 55 | 40 | Gazebo |
| obstacles per sample | 290 | 477 | 362 | **Isaac** |
| frontal clearance c0 (m) | 2.50 | 0.70 | 0.64 | Gazebo (both far) |
| doorway observations | 32 | 51 | 36 | **Isaac** |
| ENG-C phases at the door | 10 | 6 | 1 | Gazebo |
| \|lateral\| in the gap (m) | 0.021 | 0.066 | 0.043 | **Isaac** |
| object detections per run | 32 | 866 | 650 | Gazebo (both far) |
| median object confidence | 0.60 | 0.86 | 0.92 | Gazebo |
| collisions | 0 | 0 | 0 | tie |

**Where Isaac is better:** map fidelity (obstacle density and doorway observations closer to
real, because the geometry comes from the same 3D scan), crossing accuracy, and the ceiling it
opens — material-aware glass, controllable lighting, real camera rendering, an articulated
robot. Gazebo cannot do any of those.

**Where Gazebo still wins:** traverse pacing (its calibrated noise contract reproduces the
real duration), the ENG-C engagement count, and cost — CPU 19% and 850 MB against Isaac's
588% CPU, 4.2 GB RAM and ~1.5-2.3 GB of VRAM.

**Where BOTH are far from real:** frontal clearance (0.6-0.7 m vs 2.50 m — both twins are
crowded by geometry the real robot does not see) and detection volume (650-866 per run vs 32,
because the emulator fires on every opportunity while the real perception server ran at ~1 Hz
with drops). Both are fixable and both are worth fixing before either twin is used for
calibrated claims.

**Honest bottom line:** Isaac is not yet globally more faithful than Gazebo — it wins three
metrics, loses three and ties on safety. Its advantage is the ceiling, not today's fidelity.

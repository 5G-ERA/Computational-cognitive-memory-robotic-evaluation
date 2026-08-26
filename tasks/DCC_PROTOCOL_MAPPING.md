# Mapping the G1 stack onto Renxi's DCC evaluation protocol

**Source documents** (17 Aug 2026): *A Computational Theory of Cognitive Memory* (v2, main +
supplementary) and *DCC Robotic Coffee Delivery Evaluation Protocol* v1.0 draft.

**What this document is for.** The protocol's robotic benchmark is the coffee-delivery task — a
humanoid carrying an open cup through narrow corridors, with an instrumented surrogate payload
instead of hot coffee. That is this project. The paper's robotic results table is written but
**empty**: 65 `[[ ]]` placeholders, including `[[C3 ESTIMATE]]`, `[[C4 ESTIMATE]]`,
`[[ROLE-TRANSITION RESULT]]` and `[[EFFICIENCY ENDPOINT]]`. **These sessions are what fills it.**
This document maps every field the protocol requires onto what the G1 emits today, and proposes
how to build the one thing we have nothing for: the independent governed-resolution reference.

---

## 1. The structural problem, first

The protocol needs one robot that both **crosses the door reliably** and **exposes DCA-conformant
interface fields**. We have those in *different branches*.

| Branch | Crosses the door | Emits `laser_trust`, `door_contra`, `iface_q`, `meta_state` |
|---|---|---|
| `feature/door-centring-rate` (tag `golden-doorcross`) | **yes** — 5/5 on 14 Aug | **no** |
| `tutor-feedback-metareasoner-sim`, `feature/voxel-memory` | untested at the door since the fixes | **yes** (`G1_METASM=1`) |

Nothing in the protocol can be measured until these are one branch. That merge is the first task,
and it is not cosmetic: the three door fixes (`G1_DOOR_CTR2`, `G1_DOOR_YAW2`, `G1_DOOR_EXIT_CTR`)
touch the same door state machine that the META work touches.

---

## 2. Interface field mapping

The revised interface is
`I¹ = ⟨global map, local map, current sensor readings, historical sensor readings, quality,
uncertainty, authority⟩`.

### 2.1 Already emitted (original interface, `I⁰`)

| Protocol field | G1 field | Where |
|---|---|---|
| Global map | `nav_map_lab.json`, reference grid | `data/` |
| Local map | occupancy cells, `nobs`, `n_hard` | per-sample |
| Current lidar | `c0`, `c0_hard`, `c0_std`, `clearance_m`, `clearL_m`/`clearR_m`, `clear_left`/`clear_right`, `balance` | per-sample |
| Current RGB | `perc_n`, `[VIS]` log line: `free_c`, `vismap`, `color`, `carpet`, `dets=[...]` | per-sample + log |
| Current pose | `x`, `y`, `yaw`, `loc_conf`, `loc_match` | per-sample |
| Payload state | spill marker (`spill_mark.py`), operator-scored | per-run |
| Battery | `bat` | per-sample |

### 2.2 Already emitted, but only on the META branch (`G1_METASM=1`)

| Protocol field | G1 field | Note |
|---|---|---|
| Sensor-quality state | `laser_trust` (retrospective laser validity), `laser_noise`, `scan_churn`, `scan_fresh`, `filt_rej` | `laser_noise` etc. exist on both branches; `laser_trust` only on META |
| Contradiction between senses | `door_contra` (door-centre contradiction) | META only |
| Interface quality | `iface_q` | META only |
| Resolved meta-state | `meta_state` = NORMAL / DEGRADED / BLIND / RECOVERY / ASSIST | META only |

### 2.3 Missing — must be built

| Protocol field | Status | Notes |
|---|---|---|
| **Selected historical lidar** before/during dead-region entry | **partially built, currently failing** | This is `G1_VOXMEM` on `feature/voxel-memory`. The twin verdict was 5/6 neutral, 1/6 catastrophic (7 collisions in one run, 41 cells held vs 17–24 healthy). Cause: only TTL was implemented, not ray-traced clearing, so a re-confirmed cell never expires. **The protocol makes this mandatory, not optional** — it is a required field of `I¹` and "Remove historical lidar" is a prespecified ablation |
| **Selected historical RGB** before/during illumination change | not built | We keep camera JPEGs per run (`_tNNNs.jpg`) but they are not part of the decision interface |
| **Illumination adequacy** | not built | Currently the robot cannot distinguish "no object" from "cannot see". This is the protocol's central aliasing witness |
| **Lidar coverage / health state** | not built as a field | We *know* the vendor cut (~1 m, `NEAR_BLIND`) but never expose "this region is coverage-limited" as distinct from "this region is free" |
| **Observation uncertainty, timestamps, coordinate frames** | partial | Timestamps yes (`t`), frame is implicit (`frame_check` in the header), per-observation uncertainty no |
| **Applicable authority** (safety / navigation / resumption) | implicit only | The guard chain can only ever remove speed, and META caps speed — the *behaviour* is partitioned, but the applicable authority is never emitted as a field |
| **Resolved cognitive role** | not emitted | `meta_state` is a *degradation* taxonomy, not the protocol's five roles. See §4 |

---

## 3. The five cognitive roles against our failure modes

The protocol's initial memory set maps cleanly onto phenomena we have already measured.

| Role | Protocol relation | What we have measured |
|---|---|---|
| Motion | speed/turning affect payload stability and spill risk | Spill marker, `spd` caps, `G1_M2_FRAGSPEED` |
| Sensor / lidar coverage | corridor geometry and coverage affect spatial uncertainty | The vendor blind band: **107 of 193 collisions happened with the laser reporting clear beyond 0.6 m**; 49 of those had been seen and then lost |
| Illumination | illumination adequacy affects RGB semantic reliability | Not instrumented — **our largest gap** |
| Object | reliable identification affects planning and completion | YOLO detections, `dets=[...]`, `door_vis` |
| Energy | battery affects available capability | `bat`; already identified as a dominant uncontrolled variable, and the strafe rate roughly halved across one session (3.44 → 1.96 cm/tick) |

---

## 4. Proposal: how to build Ω_t for the door crossing

`Ω_t(y_t) = ⟨δ_t(y_t), κ_t(y_t)⟩` — the prescribed decision plus the certificate. It must be
specified **independently of the architecture being evaluated**, or the assessment is circular.
This is the protocol's primary outcome and we have nothing like it.

**The key move: the reference comes from the experiment design, not from the robot's sensors.**
Every condition the protocol asks us to manipulate is one we *control*: the light switch, whether
a chair is placed, the battery band, which corridor geometry. If each episode is scripted, then
what the robot *should* resolve at each boundary is known by construction — no adjudicator has to
infer it from the robot's own telemetry.

### 4.1 Decision boundaries

Use the ones the code already emits, so boundaries are countable and reproducible:

1. **Phase transitions** already logged in `phase` (`DWA-F` → `ENG-T` → `ENG-AL` → `ENG-GO`/`ENG-CG`
   → crossed), plus the `door_engage`, `door_crossed`, `escape_start`, `meta2_*` events.
2. **Scripted condition changes**: light off at *t*, chair placed at waypoint *k*, battery crossing
   a declared band.

### 4.2 δ_t from the script

A deterministic table over *externally known* state — not over what the robot believes:

| Scripted condition at the boundary | Prescribed δ_t |
|---|---|
| Adequate light, no object, battery above band, wide corridor | `motion` |
| Robot within the geometric dead region, no confirmed object | `lidar_quality` |
| Lights off / changed, object question pending | `illumination` (or withhold RGB semantics) |
| Both lidar coverage and RGB inadequate | `review` or `defer` — **never a forced object conclusion** |
| Object placed and within reliable detection envelope | `object` |
| Battery below declared band | `energy` |
| Condition lifted (lights back, object removed) | **return** to the previously applicable role |
| Problem outside every preserved role | `no-use` (⊥) |

The dead-region entry is computable *a priori*: given the map geometry and the vendor cut, we can
mark for each pose which cells are coverage-limited. That makes "the robot is now blind here" an
externally derived fact rather than a sensor reading, which is exactly what independence requires.

**Caveat to state openly:** pose comes from the robot's own SLAM, so it is not strictly independent.
Pose is not the contested variable — but it should be recorded as a known limitation, and the twin
(true pose available) should be used to bound the error.

### 4.3 κ_t, the certificate

Per boundary, record: contextual evidence used, sensor provenance and quality, the payload and
battery state, the hard constraints in force, which authority permitted the action, and the reason
for continuing / switching / withholding / returning. Most of this we already dump at ~3 Hz; what
is missing is the **authority field** and the **reason code**.

### 4.4 What the robot must emit for scoring

`A_meta = 1[Z_t = δ_t]` needs `Z_t` — the role the robot actually resolved. Today the closest thing
is `meta_state`, a different taxonomy. **Add a `role` field** taking one of
`{motion, lidar_quality, illumination, object, energy, no_use, review, defer}`, plus `role_reason`
and `authority`. Small change, and nothing in the protocol can be scored without it.

---

## 5. Build order

1. **Merge the two halves** — door fixes and META fields into one branch (§1). Nothing is
   measurable before this.
2. **Emit `role`, `role_reason`, `authority`** (§4.4). This is what makes the primary outcome
   computable at all.
3. **Fix the voxel memory with ray-traced clearing** — mandatory field of `I¹`, and a prescribed
   ablation. Validate in the twin against the failure it already produced.
4. **Instrument illumination adequacy** — the protocol's central aliasing witness, and our largest
   gap. Needs a light schedule and a measured or declared adequacy signal.
5. **Implement C1**, the temporal incumbent verifier: retain / reject / **unresolved** over the
   single motion→payload memory. It is new code whose purpose is to be deliberately limited.
6. **Script the episode families** (stable, lidar degradation, illumination degradation, joint
   insufficiency, object identification, low battery, recovery-and-return, no-use control) with
   randomised or counterbalanced ordering.
7. Freeze the analysis model, then run confirmatory.

---

## 6. Two things to raise with Renxi before starting

**Held-out layouts.** The protocol requires reserving confirmatory corridor layouts and event
combinations from development use. Everything measured so far — including the `golden-doorcross`
configuration frozen on 14 Aug — was tuned on the one door in the one flat. Under this protocol
that is **development data**, and confirmatory runs need configurations we have never touched. With
a single flat this is a real practical constraint, not a formality, and it needs deciding before
any confirmatory run happens rather than after.

**Direction already written.** The supplementary states the robotic result — *"Only C3 and C4 met
the prespecified stability criterion"* — while every estimate in the table is still a placeholder,
and the protocol itself requires freezing the analysis before outcomes are inspected. If the G1
does not reproduce that pattern, the table has to say so. Worth agreeing explicitly, in advance,
that the numbers fill in whatever direction they land.

---

## 7. What we can already contribute to the paper

The 14 Aug session produced an empirical witness of **Theorem 1 (the Contextual Aliasing Bound)**
in our own data, in a variable the paper does not use as an example.

The door-crossing controller servos to a *vision-measured* door centre. The interface did not
distinguish "centre measured reliably" from "measurement stale because the jambs now sit inside the
LiDAR blind band". Those two contexts were representationally aliased, the robot resolved
"centred" while 0.19 m off-axis, and its left arm struck the frame. Restoring the distinction at
the interface boundary — past the gap centre, ignore the measured centre and servo to the map axis
— fixed it. **The measured aliasing bias was −0.11 m**, logged twice by the guard itself
(`DOOR-EXIT-CTR`).

This is the paper's own structure: a distinction collapsed in the interface, a resolver that could
not recover it, and the fix applied at the evidence boundary rather than in the controller.

A second contribution, methodological: **`ncol = 0` does not mean clean.** Our collision detector
runs on odometry and IMU, and a light arm scrape does not perturb the base. One arrival scored zero
collisions and touched the frame — the operator saw it, the instrumentation did not. The protocol
scores "surrogate spill, near-collision and emergency intervention", so this blind spot has to be
closed or declared before those numbers mean anything.

# T1–T12: staging specification

**Status:** draft for Renxi, 24 Aug 2026. The twelve transition configurations were fixed by
Renxi on 17 Aug (§4 of the pre-registration). This document does not revisit them: it
specifies *how each one is staged*, which is what the confirmatory tier still lacks.

Every configuration below states: the setup, the trigger the operator performs, δ_t on each
side of the boundary, what κ_t must record, the pass criterion, and whether the twin can
stage it. δ_t is known **by construction** — it comes from the script, never from the robot's
telemetry (§3 of the pre-registration).

**Reservation.** These twelve are reserved. Development uses separate stagings of the same
witness classes. Nothing below is to be run before the confirmatory session.

---

## Two conflicts to resolve before the first session

**1. T7 cannot be staged under the current battery rule.** §4 says confirmatory runs are cut
at 60%. T7 stages "battery band crossed", and the energy role in `dcc_roles.py` fires below
60. The transition would therefore coincide exactly with the stopping rule, so it is never
observable inside a valid run. Either the declared band for T7 moves up (70% is the natural
choice — the strafe rate was measured to halve between 63% and 46%, so 70% is still inside
healthy capability and leaves ten points of run before the cut), or T7 gets its own,
separately declared floor. **This needs a decision; it is a genuine hole, not a detail.**

**2. T7 is the only configuration the twin cannot stage.** The twin has no battery model —
`bat` reads 100 throughout. Everything else has a mechanism already: glass (`G1_SIM_GLASS`),
declared light (`G1_SIM_LUZ`, now the single source of truth for both the detector emulator
and the frame luma), objects (the calibrated detection emulator), blockage (geometry), and
the representative swap (the illumination gate). If the twin is admitted as a staging
surface, T7 stays robot-only unless a declared battery model is added.

---

## The geometry every configuration uses

Pose A `(0.99, 0.57)` at −120°; doorway centre `(−3.90, 1.25)`, crossing axis 135°, mouth
1.13 m; B ≈ `(−4.71, 2.84)`. These are the same constants the robot runs on
(`isaac_bridge.py`, `g1_goto.py`), not re-measured for this document.

---

## T1 · motion → lidar-coverage  ·  *switching*

| | |
|---|---|
| **Setup** | Tinted glass panel across the doorway flank, at the clean site established on 21 Aug. Room lit (RGB admissible). No object staged. |
| **Trigger** | None — the transition occurs when the robot's approach brings the glazed region inside the coverage band. |
| **δ_t before** | `motion` |
| **δ_t after** | `lidar_quality` |
| **κ_t records** | `cov_blind`, `cov_missing`, `cov_n`, `laser_trust`, pose, the declared glass rectangle, `authority` |
| **Pass** | The condition resolves `lidar_quality` within the declared window of the scripted boundary, with `illum` still admissible so the transition cannot be attributed to lighting. |
| **Twin** | Yes — `G1_SIM_GLASS` discards returns inside a declared rectangle while the wall stays in the world and in the map. |

**Why it is not trivially satisfied:** the site matters. Two 21-Aug tandas were invalidated by
siting — `cov_blind` = 1.00 with the robot inside mapped clutter, so every bearing died in the
blind band and the field carried no information. Site validation precedes the run.

## T2 · lidar-coverage → motion  ·  *return*

Same setup as T1; the robot leaves the glazed region. δ_t returns to `motion`. **Pass** requires
the return, not merely the exit: a condition that stays in `lidar_quality` after the ground is
gone has failed the return outcome, which §5 scores separately. Twin: yes.

## T3 · motion → illumination  ·  *switching*

| | |
|---|---|
| **Setup** | Room lit, no glass, no object. Robot on approach. |
| **Trigger** | Operator switches the lights, **recording wall-clock time** — that is the independent record and it is what Ω_t takes. |
| **δ_t before** | `motion` |
| **δ_t after** | `illumination` |
| **κ_t records** | `illum_b` (the EMA, per the frozen contract), `dvis_gate`, `door_b`, the declared switch state and its timestamp |
| **Pass** | The condition withholds RGB door semantics and does not emit `object`. |
| **Twin** | Yes — `G1_SIM_LUZ` drives both the emulator and the frame luma. |

**Contract dependency.** Admissibility is `EMA(α=0.2) of mean frame luma > 99`
(`tasks/VISUAL_QUALITY_CONTRACT.md`). Its separating gap is 2.7 luma units, which is clean on
the material we have but thin. The contract also has **no hysteresis**, and the mixed-light run
of 21 Aug shows the EMA crossing the threshold *within a single traverse*. For T3/T4 that is a
live risk: a role transition should require evidence, not one crossing. Adding entry/exit
thresholds plus a minimum dwell before the confirmatory tier is recommended.

## T4 · illumination → motion  ·  *return, renewal*

Lights back on, time recorded. δ_t returns to `motion`. **Pass** requires both the return *and*
that the original illumination mapping is intact afterwards — renewal, not replacement. Twin: yes.

## T5 · motion → object  ·  *switching*

| | |
|---|---|
| **Setup** | Lit room, no glass. The calibration chair placed on its tape mark at **1.8 m** — the witness distance established on 21 Aug, where light alone flips the detector between unstable (0.47–0.61 dark) and stable (0.82–0.85 lit). At 1.5 m both conditions sit at 0.92 and there is no contrast. |
| **Trigger** | Object placed before the run; the transition occurs when it enters the reliable envelope. |
| **δ_t before** | `motion` |
| **δ_t after** | `object` |
| **κ_t records** | detections with confidence, `perc_n`, `perc_age`, `illum_b`, declared object position |
| **Pass** | `object` resolved while RGB is admissible. |
| **Twin** | Yes — the calibrated detection emulator, whose cards are gated by our own detector. |

## T6 · object → motion  ·  *return*

Object removed mid-run (operator, time recorded). δ_t returns to `motion`. **Pass** requires the
return and the absence of `object` persistence — false persistence of a temporary role is a
named failure in §5, not an inefficiency. Twin: yes.

## T7 · motion → energy  ·  *switching*

**Blocked pending the decision above.** Setup: run started at a declared battery level such
that the band is crossed *inside* the run and above the 60% cut. δ_t moves `motion` → `energy`.
κ_t records `bat`, the declared band, and the replanning or governed non-use that follows.
**Pass** requires the plan to adapt, not merely a slower traverse. Twin: **no** without a
declared battery model.

## T8 · motion → defer  ·  *governed defer*

| | |
|---|---|
| **Setup** | Glass in place **and** lights off. Both grounds inadequate at once. |
| **Trigger** | Both staged before the boundary; times recorded. |
| **δ_t after** | `defer` (or `review` when advance is not vetoed) |
| **κ_t records** | Both grounds, plus which authority held the command |
| **Pass** | **No forced object conclusion.** §4.2 is explicit: with both channels inadequate the answer is review or defer, never an object claim. A confident object answer here is a failure, not a near-miss. |
| **Twin** | Yes — glass and light combine. |

This is the configuration the resolver was designed around: with both grounds missing,
`dcc_roles` resolves `review`/`defer` by construction rather than by a rule written for this case.

## T9 · defer → motion  ·  *renewal after defer*

Lights restored and glass removed, times recorded. δ_t returns to `motion`. **Pass** requires
renewal from a deferred state — the harder half, and the one C1 cannot reach at all, since a
single-memory verifier has no deferred state to renew *from*. Twin: yes.

## T10 · object → no-use  ·  *governed no-use*

Passage physically blocked while an object is identified. δ_t resolves `no_use`: the problem
falls outside every preserved role. **Pass** requires a governed no-use outcome — a terminal
state that is completion, governed defer or governed abort (§5 stability). A robot that simply
stops without resolving is stable but has not demonstrated continuity. Twin: yes, by geometry.

## T11 · representative swap inside the motion role  ·  *role identity under representative change*

| | |
|---|---|
| **Setup** | The doorway has two representatives of its centre: the vision-measured bearing and the map axis. The illumination gate decides which may govern. |
| **Trigger** | Light change, as T3. |
| **δ_t** | The **role is unchanged** across the boundary; only the representative changes. |
| **Pass** | The meta-decision does not change when the representative does. If the resolved role flips, role identity was not preserved under representative change. |
| **Twin** | Yes — rehearsed both ways already, −49% crossing deviation with the gate on. |

This is W3 in production, and the one configuration where a *null* result is the pass.

## T12 · successor mapping supersedes incumbent  ·  *non-rewrite*

A successor mapping is created while the original stays intact and readable. **Pass** is
verified in the record, not in behaviour: the original mapping must be recoverable after the
successor exists. κ_t records both, with provenance and timestamps. Twin: yes — it is a record
property, independent of the world.

---

## What is instrumented today, and what is not

| Needed by | Field | State |
|---|---|---|
| all | `role`, `role_reason`, `authority` | ✅ emitted per sample (step 2) |
| T3, T4, T5, T8, T11 | `illum_b`, `dvis_gate` | ✅ with the frozen contract; **no hysteresis** |
| T1, T2, T8 | `cov_blind`, `cov_missing`, `cov_n`, `laser_trust` | ✅ — `cov_missing` needs `G1_COVREF`, and K_online is **not frozen** |
| T5, T6 | detections with confidence | ✅ |
| T7 | `bat` band | ⚠️ conflicts with the 60% cut |
| T1, T2 | historical lidar (voxel memory) | ✅ ray-traced clearing built; **exposes vs injects still undecided** |
| all | Ω_t certificates | ✅ guion.py writes them in the act of staging and copies them next to the run (since 24-Aug); facing gate in delta_muestra since 25-Aug |

The last row is the real remaining gap for step 6. Everything above it is either done or has a
named decision attached.

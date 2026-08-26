# DCC robotic benchmark — pre-registration

**Status: approved by Renxi Qiu, 17 Aug 2026.** Committed before any benchmark code is written, so
that "frozen before outcomes were inspected" is verifiable by commit date rather than asserted.
Revisions before confirmatory running must be committed as dated amendments; none after.

Companion documents: [`DCC_PROTOCOL_MAPPING.md`](DCC_PROTOCOL_MAPPING.md) (what the stack emits vs
what the protocol needs) and [`GOLDEN_DOOR_CROSSING.md`](GOLDEN_DOOR_CROSSING.md) (the frozen
object-level configuration, tag `golden-doorcross`).

> **This document replaces an earlier draft that was wrong in kind.** That draft laid out a
> factorial sweep over obstacles × lighting × battery × direction. Renxi's correction: *"it appears
> to treat the experiment primarily as a robot robustness benchmark … rather than as a test of
> whether cognitive memory and DCC improve the robot's final decisions."* The intended question is
> whether the meta-decision improves **when the same physical trajectory and evidence are
> represented in different cognitive forms**. A factorial adds the physical variance that this
> question requires us to *remove*. The design below holds the physical situation constant and
> varies only the cognitive representation.

---

## 1. Two tiers

| Tier | Conditions | What it establishes |
|---|---|---|
| **Replay** — primary controlled analysis | C1, C2, C3, C4 | Meta-decision accuracy and governed-resolution reconstruction on *identical* recorded evidence |
| **Physical** — closed loop | **C3 vs C4 only** | Only what replay cannot: authorised realisation, consequence capture, renewal, switching, return, historical non-rewrite |

**Why replay is primary.** One recorded episode — one trajectory, one evidence stream — is passed
through all four conditions. The physical situation is not *balanced* across arms, it is
**identical**. That is the strongest available control and it is what the question demands. The
robot's job becomes producing episodes, not repeating them once per condition.

We already have this machinery: `analysis/replay_msm.py` reconstructs the meta layer tick by tick
over recorded datasets, and has been run in shadow over 309 real runs.

**What replay cannot establish**, and therefore what the physical tier is for: authorised
realisation, consequence capture, renewal, switching, return, and historical non-rewrite.

---

## 2. Witness pairs

The experimental object is the **aliasing pair**: two physical situations that are *equivalent under
the original interface* `I⁰` but whose independently required outcomes are *incompatible*. Theorem 1
(Contextual Aliasing Bound) predicts that no resolver measurable with respect to `I⁰` can satisfy
both. The revised interface `I¹` must expose the distinguishing evidence.

A pair is admissible only if all three hold: (a) `I⁰` is genuinely equivalent across the pair,
(b) the required outcomes are incompatible, (c) `I¹` exposes the distinction needed to resolve them.
**Each pair carries an unresolved control** — a third case in which the distinguishing evidence is
*still absent even under `I¹`*, where the correct outcome is review or defer. Without it, the
revised interface would be rewarded for forced classification.

| # | Pair | Identical under `I⁰` | Incompatible outcomes | Paper claim |
|---|---|---|---|---|
| W1 | **Glass vs open doorway** | No LiDAR return | Not passable / passable | Contextual Span |
| W2 | **Dark-no-detection vs lit-no-object** | Negative RGB | Cannot see / no object present | Contextual Span |
| W3 | **Representative loss vs role loss** | A mapping becomes unavailable | Role survives on another representative / role is lost | Role equivalence |
| W4 | **Missing evidence vs unresolved authority** | Robot does not proceed | Defer for evidence / hold for authority | Authority Partitioning |
| W5 | **Governed no-use vs successor switching** | Incumbent stops governing | Resolved no-use / successor governs, original preserved | Review-defer, non-rewrite |
| W6 | **Stable continuation vs return after recovery** | Motion role active at the end | Never left / left and returned | Renewal |

W1 and W2 are approved; W3–W6 are Renxi's recommended additions and are adopted.

### 2.1 W3 already exists in our system

The door bearing has **two representatives of one role**: the vision-measured door centre
(`door_c_meas`) and the map axis. On 14 Aug the exit fix switched from the first to the second when
the jambs entered the LiDAR blind band — *the role was unchanged, the representative was replaced*.
That is precisely representative loss without role loss, already implemented and logged
(`DOOR-EXIT-CTR`, measured bias −0.11 m). Role loss is the case where both representatives fail.

### 2.2 W2 requires a visual-quality contract

Approved **with a visual-quality contract**: a declared operating envelope stating when an RGB
detection is admissible. Without it, "cannot see" has no defined boundary and the pair is not
scoreable.

**This cannot be written from existing data.** Measured over 1978 stored frames, mean frame
luminance is flat across the working day (09:00→18:00: 105, 103, 109, 107, 105, 104) while contrast
varies three times as much (σ/mean 0.37 vs 0.12). A mean pinned near 105 under every condition is
the signature of the camera's **auto-exposure**, not of constant lighting: it measures the AGC
target, not the room.

**Prerequisite task — calibration session.** Drive the lights through declared states, record
frames, and determine which statistic actually tracks illumination (contrast and grain are the
candidates; mean is not). The contract is then written against the statistic that responds, with a
declared admissible range, and frozen. Ω_t is unaffected — it takes the operator's switch state.

---

## 3. The independent reference Ω_t

`Ω_t(y_t) = ⟨δ_t(y_t), κ_t(y_t)⟩`, specified independently of the architecture under test.

**The reference comes from the experiment script, not from the robot's sensors.** Every
distinguishing condition is one we set: which side of the pair is staged, the switch state, whether
the passage is glazed or open, which representative has been disabled. What *should* be resolved is
therefore known by construction, and no adjudicator infers it from the telemetry of the system under
test. For the unresolved controls, δ_t is review or defer **by construction** — the staging
guarantees the distinguishing evidence is absent.

**Declared limitation:** pose comes from the robot's own SLAM, so any pose-derived staging fact is
not strictly independent. Pose is not a contested variable here; the twin (true pose available)
bounds the error.

---

## 4. Physical tier: scale and design

Per Renxi, 17 Aug:

| | Configurations | Repetitions | Conditions | Runs |
|---|---|---|---|---|
| **Preferred** | 12 reserved transition configurations | 3 | C3, C4 | **72** |
| Minimum defensible | 6 transition classes | 4 | C3, C4 | 48 |

**The independent unit is the transition configuration or the run — not each logged decision
boundary.** (This corrects the earlier draft, which counted 8–12 boundaries per run as units and
thereby overstated the effective sample size.)

The twelve transition configurations, each staging a role transition that replay cannot establish:

| # | Transition | Tests |
|---|---|---|
| T1 | motion → lidar-coverage (enters glazed region) | switching |
| T2 | lidar-coverage → motion (leaves it) | return |
| T3 | motion → illumination (lights off) | switching |
| T4 | illumination → motion (lights on) | return, renewal |
| T5 | motion → object (obstacle becomes identifiable) | switching |
| T6 | object → motion (obstacle removed) | return |
| T7 | motion → energy (battery band crossed) | switching |
| T8 | motion → defer (joint insufficiency staged) | governed defer |
| T9 | defer → motion (evidence recovers) | renewal after defer |
| T10 | object → no-use (passage blocked) | governed no-use |
| T11 | representative swap inside the motion role | role identity preserved under representative change |
| T12 | successor mapping supersedes incumbent | non-rewrite of the original record |

Battery: measured at ~1 point per run over 83 real runs. Confirmatory runs are **cut at 60%** —
the strafe rate was measured to halve between 63% and 46%, so capability degrades well before the
operational floor. At ~40 usable runs per charge, 72 runs is roughly two charges of pure running;
realistically 3–5 sessions with staging, repositioning and re-baselines.

---

## 5. Outcomes

### Replay tier
- **Current meta-decision accuracy** `A_meta = 1[Z_t = δ_t]`, and **governed-resolution
  reconstruction** `A_Ω`, over C1–C4.
- Contrasts: `C4−C3` (DCC given the revised information), `C4−C2` (interface given distributed
  resolution), `C3−C1`, `C4−C1`.
- **Unresolved controls are scored as correct only when the condition returns review or defer.** A
  confident answer on an unresolved control is a failure, not a near-miss.

### Physical tier (C3 vs C4)
- **Primary: correct authorised realisation and renewal.**
- **Efficiency: time to the correct governed transition** — *not* total door-crossing time.
- The following are **failures, not inefficient successes**: incorrect persistence of a role,
  unauthorised deployment, unsafe continuation, unnecessary switching.
- Recorded alongside: historical non-rewrite (the original mapping intact after a successor is
  created), consequence capture, spill, near-collision, emergency intervention.

**Stability** (binary, per episode): no uncontrolled safety-contract violation, no unresolved forced
continuation, no unbounded payload or control disturbance, and a terminal state that is completion,
governed defer or governed abort. Efficiency is interpreted only among stable episodes — permanent
stopping can be stable without demonstrating continuity.

**Adjudication.** Spill and arm contact are scored by the operator against a written rubric, because
the instrumentation cannot see them (§7.3).

---

## 6. Reserved material and the development rule

The twelve transition configurations are **reserved**: never staged during development. Development
uses separate stagings of the same witness classes.

**Operational rule.** The object-level controller is frozen at `golden-doorcross`. **All** meta-layer
development — role resolution, C1's verifier, the interface fields, the voxel-memory fix — happens on
development stagings only. The reserved configurations stay untouched by the layer under test.

This is what reservation actually protects here. The object-level controller is a **constant shared
by every condition**, sitting below the meta layer, so it cannot bias C4−C3; it inflates absolute
performance, not the contrasts. The meta layer is the thing under test and is not yet written.

---

## 7. Limitations declared in advance

1. **One environment** — Renxi's office, which is also where all development happened. No claim of
   generalisation across environments will be made. Absolute stability and efficiency figures are
   specific to this space.
2. **Object-level tuning on the evaluation geometry.** `golden-doorcross` was tuned on this door,
   frozen and dated 14 Aug, before this design existed. Contrasts are unaffected (§6); absolute
   numbers are not.
3. **`ncol = 0` does not mean clean.** The collision detector runs on odometry and IMU and cannot
   see a light arm scrape; on 14 Aug an arrival scored zero collisions and touched the frame. Until
   arm contact is instrumented, a human scores every crossing.
4. **The DCC layer has never run on the real robot.** Zero of 300 real runs carry `meta_state`,
   `laser_trust`, `door_contra` or `iface_q`; the 76 that do are all simulator runs. Every
   DCC-relevant claim currently rests on the twin.
5. **Replay fidelity.** `replay_msm.py` documents one deliberate conversion (trust recovery
   expressed per second rather than per tick, because real runs sample at ~2 Hz) and one
   approximation (no real reversal after a shadow RETREAT, approximated by a 20 s cooldown). Both
   must be restated wherever replay results are reported.

---

## 8. Build order

1. ~~**Merge the two halves**~~ — **done 17 Aug, branch `feature/dcc-integration`.** See §9.

2. **Emit `role`, `role_reason`, `authority`** — without an explicit resolved role there is no `Z_t`
   and the primary outcome cannot be computed.
3. **Illumination calibration session** → write and freeze the visual-quality contract (§2.2).
4. **Fix the voxel memory with ray-traced clearing** — a required field of `I¹` and a prescribed
   ablation; currently 5/6 neutral and 1/6 catastrophic in the twin.
5. **Implement C1** (temporal incumbent verifier: retain / reject / unresolved) and **C2/C3** as
   interface-restricted variants.
6. **Record the witness episodes** for the replay tier, including unresolved controls.
7. Freeze the analysis model; run replay; then the 72 physical runs.

---

## 9. Integration record — 17 Aug

`feature/dcc-integration` = `feature/door-centring-rate` (object level, tag `golden-doorcross`)
merged with `tutor-feedback-metareasoner-sim` (META interface fields).

**One real conflict:** both branches had independently added the same `DOOR_CTR_TOL` /
`DOOR_CTR_S` parameterisation with different wording. Resolved in favour of the door branch, which
is a superset, keeping one measurement the other comment recorded and this one did not: a 0.14 dead
zone consumes 70% of the ±0.20 m physical margin. The `CROSS` block was auto-merged by git without
being flagged, so it was reviewed by hand — all three door fixes survive intact.

**Deliberate change: `METASM` default `"1"` → `"0"`.** It shipped enabled on the `-sim` branch,
where the META machine was the object of the work. Here it coexists with a frozen object level, and
enabled-by-default would mean the merged branch no longer reproduces `golden-doorcross` — breaking
both the project rule and this document's premise that the object level is a constant, identical
across all four conditions. Verified first that every campaign passes the flag explicitly.

**Not merged:** the voxel memory. It stays on `feature/voxel-memory` until ray-traced clearing is
implemented (5/6 neutral, 1/6 catastrophic in the twin).

### Verification

*Static.* Of 214 added code lines, three touch the control path without naming a flag — the META
speed caps (0.24, 0.28) and the ASSIST stop. All three sit inside `if METASM:`, so with the flag off
the object level is untouched.

*Twin regression*, same door configuration in both arms, `METASM` the only variable:

| Arm | Runs | Arrived | Crossed | Collisions | Lateral offset at the gap |
|---|---|---|---|---|---|
| `merge_off` (must reproduce golden) | 3 | 3 | 3 | 0 | +0.061, +0.013, +0.030 |
| `merge_on` (META active) | 3 | 3 | 3 | 0 | +0.033, +0.015, +0.016 |

Interleaved order, calibrated noise, `METASM` the only variable. Every offset sits inside the
success band measured on the real robot (|lat| ≤ 0.14; the runs that failed on 14 Aug crossed at
−0.23). The META arm is marginally tighter, which is not a claim — n=3 per arm, and the twin is not
the robot.

Field emission confirmed: with `METASM=0` only the always-computed diagnostics appear; with
`METASM=1` all four DCA fields carry values and the machine transitions NORMAL ↔ BLIND. The META
layer active does not degrade the crossing.

### Operational caveat when switching branches

The merge brings the file reorganisation: **`waypoints.json` and `nav_map.json` move from the repo
root into `data/`**. Contents were compared before accepting the merge and are byte-identical, so
the robot navigates to the same waypoints — but any script or command referencing the old root path
will break after checkout.

### Observation to follow up

With `METASM=1`, `BLIND` occupies 39–50% of samples in the twin. That may be legitimate under
calibrated noise, or the predicate may be too eager. It matters because `meta_state` will feed role
resolution: a state that is on half the time discriminates little. To be checked against real runs
when the roles are implemented.

## 10. Amendment — coverage-field revision (20 Aug evening, before any confirmatory data)

Recorded the same evening as the diagnosis because the resolver contract changes; no reserved configuration has been run.

**Finding (measured on the 8 real runs of 20 Aug + 366 historical runs).** The coverage field
`cov_def` (fraction of headings the reference map predicts and the scan does not return) was
calibrated in the twin (threshold 0.20) against the Summit-derived map. On the real robot it
saturates: per-sample median 0.96. Cause, quantified: the cell overlap between what the G1
actually sees and the Summit map is ~50–59% — the field mostly measured "the G1 is not the
Summit". Under it, C3 resolved *unresolved* on 99% of samples and C4 *lidar_coverage* on 99%:
the principal contrast C4−C3 measured nothing. Two further structural facts: the fraction
jumps when the robot turns (the sector faces new surfaces), and the environment drifts week to
week (stable-cell overlap Jun→Aug 58%), so **any long-history absolute reference keeps a floor
of phantom predictions**.

**Revised field.** `cov_missing`: the number of cells of a *G1-own visibility reference* that
are predicted and ABSENT in ≥2 consecutive scans. Localized (no denominator that moves with
heading) and persistent (a turn transient lasts one scan; a coverage loss lasts the whole
approach). The reference is built by `tools/mapa_visibilidad.py` from `laser_snapshots`
(per-snapshot statistics with the exact `cov_def` geometry). Evidence-base note: snapshots
store the accumulated obstacle map clipped to ±2.6 m, so the replay field is declared on that
base with radius 2.5 m; the online variant (instantaneous scans, emitted when `G1_COVREF` is
set) is calibrated separately.

**Validation (development material, declared as such).** Synthetic glass — returns erased
from a declared world rectangle of the real 20-Aug recordings, the twin's `SIM_GLASS`
mechanism applied offline: detection in 8/8 runs at K=4, with ~3 false events/run
attributable to historical-map drift. Four-condition replay on those episodes (unit = run,
percentile bootstrap, 10,000 replicates, seed 7): **C4−C3 = +7.7 pp, CI95 [+5.9, +9.5];
C4−C2 = +7.4 pp, CI95 [+5.0, +9.7]** — same direction as the twin W1 (+5.7 pp). Failure modes
match the prescribed roles: C1 falsely retains, C2 is blind to the loss, C3 answers
*unresolved*, and only C4 reaches the correct governed transition (switching delay 7.4–17 s;
the other arms never arrive).

**Frozen consequences.**
1. Resolver trigger is `cov_missing ≥ 4` (replay). `cov_def` is still emitted as legacy
   evidence but no longer triggers the role.
2. Confirmatory sessions freeze the coverage reference **per session**: calibration laps
   (`G1_LASER_SNAP=0.5`, no staging) before staging, reference built and declared before the
   first scored run. Long-history references are development-only.
3. `K_online` for the instantaneous-scan variant is not yet frozen; the 21-Aug night session
   measures its normal-operation distribution.

**Flagged, not yet decided (needs Renxi).** The object-question predicate
(`c0_hard − c0 > 0.30`, twin-tuned) fires on 29% of real samples and dominates the
unnecessary-switching secondary (real distribution: p75 = 0.45, p95 = 1.34). Options: raise
the threshold to the real p95, or require persistence as an I¹-only derived field. Until
decided, the current threshold stands and its false-trigger rate is reported.

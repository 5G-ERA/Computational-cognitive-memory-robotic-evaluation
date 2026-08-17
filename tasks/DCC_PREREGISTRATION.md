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

1. **Merge the two halves** — the door fixes and the META interface fields are on different
   branches, and nothing is measurable until they are one.
2. **Emit `role`, `role_reason`, `authority`** — without an explicit resolved role there is no `Z_t`
   and the primary outcome cannot be computed.
3. **Illumination calibration session** → write and freeze the visual-quality contract (§2.2).
4. **Fix the voxel memory with ray-traced clearing** — a required field of `I¹` and a prescribed
   ablation; currently 5/6 neutral and 1/6 catastrophic in the twin.
5. **Implement C1** (temporal incumbent verifier: retain / reject / unresolved) and **C2/C3** as
   interface-restricted variants.
6. **Record the witness episodes** for the replay tier, including unresolved controls.
7. Freeze the analysis model; run replay; then the 72 physical runs.

# DCC robotic benchmark — pre-registration (proposal)

**Status: proposal, pending Renxi's sign-off.** Committed *before* any code is written for the
benchmark, so that "frozen before outcomes were inspected" is verifiable by commit date rather
than asserted. Nothing here may be revised after confirmatory runs begin; revisions before that
point must be committed as dated amendments.

Companion documents: [`DCC_PROTOCOL_MAPPING.md`](DCC_PROTOCOL_MAPPING.md) (what the stack emits vs
what the protocol needs) and [`GOLDEN_DOOR_CROSSING.md`](GOLDEN_DOOR_CROSSING.md) (the frozen
door-crossing configuration, tag `golden-doorcross`, 14 Aug).

---

## 1. Physical resources actually available

Confirmed with the operator, 17 Aug 2026:

- **One door**, in the flat that is also the mapped environment. This is the binding constraint.
- **Two obstacles**: a **chair** and the **G1's own transport crate**. They are not interchangeable
  and that is useful — the chair has thin legs and a seat above the scan plane, so it is *hostile
  to the LiDAR*; the crate is a large solid, *reliably detected*. One exercises the lidar-coverage
  role, the other the object role.
- **Lighting switchable at any moment**, including mid-run. This is what makes the illumination
  role measurable at all.
- **Battery**: measured over 83 real runs, **~1 percentage point per run**, median run 87 s.

### Measured illumination baseline

From 1978 stored camera frames across 76 runs: mean frame luminance runs **92–116 per run**
(global range 45–169). That narrow band is the *adequate* condition — it also shows illumination
has never been manipulated, so no existing run contributes to the illumination witness. The
baseline exists so that "inadequate" can be defined against a measured reference rather than by
eye.

---

## 2. Factor space

**Fixed by condition (the 2×2 of the protocol):** decision process (temporal incumbent verification
vs distributed meta-resolution) × interface (original `I⁰` vs revised `I¹`) → C1, C2, C3, C4.

**Geometry variants** (chair = *Ch*, crate = *Cr*):

| | Layout |
|---|---|
| G0 | Clear approach |
| G1 | Crate narrowing the approach on one side |
| G2 | Chair in the approach |
| G3 | Both, staggered |

**Lighting schedules:**

| | Schedule |
|---|---|
| L0 | On throughout |
| L1 | Off before the door approach, stays off |
| L2 | Off on approach, back on after crossing — **this is the return test** |
| L3 | Off from the start, on at mid-corridor |

**Battery bands:** high (100–80), mid (79–60), low (<50, used only by the energy family).
Confirmatory runs outside the energy family are **cut at 60%**: yesterday the measured strafe rate
halved between 63% and 46%, so performance degrades well before the operational floor.

---

## 3. Episode families

The eight families of the protocol, each realised with the resources above. **Defining factors are
fixed — they are what makes the family that family — and are not randomised.**

| Family | Geo | Light | Battery | What it presents |
|---|---|---|---|---|
| F1 stable | G0 | L0 | high | No role degraded; motion/payload stays active |
| F2 lidar degradation | G2 | L0 | high | Chair: dead region without a confirmed object |
| F3 illumination | G0 | L1 | high | RGB unusable, LiDAR healthy |
| F4 joint insufficiency | G2 | L1 | high | Neither sensor suffices → review/defer, **never a forced object conclusion** |
| F5 reliable object | G1 | L0 | high | Crate: solid and well detected |
| F6 low battery | G0 | L0 | low | Energy limits available capability |
| F7 recovery and return | G2 | L2 | high | Light returns *and* the chair is removed mid-run |
| F8 no-use control | G3 | L0 | high | Passage blocked: no preserved role applies |

**Randomised within each family** (the free factors): direction (A→B / B→A), initial lateral offset
(−0.15 m / centred / +0.15 m) and event timing (early / late).

---

## 4. Reserved cells

Drawn with seed **`G1-DCC-2026-08-17`**, three realisations per family, one reserved. Reserved cells
are **never run during development** and are used only for confirmatory measurement.

| Family | Realisations (direction, offset, timing) | **Reserved** |
|---|---|---|
| F1 stable | B→A,+0.15,early ǀ B→A,centred,late ǀ A→B,−0.15,late | **B→A, +0.15 m, early** |
| F2 lidar degradation | A→B,+0.15,early ǀ A→B,−0.15,early ǀ A→B,centred,late | **A→B, +0.15 m, early** |
| F3 illumination | A→B,+0.15,early ǀ A→B,−0.15,late ǀ B→A,+0.15,early | **A→B, +0.15 m, early** |
| F4 joint insufficiency | A→B,−0.15,late ǀ B→A,centred,early ǀ A→B,+0.15,early | **A→B, −0.15 m, late** |
| F5 reliable object | A→B,+0.15,late ǀ A→B,−0.15,early ǀ B→A,+0.15,late | **A→B, +0.15 m, late** |
| F6 low battery | B→A,centred,late ǀ A→B,centred,early ǀ A→B,−0.15,late | **B→A, centred, late** |
| F7 recovery and return | B→A,centred,late ǀ B→A,−0.15,early ǀ A→B,+0.15,early | **A→B, +0.15 m, early** |
| F8 no-use control | A→B,+0.15,early ǀ B→A,−0.15,early ǀ A→B,+0.15,late | **A→B, +0.15 m, early** |

---

## 5. Ω_t — the independent reference

**The reference comes from the experiment script, not from the robot's sensors.** Every condition
the protocol requires us to manipulate is one we control: the switch, the obstacle, the battery
band, the geometry. With the episode scripted, the role that *should* be resolved at each boundary
is known by construction, and no adjudicator has to infer it from the telemetry of the system under
test. That is what keeps the assessment non-circular.

**Two illumination signals, deliberately kept apart:**

- the **switch state** (scripted, known to the operator) feeds **Ω_t**;
- the **camera-measured luminance** is an **interface field** — what the robot may know.

If the robot derived the reference from its own camera, the evaluation would be circular. The same
separation applies to the dead region: which cells are coverage-limited at a given pose is
computable *a priori* from the map geometry and the vendor cut, so blindness is an externally
derived fact, not a sensor reading.

**Decision boundaries** are the phase-family transitions already logged in `phase`, plus the
`door_engage` / `door_crossed` / `escape_*` / `meta2_*` events, plus each scripted condition change.
Measured over 296 real runs: median 9 phase-family transitions and 15 events per run, giving
roughly **8–12 scoreable boundaries per run**.

**Declared limitation:** pose comes from the robot's own SLAM, so the dead-region derivation is not
strictly independent. Pose is not the contested variable, but the twin (true pose available) should
be used to bound the error, and the limitation stated in the paper.

---

## 6. Sample size

Confirmatory = 8 families × 1 reserved realisation × 4 conditions = **32 cells**. At **3 replicates
per cell → 96 confirmatory runs**, yielding roughly 800–1150 scoreable boundaries, about 200–290 per
condition arm — adequate for the clustered binomial model with route-level clustering.

Battery: ~1 point per run, so ~40 runs per charge above the 60% cut. **96 runs ≈ 3 charges of pure
running**, realistically **5–8 sessions** once setup, obstacle repositioning, re-baselines and
failed runs are counted. Development runs (the other two realisations per family) are unlimited and
do not consume this budget.

---

## 7. Criteria proposed to fill the paper's placeholders

The supplementary leaves `[[ ]]` for these. Proposed here so they are frozen in advance:

**Stability** (binary, per episode). An episode is stable when it contains no uncontrolled
safety-contract violation, no unresolved forced continuation, no unbounded payload or control
disturbance, and ends in completion, governed defer or governed abort. Operationally: no emergency
stop by the operator, no collision requiring physical intervention, spill below the marker
threshold, and a terminal state that is one of the three permitted.

**Efficiency**, evaluated **only among stable episodes** (permanent stopping can be stable without
demonstrating continuity): time from start to terminal state, with governed defer and governed
abort included at their realised duration and flagged, never excluded — excluding them would reward
early quitting.

**Adjudication.** Spill and arm contact are scored by the operator against a written rubric, because
our instrumentation cannot see them (see §8).

---

## 8. Limitations declared in advance

1. **Parameter tuning on the evaluation geometry.** The `golden-doorcross` configuration was tuned
   on this door. Reserving cells does not undo that. What helps is that it was frozen and dated on
   14 Aug, *before* this design existed, so it is a pre-existing artefact rather than something
   adjusted in view of confirmatory outcomes. **Rule from here on: any further tuning happens only
   on development cells.**
2. **One door, one flat.** Geometry is varied by obstacle placement, which moves the dead region —
   the factor that matters — but it is not the same as independent corridor layouts. No claim of
   generalisation across environments can be made from this benchmark.
3. **`ncol = 0` does not mean clean.** The collision detector runs on odometry and IMU and does not
   see a light arm scrape; on 14 Aug an arrival scored zero collisions and touched the frame. Until
   arm contact is instrumented, a human must watch every crossing and score it.
4. **The META layer has never run on the real robot.** Zero of 300 real runs carry `meta_state`,
   `laser_trust`, `door_contra` or `iface_q`; the 76 runs that do are all simulator runs. Every
   DCC-relevant claim currently rests on the twin.

---

## 9. Open decisions for Renxi

1. Does obstacle-induced geometry variation satisfy the requirement to reserve *corridor layouts*,
   or must it be reported only as reserved *event combinations*? The protocol's wording admits
   both; the claim wording depends on the answer.
2. Are the stability and efficiency definitions in §7 the ones intended for the paper's frozen
   criteria?
3. Given limitation 1, should the confirmatory runs use `golden-doorcross` as-is, or a
   deliberately untuned configuration, accepting a lower absolute success rate in exchange for a
   cleaner claim?

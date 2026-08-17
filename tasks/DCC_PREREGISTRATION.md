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

- **One environment: Renxi's office** — confirmed 17 Aug as both the intended venue and the space
  where every test to date was run. There is no second environment. One door, one map, one set of
  waypoints. See §8.2 for what this does and does not compromise.
- **Obstacles: the environment's own.** Agreed with Renxi (17 Aug): boxes, sofa and door already
  make the space hard, and **no additional obstacles are needed**. A chair and the G1's transport
  crate remain available if a specific contrast is wanted — the chair has thin legs and a seat above
  the scan plane, so it is *hostile to the LiDAR*, while the crate is a large solid and *reliably
  detected* — but the default is to use what is there.
- **Glass and windows.** Raised by the operator and confirmed by Renxi as a real factor: LiDAR does
  not see glass. This is the single best witness available to us and it is discussed in §2.1.
- **Lighting switchable at any moment**, including mid-run. This is what makes the illumination
  role measurable at all.
- **Battery**: measured over 83 real runs, **~1 percentage point per run**, median run 87 s.

### 1.1 Glass: the paper's aliasing example, physically present

The supplementary's canonical LiDAR witness is that *"a missing lidar return could represent either
free space or a coverage-limited region"*. **A window is that sentence made physical**: glass
returns nothing, so a pane and an open doorway are the same reading. Renxi's comment — *"they are
the reasons that rgb camera is useful, but not reliable"* — states the complementarity exactly:

| Condition | LiDAR | RGB |
|---|---|---|
| Glass / window | **fails** (no return, reads as free) | usable |
| Darkness | usable | **fails** |
| Both | fails | fails → the protocol's *joint insufficiency* family |

The two failures are **dissociable**, which is what Semantic Locality demands: degraded coverage
and inadequate illumination must stay distinct cognitive grounds even when both recommend slowing
down. Glass gives us that dissociation without adding any equipment.

### Illumination cannot be measured from the frames we have (corrected 17 Aug)

An earlier draft of this document proposed mean frame luminance (92–116) as the "adequate"
baseline. **That was wrong and is withdrawn.** Measured over 1978 stored frames, mean luminance is
essentially flat across the whole working day — 09:00 → 18:00 gives 105, 103, 109, 107, 105, 104 —
while contrast varies three times as much:

| Statistic | Relative variation (σ/mean) |
|---|---|
| Mean luminance | 0.12 |
| Contrast (frame σ) | **0.37** |
| Grain (edge energy) | 0.19 |

A mean that stays pinned near 105 under every condition is the signature of the camera's
**auto-exposure**, not of constant room lighting. Mean luminance therefore measures the AGC target,
not the illumination.

**Consequence:** the illumination-adequacy interface field cannot be derived from existing data,
and no image statistic should be adopted before it is calibrated. **Step 0 of the illumination work
is a calibration session**: drive the lights through known, declared states, record frames, and
determine which statistic actually tracks them (contrast and grain are the candidates; mean is
not). Until that session exists, "illumination adequacy" would be invented rather than measured.

This does not affect Ω_t, which takes the **switch state** — known to the operator, independent of
the camera.

---

## 2. Factor space

**Fixed by condition (the 2×2 of the protocol):** decision process (temporal incumbent verification
vs distributed meta-resolution) × interface (original `I⁰` vs revised `I¹`) → C1, C2, C3, C4.

**Geometry variants**, built from what the room already contains:

| | Layout | Sensing character |
|---|---|---|
| G0 | Clear approach | Both senses adequate |
| G1 | Narrowed by an existing solid (boxes, sofa) | Reliably detected → *object* ground |
| G2 | LiDAR-hostile element in the approach: **glazing**, or the chair's thin legs | Reads as free space → *coverage* ground |
| G3 | Both, staggered | Two grounds present at once |

G2 has **two mechanisms that must be recorded separately** — glass (no return at all) and thin legs
(returns below the scan plane, intermittent). They fail the LiDAR differently and should not be
pooled.

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
| F2 lidar degradation | G2 | L0 | high | Dead region without a confirmed object. **Two mechanisms, recorded apart: F2a glazing, F2b thin legs** |
| F3 illumination | G0 | L1 | high | RGB unusable, LiDAR healthy |
| F4 joint insufficiency | G2 | L1 | high | Glazing *and* darkness: neither sense suffices → review/defer, **never a forced object conclusion** |
| F5 reliable object | G1 | L0 | high | Existing solid (boxes/sofa): well detected |
| F6 low battery | G0 | L0 | low | Energy limits available capability |
| F7 recovery and return | G2 | L2 | high | Light returns *and* the obstacle is removed mid-run |
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
   on this door. It was frozen and dated on 14 Aug, *before* this design existed, so it is a
   pre-existing artefact rather than something adjusted in view of confirmatory outcomes.
   **Rule from here on: any further tuning happens only on development cells.**
2. **One environment (§8.2).** No claim of generalisation across environments can be made.
   Geometry is varied by obstacle placement, which moves the dead region — the factor that matters —
   but that is not the same as independent corridor layouts.
3. **`ncol = 0` does not mean clean.** The collision detector runs on odometry and IMU and does not
   see a light arm scrape; on 14 Aug an arrival scored zero collisions and touched the frame. Until
   arm contact is instrumented, a human must watch every crossing and score it.
4. **The META layer has never run on the real robot.** Zero of 300 real runs carry `meta_state`,
   `laser_trust`, `door_contra` or `iface_q`; the 76 runs that do are all simulator runs. Every
   DCC-relevant claim currently rests on the twin.

---

### 8.2 What a single environment does and does not compromise

The venue is Renxi's office, and it is also where everything so far was developed and tuned. That
is a real limitation, but it is narrower than it first appears, and the distinction matters enough
to state precisely.

**What it does not compromise: the four contrasts.** C4−C3, C4−C2, C3−C1 and C4−C1 are all
*within-environment* comparisons. None of them requires generalisation to another space; they
require that the four conditions face the *same* challenges, which a single environment guarantees
by construction. And `golden-doorcross` is the **object-level** controller — it sits below the meta
layer and is **identical in all four conditions**. A constant shared by every arm cannot bias the
difference between arms. The tuning therefore contaminates *absolute* performance, not the
contrasts the paper actually reports.

**What it does compromise:**

- Any claim that these success rates transfer to another building. None should be made.
- The absolute stability and efficiency numbers, which are specific to this office and this door.
- **The meta layer, if it is developed on confirmatory cells.** This is the live risk, and it is the
  one thing reservation genuinely protects. The object-level controller is already frozen; the DCC
  machinery is not yet written.

**Operational rule that follows.** Freeze the object-level controller at `golden-doorcross` and do
**all** meta-layer development — role resolution, C1's verifier, the interface fields, the voxel
memory fix — on development cells only. The reserved cells are then untouched by the layer actually
under test, which is what the protocol is protecting against. Stated plainly in the paper, this is
a defensible position for a single-environment benchmark.

---

## 9. Open decisions for Renxi

1. ~~Which environment?~~ **Settled 17 Aug: Renxi's office, the same space used throughout.**
   Single environment; the consequences are worked through in §8.2 and the operational rule that
   follows from it.
2. Does obstacle-induced geometry variation satisfy the requirement to reserve *corridor layouts*,
   or must it be reported only as reserved *event combinations*? The protocol's wording admits
   both; the claim wording depends on the answer. (Moot if the answer to 1 is two environments.)
3. Are the stability and efficiency definitions in §7 the ones intended for the paper's frozen
   criteria?
4. Given limitation 1, should the confirmatory runs use `golden-doorcross` as-is, or a
   deliberately untuned configuration, accepting a lower absolute success rate in exchange for a
   cleaner claim?

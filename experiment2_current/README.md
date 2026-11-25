# Experiment 2 · current design and status

**As of 10 November 2026.** This folder holds the current design of the robotic
experiment and what has been collected under it. The rest of this repository
documents the earlier C1–C4 design on the Unitree G1 and its digital twin; that
material is kept as it was and is not evidence for the design described here.

> [!NOTE]
> **Everything here is development data.** No record has been classified under
> C2, C2.5 or C3, and nothing has been independently annotated. No comparative
> result exists yet.

## What the experiment asks

Whether a participation concept grounded in one deployment context can be
reconstructed, recalibrated and reused after a change of physical context and of
robot embodiment:

```
simulation  →  physical Summit XL  →  physical G1
```

The concept under investigation is **directional pre-entry compatibility**: the
functional relationship among doorway geometry, robot passage envelope and
posture, stop distance, lateral offset, heading, proposed motion and sensing
validity that determines whether the crossing capability is sufficiently
supported for participation.

The crossing routine is only the operational means of reaching an aligned
decision point. The experiment begins at that point. **Crossing success is not
the endpoint**, and motion-routine performance is not concept-reconstruction
performance.

## The three conditions

| Condition | What it does | Returns |
|---|---|---|
| **C2** | Direct LLM judgement from the available record | a supported or unsupported direct answer |
| **C2.5** | Classifies whether the record lies within the applicable contextual span. Read from the C3 processing record before the additional layers: a dependent intermediate output, not an independent run | within span · boundary · outside span · indeterminate |
| **C3** | Reconstructs the concept in the changed context, with local meaning, unresolved conditions, capability participation and authority | participate · review · defer · no-use · renew |

Action authority is recorded separately from the C3 outcome: *participate* does
not by itself authorise an action. *Renew* is a candidate specification renewal
that must still be grounded, validated and released.

The expected contribution of C3 is not a better classification. It is
reconstructible local meaning, unresolved conditions, governed participation and
authority separation beyond C2's direct judgement and C2.5's contextual-span
classification.

## Two parallel tracks

| Track | Tests | Depends on the other |
|---|---|---|
| **Summit XL** | the concept under a simulation-to-physical context change within one embodiment, on matched simulated and physical records | no |
| **G1** | whether the abstract concept can organise and explain observations under a different embodiment. Imports no Summit-specific thresholds, classifications, clearances, sensing assumptions or authority rules | no |

## Staged sequence

| Stage | Summit XL | G1 |
|---|---|---|
| 1. Development characterisation | done, 10 Nov | not done: no crossing completed |
| 2. Frozen-record validation | three records pass the manifest | frozen record not built |
| 3. Controlled pre-test (selected concept-boundary cases) | not started | not started |
| 4. Concept release / lock | not started | not started |
| 5. Locked evaluation on new records | not started | not started |

## Status of the physical Summit XL, 10 November 2026

Across six physical Summit XL attempts, three manifest-valid frozen records were
acquired at the aligned pre-entry point. The three valid records demonstrate
repeatable, non-interfering acquisition for one nominal scenario and establish
the evidence base for a within-record C2/C2.5/C3 comparison. No claim about
relative reconstruction performance can yet be made because the records have not
been independently annotated or classified.

| Attempted | Aligned | Frozen | Pass the manifest | Classified | Annotated |
|---|---|---|---|---|---|
| 6 | 3 | 3 | 3 | 0 | 0 |

The records are in [`summit_xl_physical_2026-11-10/`](summit_xl_physical_2026-11-10/).

### Known limits of these records

- **One nominal scenario:** same start, same door, same direction, same aligned
  pose. Room as declared by the operator, not measured: blinds down, all lights on.
- **«Robot stationary» is declared** by the traversal script, which stops the
  base and requests a fresh pose before writing the alignment mark. It is not measured.
- **One manifest check was changed after the first valid record was seen:** the
  format of the alignment mark it reads. The requirement did not change.
- **Pose and scan share one timestamp** by construction.
- **The record holds no lighting or blind-spot field**, only the laser transform
  rather than the full transform tree, no 3D cloud, no camera and no continuous
  recording.
- **Nothing has been independently checked.** «Valid» means «passes the manifest».

### A simulation-to-physical difference found during acquisition

The shadow process that freezes the record carries a safety node inherited from
simulation that treats a scan older than 0.5 s as stale. On the physical robot
the scan arrived older than that, and the node latched (`SCAN_CADUCADO`). The
limit was not changed. The latch is now stored in the record as a
perception-validity observation instead of closing the shadow before the cut.

## Status of the G1, 10 November 2026

No crossing was completed. The station sends motion commands through the data
channel held by the vendor app's page, and it had not taken hold of that channel;
two attempts were aborted with the robot stationary. The cause was measured and a
fix is prepared but not yet exercised. The G1 does not yet freeze a record at its
aligned point, so no G1 data can enter a C2/C2.5/C3 comparison.

The G1 plan is [`G1_Experiment2_test_plan_v0.3_2026-11-10.pdf`](G1_Experiment2_test_plan_v0.3_2026-11-10.pdf).

## What does not exist yet

- Matched simulated records: simulation does not yet cut at the aligned point.
- C2 and C3 processing of the three physical records, with the output sets above.
- Independent annotation.
- Any case other than the nominal one.
- The G1 frozen record and the G1 development crossings.

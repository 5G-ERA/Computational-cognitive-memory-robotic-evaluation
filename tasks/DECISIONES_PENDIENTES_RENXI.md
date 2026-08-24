# Decisions parked for Renxi — with the evidence attached

**Context:** Renxi is away until early September. Rule adopted meanwhile: no decision of his
is taken for him; each one is built out **both ways behind a declared switch**, with a
provisional default chosen and labelled, so resolving it on return is a config change, not
work. The confirmatory tier does not run in his absence: the twelve reserved configurations
stay untouched and the §12 model stays unfrozen. Everything below is development.

---

## D1 · VOXMEM: interface field, or actor?

**The question.** The voxel memory currently does `confirmed |= _keep`: historical lidar
*acts* on the planner. In the protocol it is a field of I¹, and §5.3 says sensor confidence
must not become control authority automatically. If it injects unconditionally, C1 and C3
silently receive historical lidar and the interface factor stops being separable from the
process factor.

**Provisional default: expose-only** (`G1_VOXMEM_ACT=0`). The memory is computed and emitted
per sample (`vox_inj`, `vox_ray`) but does not touch the planner. Injection — the safety
behaviour — is one env var away (`G1_VOXMEM_ACT=1`), and the 45-run campaign that validated
the ray-clearing mechanism ran *with* injection, so that evidence covers the acting branch.

**What Renxi decides:** whether the confirmatory tier runs expose-only (ablation measures
the *information*) or acting-for-all-conditions (object-level constant; ablation changes
meaning). Evidence: commits eebf802, 21a492e.

## D2 · T3/T4 are inverted for this system

**The question.** The protocol table assumes low light degrades RGB. Our frozen visual
contract measures the opposite: full light is what makes the RGB door bearing inadmissible
(+9° bias, door strike). The staged "T3" therefore exercises the T4 direction.

**Provisional handling: stage both directions, label which is which.** `guion.py` carries
the measured direction (light-on → illumination) and the classical direction as separate
scripts; development scoring reports them separately. No renumbering of the reserved table
is attempted — that is his call.

**What Renxi decides:** redefine T3/T4 against the measured phenomenon, or add a
genuinely-dark third state. Evidence: tasks/VISUAL_QUALITY_CONTRACT.md, commit 7b37d55.

## D3 · T7: the battery band collides with the stopping rule

**The question.** Confirmatory runs are cut at 60% (§4) and the energy role fires below 60,
so the transition coincides with the stopping rule and is never observable in a valid run.

**Provisional handling in the twin: declared battery trajectory.** The twin has no battery
model and will not get a fake one; but battery is a *declared meta-variable read from
telemetry*, exactly like the light switch — so the staging channel can script `bat(t)` and
the record of the change is the independent record, same §3 argument as light. This makes
T7 twin-stageable for development without pretending to model electrochemistry. The real
robot's band question (move to 70%, or a separate floor) stays untouched.

**What Renxi decides:** the real-tier band. Evidence: tasks/T1_T12_STAGING.md §conflicts.

## D4 · The reference-map / loc_match decision (a) — already taken, flagged for review

Taken on 24 Aug with Adrián (option a: declare loc_match non-comparable in the twin) because
it was measurably cheap (0.3 pp on C1) — recorded here so Renxi sees it rather than finds
it. Reopening condition is written in the commit: if localisation-degradation episodes are
ever wanted, this reopens. Evidence: commits 554af3e, 84ac759.

---

*Standing rule while he is away: anything below this line gets added, never resolved.*

## D5 · T11: in this system the representative swap IS a role

T11 expects "role identity preserved under representative change". But our implementation of
the swap is the illumination gate: illumination is the ROLE that decides which representative
of the door centre governs (W3 in production). Staging the swap (light change) therefore
changes the resolved role to illumination by design, and a certificate demanding "motion
unchanged" scores the correct behaviour as failure. Provisionally T11's second segment
expects illumination, and "identity preserved" is read behaviourally (the crossing goal is
unchanged). Whether T11 should be redefined for role-based swap implementations is his call.
Evidence: commit f838373 (the gate as role), tasks/VISUAL_QUALITY_CONTRACT.md.

## D6 · T10 measures a real gap: no_use is unreachable autonomously

First staged T10 (blocked doorway, chair identifiable): the robot spends 194 samples at the
blockage, never crosses, never resolves no_use -- it retries until timeout (DWA-F at the
end). No condition reaches the governed no-use outcome; the only path to no_use in the stack
is human ASSIST or a fault. This is the no-use control doing its job: the architecture has
no mechanism that accumulates sustained non-progress against a discovered blockage into a
no-use ground. Building one touches role semantics and §5 stability (governed abort), so it
is a design decision, not a patch. Until then T10 scores ~0 for every condition, honestly.


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

## D7 · Should the sim harness become strictly odometry-driven?

The last timing discrepancy left in the twin: odometry publishes at 3.2 Hz while the control
tick runs at 3.3 Hz, so 17.9% of consecutive recorded samples carry an identical pose
against the real robot's 2.7% (down from 68.7% before the interface-latency fix). Closing
it means making the harness tick on odometry arrival instead of sleeping a fixed interval --
shared control code, and decision-boundary rate is a meta-variable of the benchmark, so the
change is his to approve. This question was raised in the readiness report on 23 Aug and
fell out of the decisions list during a rewrite; restored here so it is not lost twice.
Evidence: commit bd9e9ea.

## D6 · T10 measures a real gap: no_use is unreachable autonomously

First staged T10 (blocked doorway, chair identifiable): the robot spends 194 samples at the
blockage, never crosses, never resolves no_use -- it retries until timeout (DWA-F at the
end). No condition reaches the governed no-use outcome; the only path to no_use in the stack
is human ASSIST or a fault. This is the no-use control doing its job: the architecture has
no mechanism that accumulates sustained non-progress against a discovered blockage into a
no-use ground. Building one touches role semantics and §5 stability (governed abort), so it
is a design decision, not a patch. Until then T10 scores ~0 for every condition, honestly.

## D8 * The glass witness: sizing spec, and which instrument can actually see it

Three measured findings (25-Aug), each with the run that produced it:

**1. Sizing.** The original staged pane (~1 m2, rect -4.3,0.6,-3.5,1.9) is below the
witness's resolution: the 24-Aug T1/T2 runs show no in-zone cov_def response at all. A
~3.6 m2 pane covering ~55 mapped cells (rect -2.6,0.8,-0.8,2.8; run 20260825_183008,
arm GLASS_D8) yields in-zone cov_def 0.32 median / 0.44 p75 -- bracketing the real
glass signature (0.44/0.32). Working spec: **a glass witness needs >= ~2 m2 of mapped
wall**. guion.py now stages the D8 pane.

**2. Global cov_def is NOT a usable ground.** Whole-run cov_def (vs the historical
refmap) has a 38-45% base rate above any threshold that would catch glass, in the twin
with no glass staged (today's clean legs: median 0.20; 24-Aug: median 0.25) -- and it
saturates on the real robot (median 1.00, the known stale-map ray-march defect). EMA
smoothing does not separate either (33% vs 49% at 0.30). A resolver ground on global
cov_def would fire constantly. The discriminative signal is *spatial* -- in-zone vs
out-of-zone -- and the resolver cannot condition on the staged zone (circular). The
sector-persistent statistic **cov_missing (session reference, 2 consecutive sweeps)**
is the instrument with the right shape, and it is already the resolver's lidar ground
(>= 3). No new ground is added; nothing about the resolver changes.

**3. The instrument was never wired where it mattered.** cov_missing only exists when
G1_COVREF is set -- and no glass run ever had it (not GLASS_D8, not the T1/T2/T8
stagings, not the campaigns). Worse: a session reference built from route legs alone
has **0 cells inside the glass rect** at every ratio tried (0.50-0.90), because the
cov sector is +-40 deg forward and the route never faces that wall -- such a reference
can never witness the glass. Fixes applied: guion.py exports G1_COVREF for ALL
configurations when the session reference exists (the observe-vs-act lesson: the
instrument must not vary with the staging), tools/mapa_visibilidad.py gained --excluye
to keep staged-degradation legs out of the reference, and the Thursday runbook Block A
now requires one calibration pass FACING the glass wall plus a rect-count check before
the reference is declared. Pending (needs the bridge, queued behind the variance
campaign): a twin calibration pass facing the wall, reference rebuild, then a GLASS_D8
leg with G1_COVREF live to measure the cov_missing separation, then rescore T1/T2/T8.

**VALIDATED later the same night (25-Aug, second pass).** The full chain now works in
the twin, with three more findings on the way:

- The D8 pane's wall is unreliably seen by the twin's own app-filter model (presence
  ratio <= 0.30 -> 0 session-reference cells at any ratio). Sizing is necessary but NOT
  sufficient: **the pane must cover reliably-seen wall** (ratio >= 0.65). The validated
  pane is -4.0,1.5,-2.7,3.0 (14 reference cells, NE door flank); guion.py stages it.
- cov_missing needed the **accumulated base** (union of the last PERSIST_N sweeps +
  current, exact matching): instantaneous-exact ran at the noise floor (40% vs 32%),
  the v2 3x3 neighborhood swallowed the glass entirely (0% vs 0%), accumulated-exact
  separates (5-tick streak >= 3 in the glass approach, control max 2 with ZERO >= 3).
  Default since 25-Aug (G1_COVM_V2=0 restores the old variant).
- The certificate needed a **facing gate**: delta demands lidar_quality/defer only when
  the declared pane is inside the instrument's declared window (<= COV_R, +-40 deg) --
  before it, T8's C4 answered illumination (correctly!) on in-zone-not-facing samples
  and the ablations outscored it by forced ignorance. After: T8 C4 88% vs C2 4 / C3 23.
  Campaign rescored: C4-C3 +54 pp [38,61] 30/31, C4-C2 +57 pp [54,70] 30/31; C4-C1
  is flat on time-averaged A_meta/A_Omega and lives at transitions instead --
  secondaries: C4 adopts 35/51 reference transitions vs C1's 22/51 (C1 misses 57%),
  C4 cost = 7.7 unnecessary switches/min pure-resolver (stabiliser damps in flight).

For Renxi: (a) confirm the ~2 m2 sizing spec as the staging norm for W1; (b) confirm
cov_missing-with-session-reference as the declared glass instrument (and the historical
refmap cov_def as diagnostic only); (c) the real tier needs the facing pass in every
session's Block A or W1 is silently dead for that session.

## D9 * Rung 7: the PhysX walker inside the benchmark loop (proposal ready, not started)

The walker (policy g1_full, 23-Aug) and the kinematic twin both work, separately.
Putting the walker under g1_goto is the last rung of the realism ladder and would make
motion texture emergent (P4 sway, T10 physical blockage, footstep pose noise) -- but it
breaks the calibration lineage (VSCALE/TAU fitted to the kinematic channel), reopens
the just-paid N=30 dispersion debt on a new tier, and introduces a fallen-robot outcome
class the scoring has no category for. Full spec with phased migration and kill
criteria: `tasks/RUNG7_WALKER_PROPOSAL.md`. Decision: whether phase 0 (a pure
measurement bench) runs at all, and if adopted, which tier the confirmatory campaign
declares. Nothing has been started.

# tasks/ — what to do next

Pending work, newest first. **The next thing that happens in the lab is R1.**

---

## ✅ DONE — R1 re-baseline (13 Aug 2026): **Gate 1 PASSES**

30 real runs on the golden binary, dry batch, driven from the lab machine. Read by battery bucket:

| Battery | Runs | Reached | A→B | B→A | Median time |
|---|---|---|---|---|---|
| **≥60%** | 13 | **12 (92%)** | **7/7** | **5/6** | 65 s |
| <50% | 12 | 5 (42%) | 4/6 | 1/6 | 105 s |

**The baseline reproduces when the robot is charged.** Read as one lump the batch looks mediocre;
read by battery it is a clean pass plus an uncontrolled variable. Five deliberate perturbation runs
(displaced up to 2.07 m, facing up to 230° off nominal) **all arrived**, and none of the nine
failures was a perturbed run. Data on branch `r1-session-2026-08-13`.

**New hard rule — battery.** Start ≥80%, stop the batch at 60%, recharge, resume, and log battery
per run. At the same commanded speed the robot walks 11% slower below 50%, and the meta-reasoner
detects it without being told (tension ×2.75, fulfilment ÷5) while sensing reliability stays flat —
the degradation is in mobility, not perception.

## ▶ NEXT LAB SESSION — door centring A/B

**Take with you:** [`G1_Session_Plan_Next.pdf`](G1_Session_Plan_Next.pdf) — the plan for this
session. The [Operator Runbook](G1_Test_Protocol_Operator_Runbook.pdf) still holds for the ritual.

**Code:** branch **`golden-plus-centring`** — golden with exactly one change (two hard-coded gates
turned into variables; defaults reproduce golden). Both arms run the same binary.

**What it tests.** On B→A every run arrives ~0.35 m off the door axis to the robot's left; clean
crossings re-centre to ~0.01 m, failures arrive still at 0.19 m, and the physical margin is only
±0.20 m. The strafe-to-axis servo already existed but only fires above 0.14 m of error and stops
correcting 0.35 m before the opening.

- **base** — no variables (golden behaviour)
- **tight** — `G1_DOOR_CTR_TOL=0.07 G1_DOOR_CTR_S=0.18`

Twelve B→A runs, alternating, battery ≥60%. **Primary metric: lateral offset at the threshold**, not
arrivals — with n=6 per arm the binary outcome cannot resolve the difference, the offset can.

**Twin evidence:** base 5/6 with 3 collisions and |offset| 0.069 m; **tight 6/6, zero collisions,
|offset| 0.012 m**, no time cost, distributions with no overlap.

**Optional if the day allows (R1b):** run four more below 45%, recharge above 80%, run four more.
If performance recovers, battery is causal; if not, the degradation is cumulative session effect.

### Rejected — do not retry without new evidence
- **"Trust vision more to centre itself."** Over 2481 real samples the door bearing correlates
  +0.51 with the true geometric bearing but **−0.00 with the lateral offset within 1.2 m**. Bearing
  says where to point, not where you are.
- **Approach bias (`Door_BiasPlus`, +0.12 m).** Twin A/B: it made centring *worse* (|offset| 0.064
  vs 0.030). Its apparent edge in arrivals sat inside the n=6 noise floor — two identical
  configurations differed by just as much.

## ▷ IN PROGRESS — voxel memory in the blind band (`feature/voxel-memory`)

> **14 Aug, implemented and calibrated against real data; twin A/B running.** Flag `G1_VOXMEM`,
> **off by default** — defaults reproduce the previous behaviour exactly. The offline safety
> replay below chose the TTL; it was not picked by eye. Not on the robot until the twin agrees.

Renxi's suggestion (14 Aug): [spatio_temporal_voxel_layer](https://github.com/SteveMacenski/spatio_temporal_voxel_layer)
— *"the robot is helpless if it is in the blind spots"*, and *"there is a setting to pay more
attention to voxels observed a few seconds ago"*. His diagnosis is exactly our failure mode, and
the data now puts a number on his "few seconds".

**Evidence.** Of 193 real collisions, **107 (55%)** happened with the laser reporting clear beyond
0.6 m. Isolating those where the laser *had* seen the obstacle and then lost it: **29 cases, median
2.2 s of blindness before impact** (p90 4.1 s). A traverse on 13 Aug shows it cleanly — clearance
0.62 → 0.98 → 1.59 → **1.67 m at the moment of impact**: the obstacle faded from the scan as the
robot closed in, and the grid erased it in the same tick.

**Where the gap is in our code.** `g1_goto.py` has a persistence filter, but only in the
*confirming* direction (a cell must appear in ≥2 of the last 3 scans before it is trusted —
anti-noise). The update loop iterates over the **current scan only**, so a cell that stops being
observed vanishes immediately. There is no retention. That is precisely what STVL adds.

**The package itself cannot be dropped in**: the G1 "Air" exposes no ROS and no DDS, so there is no
Nav2 and no costmap_2d here — the obstacle grid is ours, built from the cloud read out of the
vendor app's WebView. The *mechanism* is small and belongs in that grid.

**Design (targeted, not blanket).** Retain a cell's confirmation **only while it sits inside the
vendor blind band, after having been confirmed outside it** — that is exactly the region where
absence of evidence is not evidence of absence, and it avoids trusting stale cells in open space.
Measured starting point for the decay window: **3 s covers 69% of those collisions, 4 s covers 90%**.

**Safety caution from our own history.** Naive retention is dangerous here: in July a bug that froze
noisy door-mouth cells produced phantom obstacles and a nine-collision loop — the twin caught it.
STVL solves this with ray-traced clearing; our version needs an equivalent (a cell is cleared when
a ray demonstrably passes through it, or when the TTL expires), plus the usual env-gated default
that reproduces current behaviour exactly.

**Validation path.** Offline replay first, over every recorded collision (does it mark the obstacle
before impact?) and over the clean runs (does it invent obstacles?) — `analysis/replay_msm.py` is
the pattern. Then the twin, then the robot. Own branch, one change.

### What was built, and how the TTL was chosen

A cell confirmed at a *healthy* range is remembered for `G1_VOXMEM_TTL` seconds and re-injected
into the grid while it lies within `G1_VOXMEM_R` (1.2 m) of the robot — the band the vendor cuts.
Outside that radius the scan rules, so memory never contradicts a good observation; and if the scan
sees the cell again, memory stays out of the way. Retention requires `G1_VOXMEM_K` (2) prior
confirmations **from outside the blind band**, so a cell that only ever flickered up close can
never qualify. `G1_VOXMEM_MAX` (400) caps the set, keeping the most recent.

The TTL came from a replay over the **244 clouds saved at real collisions**. For each, the cells
memory would hold were projected against the trajectory the robot actually walked in the following
seconds, counting any that would have invaded the DWA clearance (0.22 m) — i.e. phantom obstacles
blocking ground the robot demonstrably crossed:

| TTL | Cells held per instant | Snapshots with a phantom |
|---|---|---|
| 2 s | 27.7 | 0 / 244 |
| **3 s** | **28.6** | **0 / 244** |
| 4 s | 29.4 | 2 / 244 (1%) |
| 6 s | 30.7 | 8 / 244 (3%) |

3 s is the longest window with zero measured phantoms, and it is what the default sits at. Going to
4 s would buy coverage (69% → 90% of the blindness-preceded collisions) at the price of the first
phantoms — a trade to revisit *after* the twin, with evidence, not before.

Twin A/B: `campaigns/sim_ab_voxmem.py` — 12 interleaved runs under calibrated noise, full META
stack in both arms, `G1_VOXMEM` the only variable. Per-sample field `vox_inj` records how many
cells memory is holding, so the mechanism can be audited run by run rather than inferred.

## ▷ Open engineering items

| Item | Why it matters |
|---|---|
| Move the twin workspace out of `~/Downloads` | The live simulator depends on a folder that gets cleaned; see [`../sim/RUN_AND_REBUILD.md`](../sim/RUN_AND_REBUILD.md) |
| Build the twin Docker recipe once and verify it | The recipe is versioned but has never been rebuilt from scratch |
| Re-record the B waypoint (or census the furniture there) | It sits ~40 cm from the sofa: the root cause of the start-pocket wedges. **Reinforced 13 Aug**: three twin failures entered the door *well centred* and got stuck after it — a second failure mode, independent of the centring one, and physical rather than a control parameter |
| Exercise DEGRADED on the real robot | Never reached in real data — it needs three missed-collision reports in ~30 s |
| Latch the ASSIST state | It can currently drop out if clearance rises during an alignment dance |
| Camera-led blind mode with a static plan | The last item of the supervisor's specification still unimplemented |

---

*Everything here is tracked in the documents it links to; this page is only the entry point.*

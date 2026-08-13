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

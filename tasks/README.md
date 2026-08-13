# tasks/ — what to do next

Pending work, newest first. **The next thing that happens in the lab is R1.**

---

## ▶ NEXT LAB SESSION — R1 re-baseline

**Take with you:** [`G1_R1_Real_Robot_Runbook.pdf`](G1_R1_Real_Robot_Runbook.pdf) — **the document
for this session** (6 pages: the choices already made, pre-flight, bring-up, shakedown, the run loop
and Gate 1). [`G1_R1_Session_Checklist.pdf`](G1_R1_Session_Checklist.pdf) is the 2-page summary of the
same thing; the [Operator Runbook](G1_Test_Protocol_Operator_Runbook.pdf) covers both lanes and is the
reference for the twin.

**Goal:** re-establish the baseline on the frozen golden code, so that everything measured
afterwards has something honest to be compared against.

| | |
|---|---|
| **Machine** | The **lab machine**, not the laptop: the golden runs were driven from there (perception on `127.0.0.1`), and the datasets and USB proxy live there |
| **Code** | `git checkout golden-doorvis` — the frozen tag, nothing else |
| **Runs** | 10, alternating direction (5 + 5) |
| **Payload** | **Dry** — empty cup in the hand. Water at 200 g is the *next* batch on the same binary (Disciplined Protocol). An earlier revision of this page said 247 g of water: that was wrong |
| **Marker** | Started before the first run, heartbeat visible. On golden it is the version *without* the `c` / `m` keys |
| **Stop rule** | 3 consecutive failures of the same mode → stop the session, analyse offline |
| **Rule** | No code edits in the lab. None |

**Gate 1 — decide it at the lab, before packing up:**

| Check | Pass |
|---|---|
| Easy direction (A→B) reached | ≥ 90 % |
| Hard direction (B→A) reached | ≥ 70 % |
| Collisions | median 0 per run |
| Times | golden band, ~50–110 s |

- **Gate 1 passes** → continue with the acquisition plan, *or* insert the new-stack session next.
- **Gate 1 fails** → data home, autopsy with the analysis tools. Change nothing until the cause
  has a name.

**Log sync: DONE (13 Aug 2026).** The lab machine was four commits behind with a 48 MB
`goto.log`; it now sits on the current `main` with the rotated log (its old copy archived
locally and preserved in git history). Nothing to do before the session on that front.

**New check before the session:** the lab machine also hosts a local LLM service that can hold
~20 GB of VRAM on *each* GPU, which starves the depth model. Run `nvidia-smi` and free the
cards before launching the perception server — not after the cup is full.

---

## ▷ AFTER R1 — real trial of the new stack

Same 10-run pattern as R1, so the comparison is 1:1, but on the branch that carries the meta
state machine. Practise the two operator keys: report a collision the robot missed, and tell it
you helped when it stops and asks. Details and the argument for it:
[`../docs/G1_Branch_Strategy_and_New_Stack_Case.pdf`](../docs/G1_Branch_Strategy_and_New_Stack_Case.pdf).

---

## ▶ READY FOR THE NEXT REAL SESSION — door centring

Adrián's observation on 13 Aug ("on the way back it always drifts a little to one side before the
door frame and puts itself in a worse position") is **confirmed and fixed in the twin**.

- **What it is:** on B→A every run arrives ~0.35 m off the door axis at 2 m out. Runs that cross
  cleanly re-centre to ~0.01 m; runs that fail arrive still 0.19 m off — and the physical margin
  is only ±0.20 m (0.99 m opening, 0.29 m effective half-width).
- **Cause:** the lateral strafe-to-axis servo already existed but was gated to fire only above
  0.14 m of error and to stop correcting 0.35 m before the opening.
- **Change:** both gates parameterised — `G1_DOOR_CTR_TOL` and `G1_DOOR_CTR_S`, **defaults
  identical to the old behaviour**.
- **Twin evidence (12 interleaved B→A runs, noise + camera + DOOR-VIS):** 0.14/0.35 → 5/6, three
  collisions, |offset| 0.069 m; **0.07/0.18 → 6/6, zero collisions, |offset| 0.012 m**, no time
  cost. The two distributions do not overlap.
- **Ruled out on the way:** "trust vision more to centre itself" cannot work — the door bearing
  correlates +0.51 with the true geometric bearing but **−0.00 with the lateral offset** at close
  range. Bearing says where to point, not where you are.

**How to run it on the robot:** same golden command plus `G1_DOOR_CTR_TOL=0.07 G1_DOOR_CTR_S=0.18`,
as an A/B against the unmodified line — one change, alternating arms, on a **charged** battery.
Honest caveat: the twin's baseline offset (0.069 m) is milder than the real one (0.19 m), so the
twin proves the mechanism, not that it prevents the real collisions.

## ▷ Open engineering items

| Item | Why it matters |
|---|---|
| Move the twin workspace out of `~/Downloads` | The live simulator depends on a folder that gets cleaned; see [`../sim/RUN_AND_REBUILD.md`](../sim/RUN_AND_REBUILD.md) |
| Build the twin Docker recipe once and verify it | The recipe is versioned but has never been rebuilt from scratch |
| Re-record the B waypoint (or census the furniture there) | It sits ~40 cm from the sofa: the root cause of the start-pocket wedges, in both the real flat and the twin |
| Exercise DEGRADED on the real robot | Never reached in real data — it needs three missed-collision reports in ~30 s |
| Latch the ASSIST state | It can currently drop out if clearance rises during an alignment dance |
| Camera-led blind mode with a static plan | The last item of the supervisor's specification still unimplemented |

---

*Everything here is tracked in the documents it links to; this page is only the entry point.*

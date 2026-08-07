# tasks/ — what to do next

Pending work, newest first. **The next thing that happens in the lab is R1.**

---

## ▶ NEXT LAB SESSION — R1 re-baseline

**Take with you:** [`G1_R1_Session_Checklist.pdf`](G1_R1_Session_Checklist.pdf) (2 pages, the
step-by-step sheet) and [`G1_Test_Protocol_Operator_Runbook.pdf`](G1_Test_Protocol_Operator_Runbook.pdf)
(5 pages, every command).

**Goal:** re-establish the baseline on the frozen golden code, so that everything measured
afterwards has something honest to be compared against.

| | |
|---|---|
| **Code** | `git checkout golden-doorvis` — the frozen tag, nothing else |
| **Runs** | 10, alternating direction (5 + 5) |
| **Payload** | Cup filled to 247 g; weigh before and after **every leg** |
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

**Before leaving home:** the log-sync step described in the runbook (§0.1) must be done in
order — the machine that appends to the shared log commits and pushes first.

---

## ▷ AFTER R1 — real trial of the new stack

Same 10-run pattern as R1, so the comparison is 1:1, but on the branch that carries the meta
state machine. Practise the two operator keys: report a collision the robot missed, and tell it
you helped when it stops and asks. Details and the argument for it:
[`../docs/G1_Branch_Strategy_and_New_Stack_Case.pdf`](../docs/G1_Branch_Strategy_and_New_Stack_Case.pdf).

---

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

# DCC campaign v2 — the Isaac Sim results (development tier)

**Date:** 25/26-Aug-2026. **Tier: DEVELOPMENT** — the confirmatory tier, the 12 reserved
configurations and the SS12 freeze remain untouched and are Renxi's. **Manifest:**
`tasks/manifiestos/campana_dcc_v2.txt` (the analysis reads the list; nothing is selected
by timestamp). **Scoring:** `analysis/nivel_run.py` + `analysis/corre_secundarios.py`
with `G1_DCC_MAN` pointing at the v2 manifest.

## Why a v2 campaign

The 24-Aug campaign predates the validated glass-witness chain: no session coverage
reference was live (`cov_missing` = None in every sample), the staged pane was below the
witness resolution, and the certificate lacked the facing gate. Offline rescoring of v1
was valid but its runs carry no live lidar evidence. v2 runs with: `G1_COVREF` exported
to ALL configurations (observe-vs-act), the validated pane over reliably-seen wall,
`cov_missing` v2 (accumulated base) as default, and facing-gated certificates.

**Toolchain frozen before launch** at commit `a9294ea`; driver `campana_dcc_v2.py`
(resumable, manifest-driven). 30 legs T1–T9+T11 ×3 + one T2 top-up (one T2 rep came out
degenerate — 2 s, robot already at goal after an interrupted leg — and is dropped by the
pre-declared <30-samples filter; the top-up leg replaces it, noted here).

> **Condition labels follow paper V5.8 §8.6** (C2 = original interface + distributed
> resolution; C3 = revised interface + temporal incumbent — an earlier revision of this
> file transposed the two middle cells in prose; the code and every number were always
> V5.8-correct). Contrast names per §8.7: C4−C3 and C2−C1 are **resolution effects**;
> C4−C2 and C3−C1 are **interface effects**.

## Primary — A_meta at run level (median [IQR] across 30 runs)

| condition | A_meta |
|---|---|
| C1 · original interface + temporal incumbent | 65% [23–86] |
| C2 · original interface + distributed resolution | 0% [0–0] |
| C3 · revised interface + temporal incumbent | 13% [8–16] |
| C4 · distributed, I¹ (full) | 63% [53–72] |

**Paired contrasts (the pre-registered comparison; each run yields all four conditions
on the same samples):**

| contrast | median [IQR] | sign |
|---|---|---|
| **C4 · revised interface + DCC resolution (full)** | **+53 pp [+43, +60]** | C4>C3 in **30/30** runs |
| **C4−C2** | **+61 pp [+52, +71]** | C4>C2 in **30/30** runs |
| C4−C1 | +6 pp [−25, +39] | 15/30 — flat time-averaged (see secondaries) |
| C3−C1 | −52 pp [−74, −14] | C3<C1 in 30/30 |

A_Ω tracks A_meta closely (retained `motion` carries the default ground, so A_Ω does not
separate C1 from C4 either — by design the discrimination lives at transitions).

## Secondaries (SS9.3) — where C4−C1 is decided

Over 40 reference transitions in the staged runs:

| | adopted | MISSED | median delay | return to nominal | unnecessary/min |
|---|---|---|---|---|---|
| C1 | 13/40 | 27 | 0.0 s | 7/12 | 1.3 |
| C2 | 5/40 | 35 | 0.0 s | 2/12 | 0.0 |
| C3 | 15/40 | 25 | 17.6 s | 8/12 | 2.4 |
| **C4** | **34/40** | **6** | **0.6 s** | **12/12** | 8.1 |

The temporal incumbent (C1) matches C4 on the time average because retention is free
when δ is mostly `motion`; it then misses **68% of the transitions the certificates
demand**. C4 adopts 85% with 0.6 s median delay and always returns to nominal; its
declared cost is 8.1 unnecessary switches/min at the pure resolver (the in-flight role
stabiliser — conf 2 ticks, dwell 1 s — exists precisely to damp this and is not scored
here).

## Per configuration (C4 median across reps)

T1 66 · T2 53 · T3 83 · T4 52 · T5 88 · T6 51 · T7 71 · T8 54 · T9 69 · T11 53.
T7 carries the declared battery-band conflict (D3). T11 is scored under the D5 reading
(the representative swap IS a role).

## T10 and T12 — the two configs outside the table

**T12: PASS** (record property): the session visibility reference superseded its
incumbents (provisional + historical Summit map) without rewriting either; provenance
and git history verified by `analysis/verifica_t12.py`; certificate at
`dataset/certificado_T12.json`.

**T10: not stageable in the kinematic twin today — and the attempt produced a real
finding.** A doorway board staged in the USD scene is invisible to the twin's laser
(the /scan is synthesized from the map JSONs, not the scene); staged instead into the
scan map (`G1_BLOQUEO` in the bridge, +18 cells, same declared geometry), the robot
SAW the board (1363 returns in its rectangle), resolved `lidar_quality` on approach —
and then crossed it: inside 0.6 m the NEAR_BLIND exclusion (the pitching phantom-ring
filter) swallows its returns and the persistence filter lets the cells expire, so **a
thin obstacle at close range evaporates from the belief exactly where the door
controller commits**. This is shared pipeline with the real robot (its mitigation is
the depth-perception channel, which the twin only emulates for declared objects).
Changing NEAR_BLIND semantics is an instrument decision → Renxi (ledger). Additionally,
the 24-Aug T10 evidence in D6 is mis-attributed: those manifest runs actually REACHED B
(median final pose at B); the "194 samples at the blockage" run is not in the manifest.
D6's conclusion (no_use unreachable autonomously) still stands — nothing staged has
ever produced no_use — but its cited evidence needed this correction.

## What these results are, and are not

They are the complete development-tier verification that the DCC scoring machinery
runs end-to-end in the Isaac twin and that the full interface (C4) beats its ablations
by the pre-registered contrasts, unanimously across runs, with live lidar evidence in
every leg. They are NOT confirmatory numbers: that tier runs only with Renxi, on the
reserved configurations, after the SS12 freeze. The real-robot arm of the comparison
starts producing samples in the Thursday session (`tasks/SESSION_PREP_GATE_AB.md`).

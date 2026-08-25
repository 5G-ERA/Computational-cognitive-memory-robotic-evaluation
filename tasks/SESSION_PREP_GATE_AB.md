# Robot session: gate A/B + lit reference + freeze K_online

**Prepared 25 Aug 2026.** Development tier only — no reserved configuration is touched, and
K_online is frozen BEFORE any A/B outcome is inspected (freeze-before-inspect, §12 spirit).
Everything here runs without Renxi; nothing in it takes a decision that is his.

**Three goals, in dependency order:**
1. **Dense lit session reference** — the 21-Aug reference was 30 cells, relaxed guards,
   built in darkness (declared then as a datum). Two lit laps replace it.
2. **Freeze K_online** — provisional K=3 from the twin rehearsal; confirm against the real
   lit distribution and freeze before scored runs.
3. **Gate A/B under full light** — the dangerous condition, measured 21-Aug (+0.130 m with a
   door strike, no gate). Twin rehearsal predicts −49% (0.050 → 0.025 m median |lateral|).

**First real runs carrying Z_t.** Since 21-Aug the stack emits `role`/`role_reason`/
`authority` (stabilised + raw), `illum_b`/`dvis_gate`, and VOXMEM exposure fields. This
session produces the first REAL samples of all of them.

---

## Pre-flight (before any run)

| check | how |
|---|---|
| Branch | `git fetch && git checkout ensayo/door-gate-isaac && git pull` — the gate, role fields, stabiliser and visual contract all live here. **Smoke first** (below): this branch has been exercised against the twin for days, against the robot not since the merge. |
| Perception | `bash tools/arranca_percepcion.sh` → health on **:8008** |
| App mode | verify relocalization topic (the `slam_relocation/odom` rename bit us live on 21-Aug) |
| Battery | start **≥ 85%**; walking blocks stop at **60%** (strafe halves below it) |
| Lights | ALL ON for the whole session; **operator writes wall-clock time of every switch change** — the unlabelled L1–L5 sweep cost us the primary evidence once already |
| Notebook | per leg: battery %, contacts/strikes (instrumentation cannot see hand-to-frame), start/end wall time |

**Smoke (1 short leg, unscored):** `G1_ARM=SMOKE python3 g1_goto.py goto B` with defaults.
Verify before continuing: samples carry `role`, `illum_b` ≈ 110–120 lit, `phase_sent`, and
the run reaches. If the branch misbehaves on the real robot, STOP and fall back to
`feature/dcc-integration` — the session then does goals 1–2 only (they don't need the gate).

## Block A — lit calibration laps (~15 min, ~2 battery pts)

```
G1_ARM=CAL_LIT_1 G1_METASM=1 python3 g1_goto.py goto B
G1_ARM=CAL_LIT_2 G1_METASM=1 python3 g1_goto.py goto A
```

Then build and **declare** the reference (amendment §10: declared before any scored run):

```
python3 tools/mapa_visibilidad.py --reales --min-opp 5 --min-runs 2
```

- `--reales` is MANDATORY (21-Aug lesson: twin and real runs share the dataset).
- **Expectation corrected by the 25-Aug twin rehearsal** (an earlier version of this runbook
  promised 80–120 cells on no evidence): two lit twin laps give **27 cells at `--min-runs 2`
  and 35 at `--min-runs 1`** — same order as the dark real reference (30). Lit real laps may
  do better, but do not count on it. **Run a THIRD lap if battery allows** (~1 pt): it is the
  cheapest way to make `--min-runs 2` viable. If < 25 cells with `--min-runs 2`, fall back
  to `--min-runs 1` and DECLARE the relaxation, exactly as 21-Aug did.
- The output key is `points` (not `cells`); check the count with
  `python3 -c "import json; print(len(json.load(open('<ref>'))['points']))"`.
  `load_cov_ref` reads `points` — verified compatible on 25-Aug.
- Commit the reference file + its hash before Block C. That commit is the declaration.

## Block B — freeze K_online (~10 min, ~2 pts)

One lit baseline pair WITH the new reference live:

```
G1_ARM=BASE_LIT_REF G1_METASM=1 G1_COVREF=<ref.json> python3 g1_goto.py goto B
G1_ARM=BASE_LIT_REF G1_METASM=1 G1_COVREF=<ref.json> python3 g1_goto.py goto A
```

Read the online `cov_missing` distribution of those two legs (`analysis/resumen_sesion.py`).
Decision rule, written BEFORE looking: if the lit-baseline p95 ≤ 2 (twin rehearsal: normal
{0:×401, 1:×46}, staged glass peaking at 4), **freeze K_online = 3** — commit
`G1_DCC_COV_MISSING=3` into the pre-registration amendment. If p95 ≥ 3 the real floor is
noisier than the twin's and K=4 is frozen instead, with the distribution attached. Either
way it is frozen NOW, before Block C produces anything scoreable.

## Block C — gate A/B, full light (~55 min, ~8 pts)

**The design is BALANCED, and the 25-Aug rehearsal is why.** The first version alternated
gate state and direction together (OFF→B, ON→A, OFF→B, …), which confounds them perfectly:
the rehearsal produced GATE_ON legs at 168 s against GATE_OFF at 88 s, and that difference
is unattributable — the return leg B→A is independently known to be harder ("nace
encajonada, sin sitio para girar"). Two legs were also lost to the 300 s timeout. Both
problems disappear when each gate state is run in BOTH directions.

Eight legs, walked continuously (no repositioning), two repetitions of the 2×2:

| leg | arm | direction | leg | arm | direction |
|---|---|---|---|---|---|
| 1 | `GATE_OFF` | A→B | 5 | `GATE_OFF` | A→B |
| 2 | `GATE_OFF` | B→A | 6 | `GATE_OFF` | B→A |
| 3 | `GATE_ON`  | A→B | 7 | `GATE_ON`  | A→B |
| 4 | `GATE_ON`  | B→A | 8 | `GATE_ON`  | B→A |

All legs carry: `G1_METASM=1 G1_COVREF=<ref.json> G1_VOXMEM=1 G1_DOOR_VIS=1` plus the gate
flag. VOXMEM is expose-only (collects the I¹ field, does not act).

```
G1_ARM=GATE_OFF G1_METASM=1 G1_COVREF=<ref> G1_VOXMEM=1 G1_DOOR_VIS=1 G1_DOOR_VIS_GATE=0 python3 g1_goto.py goto B
```

**Record the direction with the arm** — `G1_ARM=GATE_OFF_AB` / `GATE_OFF_BA` etc. — so the
analysis can separate the two factors without inferring direction from the filename.

**Safety note for OFF legs:** the only real full-light no-gate leg we have STRUCK the door
(21-Aug, +0.136 m, on the B→A return). Operator within reach of the stop on every OFF leg,
and especially on OFF·B→A, which is the exact condition that produced the strike.

**Pre-registered read-out (written before the data):** median |lateral| in gap for
ON vs OFF **within each direction**, then pooled; door strikes ON vs OFF; `ENG-C`
strafe-only count; `dvis_gate` active fraction (~100% ON legs / 0% OFF). Twin rehearsal
predicts −49%. If ON is not better, that is the result — the gate goes back to the bench,
not the numbers.

**If time runs short**, drop the second repetition (legs 5–8) rather than dropping a cell of
the 2×2: an unbalanced four-leg block is worth more than an unidentifiable six-leg one.

## On-site verification before packing up (~5 min)

```
python3 analysis/resumen_sesion.py
```

- every run: `role`/`authority` present, `illum_b` in the lit band, `cov_missing` present
- `dvis_gate` = 1 only on ON legs
- copy the operator notebook (switch times!) into `tasks/SESSION_LOG_2026-08-XX.md`
- commit + push before leaving the lab

## Budget

~13 legs walking ≈ 13 pts + margin → start ≥ 85%, expect ≥ 68% at the end. ~2 h.
If battery forces a cut, drop Block C's second repetition first (see above).

## Explicitly out of scope

Reserved configurations; anything confirmatory; the physical glass block (the W1 ray-march
instrument fix is still pending and is its own session); changing K_online after Block C.

# META2 — DCE governance on the G1 (Meta-Reasoner 2.0 integration)

How the tutor's **Meta-Reasoner 2.0** (package `meta-reasoner-2.0/`, guide in its .docx) governs
the real G1 during A↔B door-crossing runs, and how every decision is measured. This is the
runtime instantiation of the DCA paper (*Decentralised MetaReasoning Through Capability
Abstraction*): bounded shared experience → capability-local QoE self-assessment → arbitration
(KEEP / SWITCH / FALLBACK / HELP / INSUFFICIENT).

## Architecture

```
g1_goto.py (control loop, ~3.3 Hz)
   │  clearance, progression, reliability, laser_noise, battery, mobility, phase
   ▼
g1_meta2_bridge.py (~2 Hz)          ← smoothing, few-shot margin, persistence, profiles
   │  readings {value, reliability, uncertainty}
   ▼
meta-reasoner-2.0/meta_reasoner_2_0.py   ← the tutor's reasoner, UNTOUCHED
   │  KEEP / SWITCH(analogy) / FALLBACK / HELP / INSUFFICIENT
   ▼
g1_goto.py actuation (G1_META2=2): per-analogy speed ceiling, experience-abort escalation
```

Config: `config_meta2_g1door.json` — QoE boundaries calibrated from measured run distributions
(cruise clearance ~1.0 / progression ~0.48; door zone clearance median 0.60 / progression 0.10;
collisions happen with clearance ≈ 1.0 → the frame is LiDAR-blind, hence the mobility channel).

## Shared-experience mapping (meta-parameters)

| Meta-parameter | Source | Notes |
|---|---|---|
| `safety` | SEI clearance (c0/1.5 m, 0..1) | laser+depth forward free space |
| `progression` | SEI progression (approach rate/0.3 m/s) | HELD at last median during commanded stops (ENG-AL etc.) |
| `mobility` | achieved/commanded speed over 1.2 s | **resistance channel**: ~0 = pressing something unseen; hard veto |
| `battery_consumption` | bat/100 | task attention 0 in the door task (zero-attention normalisation) |
| reading `reliability` | SensingMonitor (`loc_conf × (1−laser_noise)`) | median-smoothed |
| reading `uncertainty` | `0.02 + 0.10·laser_noise + k·σ_hist(metric)` | **few-shot margin**, see below |

## Bridge-side layers (the reasoner itself is never modified)

1. **Median smoothing** (5 samples ≈ 2.5 s) of all inputs — raw SEI progression flickers 0↔0.5.
2. **Few-shot margin (P10)**: per-reading uncertainty adds the historical dispersion of the
   metric (`G1_M2_HIST_K=0.4`, cap 0.35). A metric oscillating across a QoE boundary widens its
   belief/plausibility interval → belief_fulfillment (the gate basis) drops and uncertainty_gap
   penalises ranking → plausibility requires consistency, not one lucky read. `=0` restores the
   one-shot behaviour (clean ablation).
3. **Switch hysteresis**: confirm after 3 consecutive winners toward caution, 6 toward the
   preferred analogy (fast to caution, slow to relax); `SWITCH_MARGIN=0.08` stable-fulfillment
   advantage required; return-to-preferred on sustained ties.
4. **Action persistence**: FALLBACK/HELP act only after 2 consecutive firm decisions (`X?` = warn).
5. **Profiles**: Efficient_Nav → no ceiling; Cautious_Nav → 0.28; FALLBACK → 0.24; HELP → 0
   (forward only — recoveries/ESCAPE/turns untouched; HARD-GUARD and the vision moderator stay above).

## Experience escalation (P9 — "the experience should inform the robot to abort")

`G1_M2_ABORT=1` (default). Triggers, checked at every governance decision:
firm **HELP sustained ≥ 8 s** (`G1_M2_HELP_S`), or a **75 s window** (`G1_M2_ABORT_WIN`) with
**≥ 60 % firm FALLBACK/HELP** (`G1_M2_ABORT_BAD`) **and net goal progress < 0.4 m**
(`G1_M2_ABORT_PROG`). Active mode → STOP + run abort (`aborted_meta2_help`, event
`meta2_experience_abort`); shadow → `META2-ABORT-SHADOW` warning once/min.
Offline acceptance (project rule: fire before the failure it targets, zero fires on clean runs):
fires at **t=94 s** in the 584 s door-loop run 100927, re-arming every ~70 s; **0/6 false** on
the clean runs of the same day — including with the few-shot margin enabled.

## Modes and how each run self-describes

| `G1_META2` | Behaviour |
|---|---|
| `0` / unset | governance off (baseline behaviour) |
| `1` SHADOW | decides + logs, control untouched — validation mode |
| `2` ACTIVE | + speed ceilings, `!M` phase mark, experience-abort |

Recorded in four independent places (never again "which mode was this run?"):
dataset header (`meta2_mode`, `meta2_enabled`, plus `env`/`sim_id` from `G1_ENV`/`G1_SIM_ID`),
run summary (`meta2_mode`, `meta2_capped_ticks` — >0 ⟺ active actually clipping), edge event
`meta2_cap_on` + `META2-CAP ON` log line, and CSV columns in `runs_summary.csv`.
Per-sample fields: `meta2_act`, `meta2_active`, `meta2_cap`, `meta2_tens`, `meta2_ful`.
Events: `meta2_mode`, `meta2_switch/fallback/help`, `meta2_cap_on`, `meta2_experience_abort`.

## Tools

- Offline replay of the whole bridge against any recorded run (no robot):
  `python g1_meta2_bridge.py dataset/<run>.json`
- Reasoner unit tests: `python -m unittest meta-reasoner-2.0/test_meta_reasoner_2_0.py`
- Calibration reports (labelled cases): see `meta-reasoner-2.0/USAGE_GUIDE.md` §4.
- Results table: `python summarize_runs.py` → `runs_summary.csv` (fill `condition`/`notes` by hand).

## Evaluation plan (paper)

Condition matrix — normal / water payload no lid (`spills_human` marked by a human) / human
nearby / low battery — × governance: frozen branch **`baseline`** (no governance), FSM
(`G1_FSM=1`), META2 active, and the DST ablations (4 modes via `evaluation_controls` in the
config) plus one-shot vs few-shot (`G1_M2_HIST_K=0`). Simulation runs are tagged `env=sim`
(`G1_ENV=sim G1_SIM_ID=<scenario>`) and must be filtered out of real-robot tables.

## Field validation status (2026-07-03)

Shadow validated on 5 runs: semantics correct (Efficient in the open, Cautious 3 s after
`door_engage`, back to Efficient after crossing) and FALLBACK/HELP precede real trouble
(firm FALLBACK 0.35 s before an impact; sustained HELP during the 584 s loop). Pending:
first confirmed ACTIVE run (check `meta2_capped_ticks > 0`), then the condition matrix.

# Stage 2 reproducibility package

This repository is the companion of the **robotic coffee-delivery benchmark (Stage 2)**
of *A Computational Theory of Cognitive Memory* (Qiu, Pham, Lendinez Ibanez, Li).
Supplementary Note 6, section 8.4, prescribes what the Stage 2 package must record;
this file maps every prescribed item to the concrete artefact in this repository, gives
the exact commands that re-run and re-score the evaluation, and declares what is
environment-bound and cannot be re-run bit-identically.

**Branch map.** `ensayo/door-gate-isaac` is the evaluation branch (everything below
lives here). `baseline` holds the frozen navigation configuration without governance,
kept for fair comparison. `main` holds the platform layer (the stock-G1 single-channel
stack) and points here.

**Tiers.** Everything in this package is the **development tier**: the digital-twin
runs (Isaac Sim) plus the real-robot sessions logged in `dataset/`. The confirmatory
tier and the reserved configurations have not been run and are not claimed.

---

## 1 · The Note 6 §8.4 checklist, item by item

| §8.4 item | Where it lives |
|---|---|
| Robot and sensor versions | §3 below (platform); per-run `env_g1` + `git.sha` inside every `dataset/<run>.json` |
| Calibration files | §4 below: named constants with derivation history in `sim/isaac/isaac_bridge.py` (motion), `sim/emulador_deteccion.py` + `dataset/curvas_etiqueta.json` (vision), `analysis/escala_pose.py` (measurement scale), `calib/` (camera) |
| Maps | `summit/ref_map_g1.json` (historical reference), `nav_map.json` (furniture), `dataset/visibilidad_gemelo_sesion.json` (session coverage reference, twin) + `_prov` predecessor; scene generator `sim/isaac/office3d.py` |
| Initial and successor records | `dataset/certificado_T12.json` + `analysis/verifica_t12.py` (successor mapping verified non-rewriting; predecessors recoverable via git history — commits recorded in the certificate) |
| Interface schemas | `dcc_conditions.py` (`I0_CAMPOS` / `I1_EXTRA`: the original vs revised interface field sets); per-sample schema `g1_goto_run/v1` written by `g1_goto.py` |
| Reference certificates | `dataset/*_omega_ref.json` (one per staged run, written in the same act that stages the world — `guion.py`); derivation function `dcc_omega.py::delta_muestra` |
| Route layouts | Waypoints A/B/C + door centre and axis: constants in `g1_goto.py`; session layout in `tasks/SESSION_PREP_GATE_AB.md` |
| Lighting and object schedules | `guion.py` `GUIONES` — per-configuration schedules (T1–T11) for light, glass, objects, by wall-clock instant or zone entry |
| Battery schedules | `guion.py` T7 (declared trajectory 75→55; the D3 conflict with the 60% cut is declared in `tasks/DECISIONES_PENDIENTES_RENXI.md`) |
| Randomisation seeds | §5 below |
| Trial allocation | `tasks/manifiestos/campana_dcc_v2.txt` (current campaign), `campana_dcc.txt` (v1, superseded — kept), `campana_isaac.json` (N=30 variance campaign) |
| Sensor histories | `dataset/<run>.json` → `samples[]` (per-tick fields) + `laser_snapshots[]` (accumulated-belief crops) + archived camera frames alongside each run |
| Role resolutions | Per sample: `role`, `role_crudo`, `role_reason` — resolver `dcc_roles.py` (pure function; thresholds at top of file) |
| Authority records | Per sample: `authority`, derived from `phase_sent` guard markers (`dcc_roles.py::autoridad`) |
| Actions | Per sample: `cmd`, `sent`, `spd`; per run: `events[]` |
| Safety interventions | `events[]` (collisions, with pre-impact frames), ASSIST markers in `phase_sent`, operator notes in `tasks/SESSION_LOG_*.md` |
| Analysis scripts | `dcc_omega.py`, `dcc_conditions.py`, `dcc_roles.py`, `dcc_secundarios.py`, `analysis/` (scorers, negative controls, variance, realism) |

## 2 · Re-running and re-scoring

Scoring is **offline and deterministic**: it reads recorded runs plus their
certificates and never calls the robot, so every number in
`tasks/RESULTADOS_ISAAC_V2.md` can be reproduced from the repository alone.

```bash
# negative controls of the certificate/scoring machinery (5 controls, must all pass)
python3 analysis/test_dcc_omega.py

# primary outcomes (A_meta, A_Omega) + paired contrasts at run level, campaign v2
G1_DCC_MAN=$PWD/tasks/manifiestos/campana_dcc_v2.txt python3 analysis/nivel_run.py

# secondary outcomes (§9.3): adopted/missed transitions, switching delay, returns
G1_DCC_MAN=$PWD/tasks/manifiestos/campana_dcc_v2.txt python3 analysis/corre_secundarios.py

# twin variance / realism battery (N=30 campaign)
python3 analysis/varianza_isaac.py

# T12 record verification
python3 analysis/verifica_t12.py
```

Re-running the **campaign itself** requires the twin (Isaac Sim bridge on the lab
GPU machine): `python3 campana_dcc_v2.py` (resumable; appends to the v2 manifest).
Staged single runs: `python3 guion.py T8 --destino B`. These are environment-bound
(GPU, Isaac 5.1, the bridge container) and reproduce *statistically*, not
bit-identically; the recorded dataset is the frozen evidence.

**Claim-boundary mapping (Note 6 §8.3.9).** The four separately-reported endpoints:
structural interface conformity → field presence per condition (`dcc_conditions.py`)
and the negative controls; reference-decision recovery → A_meta / A_Ω against the
certificates; DCC continuity → the staged T1–T12 transitions and the §9.3
secondaries; behavioural outcomes → `result`, collisions, safety events. No claim in
`tasks/RESULTADOS_ISAAC_V2.md` crosses these boundaries.

## 3 · Platform and versions

- **Robot:** stock Unitree G1 "Air" (consumer; single WebRTC channel through the
  vendor app — no ROS, no SDK on board). The exact app/firmware version is recorded
  per real session in the operator log; it is not queryable programmatically.
- **Twin:** Isaac Sim 5.1.0 (container `isaaclab_setup`), bridge
  `sim/isaac/isaac_bridge.py`, adapter `g1_sim_adapter.py` (the robot stack runs
  UNMODIFIED against the bridge).
- **Host:** Python 3.10.12; `reproducibility/requirements-frozen.txt` is the pip
  freeze of the machine that produced the recorded campaigns; `requirements.txt` is
  the curated install set.
- **Provenance per run:** every `dataset/<run>.json` records `git` (sha + dirty
  flag), `env_g1` (the full flag environment) and `schema`.

## 4 · Calibration constants (what they are, where derived)

| Constant | Value | Derivation |
|---|---|---|
| `ESCALA_V` (VSCALE) | 0.74 | closed-loop refit after the measurement-scale correction; history in `sim/isaac/isaac_bridge.py` comments |
| `TAU` (command smoothing) | 0.42 | fitted on real command streams |
| Interface latency | 0.20 s ± 0.035 jitter | measured over 38,647 real ticks |
| Pace | 20 steps/s wall-clock | enforced (the "hidden clock" fix) |
| `K_COMPARA` (path scale) | 8 (~2.4 s) | declared comparison scale, `analysis/escala_pose.py` |
| Visual contract | luma EMA α=0.2, threshold 99/100 | 21-Aug material; measured caveats in the D2/D3 ledger note |
| Detection curves | chair measured; couch ×0.37, refrigerator ×0.51 | staged tandas + 6,112 free-navigation detections (`dataset/curvas_etiqueta.json`) |
| Coverage instrument | `cov_missing` v2, accumulated base, K=3 pending freeze | validated 25-Aug (glass vs control legs); `g1_goto.py::_cov_missing_celdas` |

## 5 · Randomisation and seeds

- Detection emulator: fixed seed (`semilla=7`, `sim/emulador_deteccion.py`).
- Twin sensor noise: `G1_SIM_NOISE=1` (distributions in the adapter; **draw seed not
  fixed** — declared: twin runs reproduce statistically, not bit-identically).
- Call/trial order: fully recorded in the manifests (no hidden randomisation).
- Scoring: deterministic (no sampling); bootstrap, when used, states its seed in the
  analysis script.

## 6 · Integrity verification

```bash
cd reproducibility && sh verifica.sh
```

`reproducibility/SHA256SUMS` lists the package core (scoring modules, manifests,
certificates, campaign runs, maps, curves) by **repository-relative path** — a
deliberate lesson from an earlier package whose manifest shipped absolute paths and
was unverifiable by any recipient. Regenerate after any legitimate change with
`sh reproducibility/genera_sumas.sh`.

## 7 · What is *not* in this package

The confirmatory tier and the 12 reserved configurations (untouched by design; they
run only under the supervising author's control). The real-robot arm of the C1–C4
comparison beyond the sessions already logged. The V5.8 paper text itself.

# Experiment 2 reproducibility package

This repository is the companion of **Experiment 2 — the robotic
development-to-deployment evaluation** — of *A Computational Theory of Cognitive
Memory* (Qiu, Pham, Lendinez Ibanez, Li; **V5.8**). Supplementary **Note 8**
specifies the evaluation; its §8.16 prescribes what the released package must
contain. This file maps every prescribed item to the concrete artefact in this
repository, gives the exact commands that re-run and re-score the evaluation, and
declares what is environment-bound and cannot be re-run bit-identically.

**Branch map.** `ensayo/door-gate-isaac` is the evaluation branch (everything below
lives here). `baseline` holds the frozen navigation configuration without
governance, kept for fair comparison. `main` documents the platform layer and
points here.

**Lifecycle position (Note 8 §8.1).** Everything in this package is
**development-stage**: the interface, observer configuration, evidence contracts,
role and resolution rules, authority fields, instruments, controls and audit
machinery constructed and diagnosed here, plus the development campaigns that
exercised them (digital twin + the logged real-robot sessions). The **frozen
confirmatory C1–C4 deployment evaluation has not been run**: it requires the
configuration freeze and release, and it runs only under the supervising author's
control. Development diagnostics are never presented here as deployment-effect
evidence — the distinction Note 8 §8.4 draws is the one this package keeps.

---

## 1 · Conditions and contrasts (Note 8 §8.6–8.7)

Implemented in `src/dcc_conditions.py` — the assignment matches V5.8 exactly:

| Condition | Interface | Resolution | In code |
|---|---|---|---|
| C1 | original (I⁰) | temporal incumbent verification | `PROCESO=temporal, INTERFAZ=I0` |
| C2 | original (I⁰) | distributed role reconstruction | `PROCESO=distribuida, INTERFAZ=I0` |
| C3 | revised (I¹) | temporal incumbent verification | `PROCESO=temporal, INTERFAZ=I1` |
| C4 | revised (I¹) | boundary governance + DCC resolution | `PROCESO=distribuida, INTERFAZ=I1` |

The original interface exposes map, local map, current LiDAR/RGB/pose/payload/
battery (`I0_CAMPOS`); the revised interface adds selected sensing history, sensing
and illumination quality, uncertainty, timestamps, frames and authority grounds
(`I1_EXTRA`). Prespecified contrasts, with the paper's names: **interface effects**
Δ = C3−C1 and Δ = C4−C2; **resolution effects** Δ = C2−C1 and Δ = C4−C3.

## 2 · The Note 8 §8.16 checklist, item by item

| §8.16 item | Where it lives |
|---|---|
| Robot and sensor versions | §4 below; per-run `env_g1` + `git.sha` inside every `dataset/<run>.json` |
| Calibration files | §5 below: named constants with derivation history in `sim/isaac/isaac_bridge.py` (motion), `sim/emulador_deteccion.py` + `dataset/curvas_etiqueta.json` (vision), `analysis/escala_pose.py` (measurement scale), `calib/` (camera) |
| Maps | `summit/ref_map_g1.json` (historical), `nav_map.json` (furniture), `dataset/visibilidad_gemelo_sesion.json` (session coverage reference) + `_prov` predecessor; scene generator `sim/isaac/office3d.py` |
| Route layouts | Waypoints A/B/C + door centre/axis: constants in `src/g1_goto.py`; session layout `tasks/SESSION_PREP_GATE_AB.md` |
| Interface schemas | `src/dcc_conditions.py` (`I0_CAMPOS`/`I1_EXTRA`); per-sample schema `g1_goto_run/v1` |
| Evidence contracts | `dcc_omega.py::FUNDAMENTO` (role → accepted grounds), the visual-quality contract (luma EMA α=0.2, threshold, measured caveats in the ledger), coverage contract (`cov_missing` v2 semantics in `src/g1_goto.py`) |
| Initial and successor records | `dataset/certificado_T12.json` + `analysis/verifica_t12.py` (non-rewriting succession; predecessors recoverable via the git commits recorded in the certificate) |
| Observer configurations | `src/perception_server.py` + `requirements-perception.txt`; twin observer: `src/g1_sim_adapter.py` env + `sim/emulador_deteccion.py` constants |
| Frozen references | `dataset/*_omega_ref.json` — one certificate per staged run, written in the same act that stages the world (`src/guion.py`); derivation `dcc_omega.py::delta_muestra` (independent of condition and robot output, Note 8 §8.9) |
| Lighting and object schedules | `guion.py::GUIONES` (T1–T11: light, glass, objects; by wall-clock instant or zone entry) |
| Battery schedules | `src/guion.py` T7 (declared 75→55; D3 conflict declared in the ledger) |
| Randomisation seeds | §6 below |
| Trial allocation | `tasks/manifiestos/campana_dcc_v2.txt` (current), `campana_dcc.txt` (v1, superseded, kept), `campana_isaac.json` (N=30 variance) |
| Sensor histories | `dataset/<run>.json` → `samples[]` + `laser_snapshots[]` + archived camera frames |
| Boundary records | every sample row is one decision boundary: interface fields, role problem, resolution, authority, action — the §8.8 record set as columns |
| Role resolutions | per sample `role`, `role_crudo`, `role_reason`; resolver `src/dcc_roles.py` (pure function, thresholds at top) |
| Authority records | per sample `authority` from `phase_sent` guard markers (`dcc_roles.py::autoridad`) |
| Actions | per sample `cmd`, `sent`, `spd`; per run `events[]` |
| Safety interventions | `events[]` (collisions with pre-impact frames), ASSIST markers, operator notes `tasks/SESSION_LOG_*.md` |
| Diagnostic controls | `analysis/test_dcc_omega.py` (5 negative controls of the scoring machinery); the 21-Aug **solid-wall control** that invalidated the first glass witness (development diagnostic, §8.4 — its records and the exclusion are in the audit trail); staged-glass vs no-glass validation legs (25-Aug) |
| Exclusion logs | `reproducibility/EXCLUSIONES.md` — every excluded or degenerate run, with reason and disposition |
| Analysis scripts | `src/dcc_omega.py`, `src/dcc_conditions.py`, `src/dcc_roles.py`, `src/dcc_secundarios.py`, `analysis/` |
| Configuration hashes | per-run `git.sha` (+ dirty flag); package integrity `reproducibility/SHA256SUMS`; release procedure in §7 |
| Machine-readable results tables | `reproducibility/resultados_stage2_dev.json` + `.csv`, regenerated by `python3 reproducibility/exporta_resultados.py` |

## 3 · Re-running and re-scoring

Scoring is **offline and deterministic**: it reads recorded runs plus their
certificates and never calls the robot; every number in
`tasks/RESULTADOS_ISAAC_V2.md` reproduces from the repository alone.

```bash
python3 analysis/test_dcc_omega.py                     # negative controls (must all pass)
G1_DCC_MAN=$PWD/tasks/manifiestos/campana_dcc_v2.txt python3 analysis/nivel_run.py
G1_DCC_MAN=$PWD/tasks/manifiestos/campana_dcc_v2.txt python3 analysis/corre_secundarios.py
python3 analysis/varianza_isaac.py                     # twin variance / realism battery
python3 analysis/verifica_t12.py                       # successor-record verification
python3 reproducibility/exporta_resultados.py          # machine-readable tables
```

Re-running the **campaigns** needs the twin (Isaac Sim bridge on the lab GPU):
`python3 src/campana_dcc_v2.py` (resumable); staged single runs `python3 src/guion.py T8
--destino B`. Environment-bound: statistical, not bit-identical, reproduction; the
recorded dataset is the frozen evidence.

**Claim-boundary chain (Note 8 §8.15), mapped.** Behavioural stability ⇏ DCA
conformity ⇏ reference recovery ⇏ DCC continuity ⇏ beneficial realised use. In this
package: conformity → interface field presence per condition + negative controls;
recovery → A_meta/A_Ω against certificates; continuity → staged T1–T12 transitions
+ §9.3-style secondaries; behaviour → `result`, collisions, safety events. Reported
separately throughout; no claim crosses a link.

## 4 · Platform and versions (Note 8 §8.2 identification items)

- **Robot platform:** stock Unitree G1 "Air" (consumer; single WebRTC channel via
  the vendor app — no onboard ROS/SDK). App/firmware version recorded per real
  session in the operator log; not programmatically queryable.
- **Sensors:** onboard LiDAR (app-filtered stream: ~82 pts/sweep, 3.7 m range cap
  — measured, modelled in the twin), RGB via WebRTC (320×180 effective), odometry
  from the app's SLAM. Camera calibration in `calib/`.
- **Payload surrogate:** open cup, 246 g full (weigh-per-run protocol in the
  campaign notes).
- **Independent safety:** operator hand-on-stop protocol (runbook §Block C safety
  note); collision events instrumented per run.
- **Twin:** Isaac Sim 5.1.0 (container `isaaclab_setup`), bridge
  `sim/isaac/isaac_bridge.py`, adapter `src/g1_sim_adapter.py` — the robot stack runs
  UNMODIFIED against the bridge.
- **Host:** Python 3.10.12; `reproducibility/requirements-frozen.txt` (pip freeze
  of the machine that produced the campaigns); `requirements.txt` curated.
- **Release identifier / configuration hash:** not yet issued — assigned at the
  configuration freeze (§7). Until then, per-run `git.sha` is the configuration
  pointer.

## 5 · Calibration constants (value, derivation)

| Constant | Value | Derivation |
|---|---|---|
| `ESCALA_V` (VSCALE) | 0.74 | closed-loop refit after the measurement-scale correction; history in `sim/isaac/isaac_bridge.py` |
| `TAU` (command smoothing) | 0.42 | fitted on real command streams |
| Interface latency | 0.20 s ± 0.035 jitter | measured over 38,647 real ticks |
| Pace | 20 steps/s wall-clock | enforced (the "hidden clock" fix) |
| `K_COMPARA` (path scale) | 8 (~2.4 s) | declared comparison scale, `analysis/escala_pose.py` |
| Visual contract | luma EMA α=0.2, threshold 99/100 | 21-Aug material; overlap caveats measured 26-Aug (ledger D2/D3 note) |
| Detection curves | chair measured; couch ×0.37, refrigerator ×0.51 | staged tandas + 6,112 free-navigation detections |
| Coverage instrument | `cov_missing` v2 (accumulated base), K pending freeze | validated 25-Aug, glass vs control legs |

## 6 · Randomisation and seeds

- Detection emulator: fixed seed (`semilla=7`).
- Twin sensor noise: `G1_SIM_NOISE=1`; **draw seed not fixed** — declared: twin
  runs reproduce statistically, not bit-identically.
- Trial order: fully recorded in the manifests; no hidden randomisation.
- Scoring: deterministic; any bootstrap states its seed in the script.

## 7 · Release procedure (configuration freeze, §8.3/§8.5)

When the development stage closes: (1) regenerate `reproducibility/SHA256SUMS`;
(2) tag the commit (`release/exp2-<version>`) — the tag name is the **release
identifier**, the tagged commit sha + the sha256 of `SHA256SUMS` are the
**configuration hash** pair; (3) any later change to interface meaning, contract
semantics, role semantics, governance or DCC semantics requires a new versioned
release, never an unrecorded adjustment. No release has been issued yet.

## 8 · Integrity verification

```bash
sh reproducibility/verifica.sh      # checks the 170+ entry SHA256 manifest
```

`reproducibility/SHA256SUMS` uses **repository-relative paths** — a deliberate
lesson from an earlier package whose absolute-path manifest was unverifiable by any
recipient. Regenerate after any legitimate change with
`sh reproducibility/genera_sumas.sh`.

## 9 · What is *not* in this package

The frozen confirmatory C1–C4 deployment evaluation (not run; reserved
configurations untouched). The Experiment 1 (multi-LLM) package — it ships
separately. The V5.8 paper text.

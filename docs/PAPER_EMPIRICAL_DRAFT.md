# Draft — Empirical Validation subsection for the DCA paper (Section VII)

> Borrador para insertar como subsección de la Sección VII (p.ej. "H. Empirical Validation on a
> Humanoid Digital Twin") del paper V6.1. NO insertado en el docx: pendiente de revisión de
> Adrian/Renxi. Los números salen de `analyze_campaign.py` sobre la campaña sim 2026-07-04
> (83 runs, gemelo Gazebo del laboratorio real). La campaña de ~20 runs con el G1 físico
> (protocolo abajo) completará la tabla con la columna real.

---

## H. Empirical Validation on a Humanoid Digital Twin

The analytical validation above is complemented by an empirical campaign on a digital twin of
the physical deployment: a Unitree G1 humanoid navigating a door-crossing delivery task
(A↔B, ~13 m, an 0.8 m doorway and a 0.71 m corridor pinch) in a Gazebo replica of the real
laboratory, generated from the robot's own SLAM maps. The full production navigation stack
(static-map global planner, local DWA, door-engagement controller) runs unmodified; the DCE
runtime (Meta-Reasoner 2.0) governs it through the shared-experience bridge at 2 Hz, exactly
as deployed on the physical robot. The governance task is the *payload* condition of the
delivery scenario: the robot carries an open cup of water. Spilling is modelled physically —
first-mode liquid sloshing in a cylindrical cup (natural frequency ω₀ ≈ 21 rad/s for R = 4 cm),
driven by the cup's horizontal acceleration including arm-offset and gait-bounce terms, with
spills generated as a non-homogeneous Poisson process on surface elevation against freeboard.
Consistently with the simulation study that motivated this design, jerk — not speed alone —
dominates spill risk. A sealed lid multiplies the hazard rate by 0.25.

Eight governance arms (N = 8 runs each; lid arms N = 6; mean ± 95% CI) instantiate the
architectural comparisons of Sections IV–VII:

| Arm | Governance | Reached | Time (s) | Spills/run | Risk (% time) |
|---|---|---|---|---|---|
| M0 baseline | none (0.30 m/s) | 8/8 | 83.4 ± 5.2 | 0.50 ± 0.37 | 8.1 ± 1.4 |
| M1 fixed-conservative | none (0.18 m/s cap) | 8/8 | 110.8 ± 2.4 | 0.00 | 0.8 ± 0.2 |
| Single-candidate governed | DCE, one analogy | 5/8 (3 HELP) | 106.2 ± 5.1 | 0.12 ± 0.24 | 0.5 ± 0.2 |
| **M2 payload (proposed)** | DCE, payload task | **8/8** | **107.9 ± 4.2** | **0.00** | **0.4 ± 0.2** |
| M2 + task-blind prior | DCE, miscalibrated task | 8/8 | 108.2 ± 3.5 | 0.00 | 0.7 ± 0.2 |
| M0, sealed lid | none | 6/6 | 88.1 ± 4.9 | 0.17 ± 0.33 | 10.6 ± 2.9 |
| M2, sealed lid (twin-calibrated) | DCE, covered-delivery | 5/6 (1 HELP) | 102.5 ± 2.2 | 0.17 ± 0.33 | 0.7 ± 0.5 |

Five findings ground the framework's claims:

**(1) Deployability governance converts spills into bounded time cost.** The ungoverned
baseline reaches the goal fastest but spills on half of its runs, spending 8.1% of each run
with the liquid surface above 70% of freeboard. Active DCE governance under the payload task
eliminates spills entirely (0/8 runs; risk 0.4%) at a 29% time cost, while preserving 8/8 task
completion. The fixed-conservative baseline achieves the same safety by construction, but
rigidly: it cannot re-certify a faster capability when conditions permit (see finding 4).

**(2) HELP is the correct outcome of an empty deployable set.** When the candidate set is
reduced to a single analogy and a hard-veto meta-parameter (mobility, the resistance channel)
enters its dangerous region while the robot presses the corridor pinch, no deployable
candidate remains; the runtime returns sustained HELP and the experience-escalation layer
aborts the run (3/8 runs). This is not a failure of the mechanism but its specified semantics:
capability-validity governance must refuse to certify rather than silently persist
(conservative non-selection, Section IX of [V3]).

**(3) The calibration-limited regime is observable in deployment.** Running the physical
laboratory's QoE calibration unchanged on the digital twin — whose clearance distribution is
systematically narrower (cruise median 0.76 vs ≈1.0) — left the efficient capability
non-certifiable for entire runs: an instance of the θ_QoE calibration-error budget of Table IV
arising live rather than analytically. Re-calibrating the QoE boundaries from measured twin
distributions (per the interface-refinement pattern) restored certification in open stretches
without touching the reasoner or the physical-robot configuration.

**(4) Task-conditioned analogy selection exploits payload changes.** With a sealed lid, the
ungoverned baseline improves only passively (spills 0.50 → 0.17/run through physics; its risk
exposure is unchanged at 10.6%). The governed system, given the covered-delivery task
configuration on the calibrated twin, actively exploits the change: 102.5 ± 2.2 s versus
111.5 ± 7.3 s for the same governance under the open-cup configuration — a policy-level gain
unavailable to fixed baselines, consistent with the lid-state delta of the 2-D study.

**(5) Temporal scales of correction separate as designed.** A task-blind (wrong) prior is
corrected *within* runs in ≈4 s: per-tick shared-experience evidence (clearance) strips the
miscalibrated capability's certification before its first failure, so the cross-run
analogical-trust layer receives no failure signal to learn from on the twin. The cross-run
layer — per-analogy Dempster–Shafer belief over {match, mismatch}, persisted between runs,
with plausibility-gated deployment and a conservative policy blend — is validated
mechanistically (plausibility trace 1.00 → 0.75 → 0.53 → 0.36 over three spill runs, with
corrected zero-shot start on the fourth). Its operational regime is the physical laboratory,
where cruise clearance certifies the fast capability and gait-induced spills are invisible to
per-tick channels: exactly the failure mode the run-level trust layer exists to catch. The
physical-robot campaign (Section H.1) exercises it.

### H.1 Physical-robot protocol (in progress)

Twenty runs on the physical G1 in the mapped laboratory, water cup payload, four conditions
(baseline / governed, open / sealed lid), spill ground truth marked manually per event
(UDP channel, timestamped into the per-tick dataset), Layer-2 belief state persisted across
runs (`G1_M2_STATE`). Metrics identical to the twin campaign; runs tagged `env=real` in the
shared results table.

---

## Internal notes (not for the paper)

- The zero-spill outcome of the wrong/wrongdst/wrongdstsim arms is NOT an experimental
  failure: it is finding (5). If the cross-run recovery curve is wanted IN SIMULATION, the
  protocol faithful to the thesis (Ch. 5, 5.3.6.3) is to force the wrong analogy (lock) for
  the first K runs (implementable as G1_M2_LOCK) — but the natural place to capture it is the
  physical robot, where open-floor cruising genuinely certifies the Efficient capability.
- The table omits the redundant arms (wrongsim / wrongdst / wrongdstsim); they are available
  via analyze_campaign.py and in runs_summary.csv (filter env=sim, sim_id lab_v1_payload_*).
- Contrast with the thesis 2-D simulation (warehouse-cluttered): M0 SE=13.0 / 2.12 spills-run;
  M2 SE=37.6 / 0.28. The 3-D twin reproduces the same safety ordering M0 < M1 = M2 with
  physically grounded sloshing.
- Reproduction: python3 sim_campaign.py (arms via CAMPAIGN_ARMS) - python3 analyze_campaign.py
  - commits up to 0b6ea24 on main.

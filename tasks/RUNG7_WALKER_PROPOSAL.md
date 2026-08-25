# Rung 7 proposal — the PhysX walker inside the benchmark loop

**Status: PROPOSAL ONLY, nothing started.** This is a benchmark meta-parameter
(the motion-fidelity tier of the twin), so the decision is Renxi's. Written
25-Aug so the option is fully specified when he returns; filed as D9 in
`tasks/DECISIONES_PENDIENTES_RENXI.md`.

## What exists today (all verified, separately)

- **The kinematic twin** (`sim/isaac/isaac_bridge.py`): g1_goto drives Isaac
  through the adapter; motion is velocity integration with calibrated
  VSCALE=0.74, TAU=0.42, interface latency 0.20 s (jitter 0.035), wall-clock
  pace 20 steps/s. This is the surface every campaign and every T1-T12 staging
  runs on. Its realism battery and its N=30 dispersion are measured (25-Aug
  campaign).
- **The walker** (`/ws/g1_travesia.py` + policy `2026-08-23_08-06-56_g1_full`):
  the articulated G1 walks A -> doorway -> B -> back on PhysX, legs driven by
  our trained policy, on the real map geometry (same numbers as g1_goto).
  Verified end-to-end 23/24-Aug; video captured. It is NOT connected to
  g1_goto: it self-navigates a fixed waypoint list.
- The infra risks are already banked as fixes in `tasks/ISAAC_SIM_PLAN.md`
  ("four quiet failures": defaultPrim, enable_cameras, the decorative G1's
  broken joints, the heading-control training convention).

## What rung 7 is

Replace the bridge's velocity integration with the walker: g1_goto's (v, w)
commands feed the policy's command interface; the recorded pose comes from the
articulation root. One switch, both ways (`G1_SIM_WALKER=0/1`), following the
no-Renxi pattern: the kinematic path stays intact and default.

## What it buys (why it is the top rung)

1. **Motion texture becomes emergent instead of hand-modeled.** TAU, command
   smoothing, start/stop transients and turn dynamics are currently parameters
   fitted to real data; a walker produces them from contact physics.
2. **P4 (IMU sway) becomes measurable in sim.** The gait layer exists precisely
   because the twin "moves like furniture"; rung 7 is its validation loop.
3. **T10 becomes physically honest.** Today the blockage board stops the
   kinematic robot by collision proxy; a walker is *actually* blocked (contact,
   push, recovery) - the no_use ground (D6) gets real evidence texture.
4. **Footstep-induced pose noise** replaces part of the injected noise model,
   with its own spectrum - the D7 duplicate-pose question may partially
   dissolve (poses come from physics stepping, not a 3.2 Hz resampler).

## What it costs / risks (why not unilaterally)

- **Calibration lineage breaks.** VSCALE/TAU/latency were fitted for the
  kinematic channel. The walker needs its own calibration (command-tracking
  gain, effective speed vs commanded, latency of the policy loop) and every
  claim that cites the 25-Aug battery would need re-derivation on the new tier.
- **The dispersion debt reopens.** The N=30 variance just measured (25-Aug)
  describes the kinematic twin; the walker has its own variance (contact
  nondeterminism, policy stochasticity) and needs its own N.
- **Wall-clock cost.** PhysX + policy inference per step is far heavier than
  velocity integration; campaign throughput drops (to be measured in phase 0;
  the travesia runs suggest roughly real-time, vs the paced kinematic legs).
- **Failure surface.** A walker can fall. A fallen robot mid-campaign is a new
  outcome class the scoring pipeline has no category for (abort? no_use? - a
  protocol question in itself, hence Renxi).

## Proposed migration (phases, each with a kill criterion)

- **Phase 0 (measurement, no integration):** command-tracking bench on flat
  ground - commanded (v, w) vs achieved, latency, speed ceiling; wall-clock
  cost per sim minute. Kill if tracking error or cost is unusable.
- **Phase 1 (switch):** `G1_SIM_WALKER=1` path in the bridge; pose/odometry
  from articulation root through the SAME interface (latency + HZ_ODOM
  unchanged, so the interface contract stays fixed). Kinematic default
  untouched.
- **Phase 2 (A/B):** balanced 8-leg design (walker/kinematic x A->B/B->A)
  plus comparison against the 133 real runs: path ratio at K=8, duration
  ratio, duplicate-pose rate, IMU sway spectrum vs real (P4 metric).
- **Phase 3 (Renxi):** decide the tier for the confirmatory campaign - and
  the fallen-robot outcome category if adopted.

## What this proposal does NOT change

The confirmatory tier stays unrun; the 12 reserved configurations stay
untouched; the SS12 variance model stays unfrozen; every existing campaign
result remains reported on the kinematic tier, labeled as such.

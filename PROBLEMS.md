# G1 A→B→A — Problem list (one change per run)

Requested by Renxi (2026-07-02): "a list of problems, solve one by one."
Status legend: **OPEN** (not addressed) · **CODED** (fix in repo, not yet validated on robot) ·
**VALIDATED** (confirmed on robot runs) · **MITIGATED** (bounded, watch only).

Each fix is validated in its own run, in the order below. Evidence cites run IDs in `dataset/`.

---

## P1 — Start-pose trap at waypoint B — **CODED (validate next run)**

The recorded waypoint B parks the robot nose-first ~40 cm from the cream sofa, inside a pocket
(sofa + boxes + drawer unit). Every B→A leg is born with no room to turn: 143039 spent 39/54 s
stuck; 145306 aborted at 140 s; 150440 collided at t=4 s and t=12.5 s without leaving the start
point; 152532 reached A but paid 1 collision at t=17 s (omap_near=81) escaping the pocket.

Key measurement that changed the fix: the LiDAR does NOT see the sofa from the start pose —
the map read c0=1.92 m "clear" in 150440 while the camera was buried in the cushion; c0 only
dropped to 0.22 AT the impact. The Mid-360 near-blind band + fabric make a laser-only trigger
useless here. The camera does see it: carpet_pct 0.00–0.46 on all four pocket starts vs
0.86–0.97 on every good start.

**Fix (in `g1_goto.py`, env `G1_ESCAPE`, default ON):** start-of-run ESCAPE — within the first
5 s, if the robot has not moved yet (<0.30 m) and either c0<0.45 (laser) or carpet_pct<0.50
(vision), back up straight 0.5 m before planning. Once per run; a real collision recovery still
overrides it. Offline replay over all 29 runs with data: fires on exactly the 4 pocket starts
(at t=0.6 s, before their first collision) and on none of the other 25.

This is the second wired instance of Renxi's "real vs fake stuck" arbitration: LiDAR says free,
vision says buried, vision wins — but only for a bounded courtesy maneuver, never to paint the map.

## P8 — Global planner ran on the LIVE laser instead of the map — **CODED (critical, found by Adrian)**

Watching the viewer, Adrian caught that the green GLOBAL plan zigzags around live laser
blobs. Confirmed in code: normal-mode global A* planned over `build_costmap(oset)` — the
live, flickering laser map — and used the loaded map only as a last-resort fallback. Two
consequences, both reproduced offline (`sim_globalplan.py`, real postmortem cloud + the real
`nav_map.json`): (1) plan instability — six Livox "frames" from the same pose give paths that
differ by 0.27 m on average (the trembling door axis that DOOR-AL fought); (2) the live plan
cuts straight through mapped walls the laser doesn't currently see — the "walks at the wall,
then turns away late" behaviour.

**Fix (default `G1_GLOBALMAP=hard`):** global A* now plans on the STABLE map — loaded walls +
score-saturated persistent cells + collision marks (the P-hard tier). The local DWA keeps
using the live laser for everything new/unmapped; aggressive mode still replans with recent
cells. `G1_GLOBALMAP=live` restores the old behaviour, `=ref` uses the loaded map only.
This is the Nav2 architecture: static global, sensor-driven local.

## P2 — Clearance model narrower than the arms/hands — **PARTLY CODED**

Renxi: "clearing is wrong — it does not cover the width of the hands yet." The data agrees:
all three runs of the latest batch had exactly one IMU-detected graze **with the map fully
aware** (omap_near = 42 / 48 / 81 known-occupied cells within 2.5 m): 152030 and 152330 at the
door mouth (−3.8, +1.2), 152532 leaving the B pocket. The planner clears the torso model, and
the swinging arms clip what the model ignores.

Numbers: aggressive-mode DWA clearance floor `G1_AGGR_R` = **0.13 m**, and DOOR-GO enters the
gap when c0 > 0.13 m. The G1's physical half-width is ~0.22 m at the shoulders, ~0.28–0.30 m
including arm swing. So in aggressive mode we authorize gaps that are physically too tight by
~15 cm per side.

**Done now:** `G1_AGGR_R` default raised 0.13→0.20 (revert: `G1_AGGR_R=0.13`), and HARD-GUARD
is now default ON — replayed over all 29 logged runs it engages ZERO times in clean reached
runs (zero cost) and would have issued 4 STOPs before 150440's first collision (`G1_HARDGUARD=0`
reverts). Note P8's fix also attacks this problem at the source: a stable global plan stops
dragging the robot along noise-bent paths that shave the doorframe.

**Attempted and rejected by simulation (honest negative result):** a "press-guard" (commanded
forward + body barely moves + vision says "on top" → back off before the IMU notices). The
152030/152330 pre-impact signature (3–4 s scraping at 0.05–0.15 m/s with cnear 21–28) is real,
but replay over all runs shows the same signature during legitimate careful door transits
(6 false fires in 145010, 3 in 142725, both clean runs): with odometry+vision only it is not
separable. It needs the leg-torque channel (already logged) — queued as future work, tuned in
`g1_replay.py` before it ever touches the robot. Longer-term idea worth discussing: tuck/lock
arms during DOOR phase via the arm SDK (physically narrows the robot).

## P3 — Perception intermittency — **OPEN (diagnostics added, measure next run)**

Renxi: "perception is unreliable." Measured: vision contributed 0 obstacle cells on 36–41 % of
ticks in the last three runs, in a mix of 1–2-tick blips and long 21–23-tick (≈6–7 s) dead
stretches. Ambiguity: perc_n=0 is legitimate when the camera faces open floor (YOLO has nothing
to report) — in 152330 the color channel stayed alive in 76 of the 107 zero ticks, so much of
it is real-empty-scene, not server failure. What we cannot yet separate is stale-server vs
empty-scene.

**Diagnostic now in repo:** per-tick `perc_age` (seconds since the last *fresh* perception
response) in every sample and in the `[VIS]` log line. Next run tells us whether the dead
stretches are latency (perc_age grows) or honest empty scenes (perc_age ~0.5 s, cells 0). Fix
decided after measuring — likely candidates: hold-last-result ≤1 s against flicker, or
server-side pipeline speedup. Note the Renxi split already bounds the damage: vision is
advisory (speed moderation + escape trigger), LiDAR owns the map.

## P4 — Door-mouth phantom veto (vision walls sealing a passable door) — **VALIDATED**

Was: clamp columns ("obstacle on top") painted a phantom 0.7 m wall across the door; a
perfectly aligned robot (goal_err −2°) turned 180° and fled (143511/143646). Fix: clamps never
enter the map, they only moderate speed (Renxi principle). Validated: 152330 and 152532 both
crossed the door; today's door failures are P2-type grazes, not vetoes.

## P5 — Room-B floor calibration (lighting) — **CODED (validate next run)**

Old calibration read the B-room carpet at 0.8–2.2 % (lighting difference); robot was
vision-blind on the return leg. `floorcolor_calib.json` v3 (measured from run filmstrips:
H105, S[25,215], V[55,195]) reads B-floor 65–78 %, office still ~100 %. Watch carpet_pct in
the next B→A run; it also feeds P1's trigger threshold.

## P6 — Low-height objects invisible to the laser band — **OPEN (queued A/B)**

Robot is "not very good on low height objects" (Renxi). The anti-floor band cuts lidar returns
below −0.5 m; low objects live there. Queued A/B: `G1_HBAND_LO=-0.7` with the anti-noise stack
on. Depth (0.10 m+) and the carpet channel cover part of it meanwhile.

## P7 — Relocalization divergence — **MITIGATED (guard active)**

≥4 pose jumps >0.5 m within 10 s → STOP + abort + postmortem cloud (`G1_RELOCGUARD`, default
on). Offline validation on the 529 m fantasy-walk run: would have stopped it at 0.4 m. No
occurrence since instrumented.

---

## Simulation harness (how fixes get accepted now)

Two repo tools replay fixes against all logged runs before they touch the robot:
`g1_replay.py` (trigger rules: escape / hard-guard / press-guard — reports fires vs real
collisions and false-positive rate on clean runs) and `sim_globalplan.py` (global-plan
stability on a real postmortem cloud, live vs static map). Acceptance rule: fires before the
failures it targets, zero fires on clean reached runs. ESCAPE passed 4/4 + 0/25; HARD-GUARD
passed 0-cost + 4 pre-impact stops; press-guard FAILED and was not shipped.

## Working order

1. **Next run** (all changes env-revertible, each with its own log marker): P1 ESCAPE +
   P5 calib v3 + P8 stable global plan + P2a (AGGR_R 0.20, HARD-GUARD on). If the run is
   WORSE than 152330/152532, isolate by flipping one env at a time, in this order:
   `G1_GLOBALMAP=live` → `G1_HARDGUARD=0` → `G1_AGGR_R=0.13` → `G1_ESCAPE=0`.
2. **Then**: P3 decision from perc_age data; P6 A/B (`G1_HBAND_LO=-0.7`).
3. **Then**: press-guard revisited with leg-torque in `g1_replay.py`.

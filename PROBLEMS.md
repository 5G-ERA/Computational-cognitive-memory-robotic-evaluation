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

## P2 — Clearance model narrower than the arms/hands — **OPEN (next change)**

Renxi: "clearing is wrong — it does not cover the width of the hands yet." The data agrees:
all three runs of the latest batch had exactly one IMU-detected graze **with the map fully
aware** (omap_near = 42 / 48 / 81 known-occupied cells within 2.5 m): 152030 and 152330 at the
door mouth (−3.8, +1.2), 152532 leaving the B pocket. The planner clears the torso model, and
the swinging arms clip what the model ignores.

Numbers: aggressive-mode DWA clearance floor `G1_AGGR_R` = **0.13 m**, and DOOR-GO enters the
gap when c0 > 0.13 m. The G1's physical half-width is ~0.22 m at the shoulders, ~0.28–0.30 m
including arm swing. So in aggressive mode we authorize gaps that are physically too tight by
~15 cm per side.

**Proposed change #2 (env-only A/B, no code):** `G1_AGGR_R=0.20`. If door transit still
succeeds (door is the binding constraint) but grazes persist, follow-up in code: raise the
DOOR-GO entry floor 0.13→0.22 and center harder (DOOR-CTR) before committing. Longer-term idea
worth discussing: tuck/lock arms during DOOR phase via the arm SDK (physically narrows the robot).

## P3 — Perception intermittency — **OPEN (diagnostics added, measure next run)**

Renxi: "perception is unreliable." Measured: vision contributed 0 obstacle cells on 36–41 % of
ticks in the last three runs, in a mix of 1–2-tick blips and long 21–23-tick (≈6–7 s) dead
stretches. Ambiguity: perc_n=0 is legitimate when the camera faces open floor (YOLO has nothing
to report) — in 152330 the color channel stayed alive in 76 of the 107 zero ticks, so much of
it is real-empty-scene, not server failure. What we cannot yet separate is stale-server vs
empty-scene.

**Diagnostic now in repo:** per-tick `perc_age` (seconds since the last *fresh* perception
response) in every sample. Next run tells us whether the dead stretches are latency (perc_age
grows) or honest empty scenes (perc_age ~0.5 s, cells 0). Fix decided after measuring — likely
candidates: hold-last-result ≤1 s against flicker, or server-side pipeline speedup. Note the
Renxi split already bounds the damage: vision is advisory (speed moderation + escape trigger),
LiDAR owns the map.

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

## Working order

1. **Next run**: validate P1 (ESCAPE) + P5 (calib v3) — same run, B→A; watch `ESCAPE-START/END`
   in the log and carpet_pct at start.
2. **Then**: P2 A/B (`G1_AGGR_R=0.20`), A→B→A; success = door still crossed, zero grazes.
3. **Then**: P3 decision from perc_age data; P6 A/B (`G1_HBAND_LO=-0.7`).

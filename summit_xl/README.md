# Summit XL — second robotic experiment (in preparation)

> **Tier: development, pre-protocol.** Nothing in this directory is a result of the paper.
> There is no frozen configuration, no pre-registration and no confirmatory run. What is here
> is the instrumentation, the controllers, a 2D bench and the raw material of three pilot
> sessions (17–19 Sep 2026), kept in the open so that the experiment is built where it will
> be audited. Every claim below carries the tier of the evidence behind it.

The G1 evaluation in this repository asks *which role governs* when a humanoid's senses
disagree. This directory prepares the same question on a second, very different body: a
**Robotnik Summit XL** — wheeled, skid-steer, ROS 2 Humble, Nav2 in the vendor's containers,
a 3D LiDAR flattened to a 2D scan — driving **A → door → B** through a 742 mm office doorway
with a 613 mm-wide footprint.

It is intended as the second experiment on three threads of the paper: **explanation** (a
replay that says *why* a traversal succeeded or failed), **cognitive memory**, and **DCA**,
where the Summit's shared-experience interface is **clearance**. Phase 1 is deliberately
modest: repeated traversals in nominal conditions, with nominal safety, logged so that they
can be compared run for run with the G1's.

`summit/` (one level up) is a different thing: the Summit's maps used as ground truth *for
the G1*. This directory is the Summit as a subject.

## What is here

| Path | Contents |
|:---|:---|
| [`registro/`](registro/) | **Run logger with the G1's schema.** Passive ROS 2 node → one `g1_goto_run/v1` JSON per run and one row in `runs_stats.csv` with the G1's 71 columns, in order. Offline test suite, traversal wrapper, G1-vs-Summit table merger |
| [`controlador/`](controlador/) | What ran on the robot on 18 Sep: the clearance-guided doorway controller (`cruza_vano.py`), the Nav2 crossing manoeuvre, the A→B orchestration, the laser heartbeat, the one-command session start |
| [`banco2d/`](banco2d/) | A kinematic 2D simulator on the real map, driving the **real robot's Nav2 parameters**, with ground-truth pose, clearance and collision. Reproduces the doorway problem without the robot |
| [`analisis/`](analisis/) | Offline analyses: the footprint-vs-doorway sweep (`ensayo_vano.py`), AMCL error against ground truth |
| [`datos/`](datos/) | Raw material of the 18 Sep session (per-leg logs, clearance replays, effective Nav2 parameters, Nav2 log) and of the 19 Sep bench validation |

## The logger, and why its numbers can sit next to the G1's

```bash
python3 summit_xl/registro/prueba_core.py          # 20 offline checks, no ROS, no robot
# on the robot — wraps any traversal, touches nothing that moves it:
bash summit_con_registro.sh B C1-LASER -- bash ~/summit_ab_holgura.sh B
# both robots in one table:
python3 summit_xl/registro/compara_runs.py --g1 dataset/runs_stats.csv --summit RUNS/runs_stats.csv
```

- The clearance / progression / reliability / laser-noise / localisation-confidence metrics
  are **not reimplemented**: the logger imports [`src/g1_metrics.py`](../src/g1_metrics.py),
  the same file the G1 uses, and writes its SHA-256 into every run header.
- Column parity is checked **against the source** of [`src/g1_goto.py`](../src/g1_goto.py),
  not against a copied list: if the G1 gains a column, the test fails.
- The G1's own [`tools/summarize_runs.py`](../tools/summarize_runs.py) reads a Summit run
  unmodified. That is one of the 20 checks.

**Same name, different measurement — declared in the `defs` field of every JSON:**

| Column | G1 | Summit XL |
|:---|:---|:---|
| `collisions` | pose did not advance 5 cm in 0.9 s while commanded forward, or a leg-torque spike | **wheels** turned < 25 % of the *effective* command, with effort. The G1's rule cannot work here: the Summit's odometry is open-loop (computed from commands), so the pose advances while the robot is pinned; and at the 0.05 m/s crossing speed the fixed 5 cm threshold would count every slow crossing as a collision. One contact = one collision |
| `progression` | normalised to 0.30 m/s, the G1's gait | same normalisation, kept for scale; compare `progress_rate` (m/s) for units |
| `c0` | centre → nearest return in a ±25° cone | identical, but the Summit's bumper is 0.361 m from its centre: see `c0_body_min` |

As the G1 code itself warns: **`collisions = 0` does not mean clean.** A graze that does not
slow the wheels is invisible; an operator can mark one by hand.

Summit-only columns follow the 71 shared ones: `collision_ahead`, `teb_infeasible`,
`patience_exceeded`, `recoveries`, `scan_drops` (Nav2 literals read from `/rosout`),
`drive_stalls`, `amcl_jumps`, `arm`. **`collision_ahead` is a prediction by Nav2's collision
checker, not a contact, and is never counted as a collision.**

## What the pilot sessions showed, by tier of evidence

**Observed on the robot — 18 Sep 2026, one session, nominal safety throughout**
(full footprint, `footprint_padding` 0.02, `simulate_ahead_time` 2.0):

| Doorway primitive | Crossed | Note |
|:---|:---:|:---|
| Nav2 `DriveOnHeading` | 1 of 3 | the one that crossed did so by accumulation, after three aborts |
| Clearance-guided controller (`cruza_vano.py`) | 2 of 2 | first attempt each time; minimum measured clearance 45 and 39 mm per side |

Five attempts. This establishes that both primitives *can* cross; it establishes neither
reliability nor superiority.

**Read from that session's logs — 19 Sep** ([`datos/2026-09-18/nav2/`](datos/2026-09-18/nav2/)):

- The doorway aborts are `Collision Ahead - Exiting DriveOnHeading`, 18 occurrences, from the
  behaviour server's collision checker. The straight run that crossed had the same three
  rejections as the two that did not: the count does not separate success from failure.
  The literal proves a checker rejection, **not** an occupied cell — the checker also rejects
  on transform or footprint failures.
- The failure near B is a **different mechanism**: `TebLocalPlannerROS: trajectory is not
  feasible` until `Controller patience exceeded`; only then do the recoveries start, and
  those hit the collision checker. Zero infeasibility events on the six legs to the door;
  47 and 241 on the two legs to B (the leg with 47 arrived).
- Effective frames: `behavior_server` and `local_costmap` both run in `robot_map`, not the
  Nav2-default `odom`.

**Offline model — not measured on the robot** ([`analisis/ensayo_vano.py`](analisis/ensayo_vano.py)):
the padded footprint rasterised as Nav2's `FootprintCollisionChecker` does (perimeter,
Bresenham), swept over entry angle, lateral offset and doorway phase against the 2 cm grid.
Validated against the closed form `2·[(L/2)·|sin θ| + (W/2)·|cos θ|]`. The laser sees the
doorway 0.730 m wide, not the taped 0.742 m. With a clean jamb the geometry tolerates ≈ 3.5°
centred; **one extra lethal cell per jamb drops that to ≈ 0.5° and forbids the crossing at
10 mm offset.** Whether that extra cell exists in the real costmap has not been measured; it
is the first thing the next session records.

**Bench only:** the logger's contact detector against the simulator's ground truth — one
staged collision, one detection, 0.36 s later, same position, no repeat while pinned. The
bench's wheel effort is synthetic. **The logger has not run on the real robot.**

**Hypothesis, unvalidated:** a normalised margin ρ = clearance margin per side ÷ perception
uncertainty as the quantity that explains crossing versus aborting. The pilot numbers are
compatible with it and cannot test it: both terms come from the episode they would explain.

## Withdrawn

Kept on the page because a diagnosis that was wrong is part of the audit trail:

- *"The scan drops cause the aborts."* 1244 of the 1640 drops fall in the final leg, after
  the recoveries began; the straight runs that failed had 1 and 0.
- *"Nav2's costmap TF cache is bounded by `transform_tolerance`."* False for Humble: the
  buffer keeps the 10 s default; the literal `earlier than all the data in the transform
  cache` is also emitted on message-filter timeouts.
- *"Swapping DDS interface priorities fixes the laser dropouts."* It blinded host and
  station; reverted.
- *"The laser-guided controller crossed 3 of 3."* It crossed 2 of 2. Three is the day's total
  across both primitives.

## Not yet done

A protocol. Conditions, outcomes and exclusion rules are not written, so nothing here can be
confirmatory. The logger, the effort thresholds and `/rosout` visibility from the robot host
are untested on hardware. The B mark is badly placed for the final turn.

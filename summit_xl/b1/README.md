# B1 — frozen baseline navigation configuration (Summit XL, simulation)

> **Tier: frozen baseline, simulation only.** B1 is not a result of the paper. It is the single
> navigation controller that every Summit XL data-collection trial must use, so that successes
> and failures come from the same controller. Nothing measured while choosing B1 enters the
> C2 / C2.5 / C3 comparison.

**Frozen:** 25 Nov 2026, 11:58 CET. **File:** [`b1_nav2_params.yaml`](b1_nav2_params.yaml),
sha256 `49f9eac43769dde82addae1f85aae1651164ed53351d04120e9366020078c8a8` (also in
[`SHA256SUMS`](SHA256SUMS)).

## What B1 is

Nav2 (Humble) with the DWB controller and its `ObstacleFootprint` critic, the
`navigate_w_replanning_time.xml` behaviour tree (no recovery behaviours), and a reduced
inflation on the global costmap:

| Setting | Value |
|:---|:---|
| Global `inflation_radius` / `cost_scaling_factor` | 0.15 m / 10.0 |
| `footprint_padding` (both costmaps) | 0.02 m |
| Local costmap inflation | none |
| `observation_persistence` (both costmaps) | 0.0 |
| Progress checker | 0.5 m in 10 s |
| Robot footprint | 0.722 × 0.613 m, **not reduced** |

It is the earlier candidate v1 with one change: `observation_persistence` 1.0 → 0.0 in both
costmaps. The goal for B1 was *feasible, not necessarily reliable*: a stock Nav2 that can
cross the 742 mm doorway from the nominal centred start, without a doorway-specific policy.

## Evidence (2D bench, two machines; rules written before each batch)

| Batch | Trials | Outcome |
|:---|:---|:---|
| Confirmation, noisy start (pose ±3 cm/±2°, latency 0–100 ms, velocity ±10 %, start delay 0–2 s, laser noise) | 60 | **55/60** safe crossings (Wilson 95 %: 81.9–96.4 %), 0 collisions, 0 door-crossing clearances < 1 cm |
| Stability: same seed, no added noise, 3 seeds × 20 | 60 | 60/60 identical outcomes |
| Independent adversarial review (new seeds) | 40 + 40 | 39/40 and 40/40; held |
| All noisy centred trials, 24–25 Nov | 160 | **152/160** |

Pass bar, fixed in advance: ≥ 51/60 noisy trials, 0 collisions, ≤ 2 door-crossing clearances
under 1 cm, and identical outcomes under a fixed seed.

## Known limits (stated, not hidden)

- **Residual failures (~2 % without added noise, ~5 % with it).** Two stop modes, both ended
  by Nav2's progress checker: (1) leaving the door slightly too straight, the padded footprint
  wedges between the door frame and the next corner and DWB stands still (explained
  geometrically); (2) a stop just before the door while already turning toward the goal (not
  yet explained). Which runs fail is driven mainly by a 1–2 cm localisation (AMCL) error;
  failures cluster when the initial localisation error is ≥ 2 cm. No general Nav2 setting
  tested (longer progress allowance, odometry-frame local costmap, stock recovery tree, shorter
  DWB horizon) removed them.
- **Clearance behind the door** goes down to a few millimetres at a second narrowing (0.728 m).
  The 1 cm clearance rule applies to the door crossing only.
- **Not run on the real robot.** The physical Summit XL runs Nav2 1.1.17 (bench: 1.1.20) and
  its DWB loop misses 20 Hz in more than half of its cycles. Applying B1 to the robot is a
  separate, supervised step.

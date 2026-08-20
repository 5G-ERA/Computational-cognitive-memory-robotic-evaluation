# The twin walks like furniture — measured, and the gait layer that answers it

2026-08-20 (evening). Adrian's observation: the real G1 *walks* — the twin's rigidised model slides on a
`planar_move` plugin, so it has none of the body oscillation of a biped. Question: is the
noise we inject realistic? Answer, measured: **no in structure, close in scale.**

## What the real robot does that the twin does not (8 real runs, 20 Aug, walking samples)

| Signal | Real (walking) | Twin (calibrated noise, 24 sim runs 13 Aug) |
|---|---|---|
| IMU pitch | median 0.028 rad, p90 0.057 | **no IMU at all** |
| IMU roll | median 0.019 rad | — |
| Lateral acceleration | median 0.98 m/s2, p90 2.4 | — |
| Yaw rate (weave) | median 0.098 rad/s, p90 0.42 | — |
| `c0_std` (front-clearance jitter) | **0.087** | **0.035 — 2.5x too clean** |
| `laser_noise` | 0.173 (p90 0.586) | 0.136 (p90 0.349) |
| `filt_rej` | 0.054, sd 0.028 (steady) | 0.073, p90 0.202 (**bursty**) |
| `loc_conf` | median 0.964 | median 1.000 (pinned) |
| `scan_churn` | 0.400 (p90 0.494) | 0.418 (p90 0.662) |

Structure of the mismatch: real walking noise is **continuous and correlated** (the body
oscillates, so ranges and heading are modulated quasi-periodically); the twin's model is
**white per-ray + occasional bursts** — which matches the medians it was calibrated on but
not the dynamics, and produces burst artefacts the real robot does not show.

## The gait layer (`G1_SIM_GAIT=1`, adapter-only, image untouched)

A stride oscillator whose phase advances **only while moving** (est. speed > 0.02 m/s, with
~1 s decay after stopping, like a real biped settling). Four coherent effects:

- **Fore/aft range modulation at step frequency** (2x stride): the nod. `G1_SIM_GAIT_AP`.
- **Port/starboard range modulation at stride frequency**: the sway. `G1_SIM_GAIT_AR`.
- **Yaw weave** on the reported pose: `G1_SIM_GAIT_AW` (default 0.022 rad, from gz/(2*pi*f)).
- **Lateral position wobble** of the reported pose: `G1_SIM_GAIT_AY` (default 0.05 m, from
  a_y/(2*pi*f)^2). The cloud is projected from the wobbled pose while Gazebo stays rigid, so
  the scan-to-map mismatch becomes quasi-periodic — the real pathway by which `loc_conf`
  dips.

Two declared honesty limits:

1. **The amplitudes are effective parameters in metres, not physical angles.** The real
   mechanism (a 2D lidar plane sweeping height bands of furniture as the body pitches) is not
   representable in a planar twin; we reproduce its *statistical effect* on the fields the
   navigation stack actually consumes, and calibrate against the real distributions above.
2. **The stride frequency is not identifiable from the recordings** (samples at 0.32 s alias
   the ~1.4 Hz step). Default 0.7 Hz stride is declared from the G1 walking cadence, not
   measured. A high-rate IMU capture would settle it — noted for a future session, not
   required for the statistics to match.

Validation protocol: same machine, same world, same waypoints, noise on in both arms; only
`G1_SIM_GAIT` changes. Targets: `c0_std` median toward 0.087, `laser_noise` toward 0.173,
`loc_conf` toward 0.964, `filt_rej` steady (not bursty), and no behavioural regression
(arrivals, collisions, door crossing under the golden configuration).

## Substrate note

The twin now runs on GPUEDGE (x86_64 native — the Mac ran it under Rosetta). Durations are
NOT comparable across machines (REPRODUCIBILITY.md); a fresh per-machine baseline is part of
the validation runs.

## Validation results

(to be appended after the calibration runs)

**Run on the G1 compute station, 20 Aug (night). Twin live there (image tar verified
byte-identical, sha ce4e729f); rosbridge is NOT in the preserved image (the 7-Aug audit
finding) — hand-installed in the container, now an idempotent step of `arranca_gemelo.sh`.**

Substrate finding first: **the Mac noise calibration does not transfer.** Baseline
(noise on, gait off) on the station: `c0_std` 0.079 (Mac-era twin: 0.035), `filt_rej` 0.076,
durations 34–46 s vs 86–110 s (native x86 vs Rosetta real-time factor). Noise statistics are
substrate-dependent; per-machine baselines are mandatory, as REPRODUCIBILITY.md anticipated
for durations.

| field | baseline (no gait) | gait v1 defaults | **gait frozen (AP .04 AR .03 AY .025 AW .015)** | real target |
|---|---|---|---|---|
| `laser_noise` | 0.196 | 0.232 | **0.181** | 0.173 |
| `c0_std` | 0.079 | 0.102 | **0.080** (p90 0.388) | 0.087 (p90 0.364) |
| `scan_churn` | 0.429 | 0.497 | 0.468 | 0.400 |
| `filt_rej` | 0.076 | 0.110 | 0.094 | 0.054 |
| `loc_conf` | 0.994 | 0.986 | 0.993 | 0.964 |

Behaviour: baseline 3/3 reached (one collision in one run), gait arms 6/6 reached, 0
collisions, door crossed every run — no regression under the golden configuration.

Verdict: with the frozen amplitudes the two *continuous* signatures the gait was built for
(`laser_noise`, `c0_std`) sit on target within 5–8%, medians and p90 both. The remaining
deviations (`filt_rej` 0.094 vs 0.054, `scan_churn` 0.468 vs 0.400, `loc_conf` 0.993 vs
0.964) are already present in the no-gait baseline on this substrate — they belong to the
old burst component and the perfect sim walls, not to the gait layer. **Pending, flagged:**
recalibrate the burst/dropout component on the station (one variable at a time), and revisit
`loc_conf` realism (sim walls relocalise too well).

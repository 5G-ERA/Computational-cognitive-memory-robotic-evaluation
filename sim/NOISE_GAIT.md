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

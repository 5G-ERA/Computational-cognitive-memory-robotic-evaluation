# N=30 variance campaign — both calibration debts, paid and read honestly

**Date:** 25-Aug-2026, evening. **Config:** the final calibration (VSCALE=0.74,
TAU=0.42, interface latency 0.20+-0.035, pace 20 steps/s, HZ_ODOM=3.2), one
nominal condition, alternating A->B / B->A, nothing tuned per leg. **Runs:**
30/30 reached; one leg (183155) is the known goto-to-current-position empty and
drops out of analysis by the <20-samples filter -> N=29 analyzed. Manifest:
`dataset/campana_isaac.json` (analysis reads the list, never guesses by date).
Verdict script: `analysis/varianza_isaac.py`.

## Debt 1 - the 13-metric realism battery was stale

The old battery (`analysis/realismo_isaac.py`) hardcodes 4 runs from 22-Aug that
predate the final fixes; it stays as the historical snapshot. Refreshed on the
N=29: **medians 11/13 inside the real IQR**. The two misses are marginal:

- *velocidad*: 0.06 vs real 0.07 on a degenerate-width IQR [0.07-0.07];
- *holgura c0*: 0.70 vs p25=0.72 — 2 cm below the interval edge.

## Debt 2 - SS12 dispersion was unqualified

**Dispersion comparable (IQR ratio in [0.4, 2.5]): 8/13.** The five failures
are ALL on the under-dispersed side — the twin is too repeatable:

| metric | IQRs/IQRr | reading |
|---|---|---|
| holgura c0 | 0.03 | sim clearance nearly constant (0.69-0.71) vs real 0.72-1.34 |
| fase % ENG | 0.14 | real is bimodal (0 or ~54%: engagement happens or not); sim always ~29% |
| colisiones | 0.00 | sim never collides; real IQR reaches 1 |
| obstaculos vistos | 0.32 | same map every leg vs a lab that changes |
| fase % BRK | 0.34 | fewer braking regimes in sim |

## The caveat that must travel with this table

The real side (132 runs) mixes days, start poses, battery states, people
walking, and different configs; the sim side is 30 legs of ONE nominal
condition. Part of the dispersion gap is **between-condition variance on the
real side**, not missing robot variance in the twin. The comparison is
right-shaped for medians and conservative for dispersion: an under-dispersion
verdict here is an UPPER bound on the twin's true deficit. Thursday's session
produces real same-condition repeats (Block C, 8 legs, one config) — the first
data that can split within-condition from between-condition dispersion.

## What this means for SS12 (unfrozen, Renxi's)

Claims that depend on medians stand on a 29-run base. Claims that depend on
run-to-run variance (confidence intervals, rarity of events like collisions)
must either declare the under-dispersion or inject the missing variance sources
explicitly (start-pose jitter, dynamic obstacles). Leg 28 (191601, 175 s, 2.4x
median duration) shows the tail is not empty even in one condition.

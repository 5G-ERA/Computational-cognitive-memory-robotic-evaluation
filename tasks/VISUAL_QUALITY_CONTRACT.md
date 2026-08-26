# Visual-quality contract (W2) — build-order step 3

**Status:** written and frozen 23 Aug 2026, from material recorded 21 Aug. Closes step 3 of
§8 of the pre-registration. No robot time was needed; the material already existed.

The pre-registration (§2.2) requires "a declared operating envelope stating when an RGB
detection is admissible. Without it, 'cannot see' has no defined boundary and the pair is not
scoreable." It also warned that mean frame luminance is pinned by the camera's auto-exposure
(105, 103, 109, 107, 105, 104 across 09:00→18:00) and named contrast and grain as the
candidates instead.

## The admissible statistic

    illum = EMA(alpha = 0.2) of the mean luma of the 320x180 frame
    illum <= 99   ->  RGB door-bearing admissible   (vision may govern)
    illum >  99   ->  RGB door-bearing INADMISSIBLE (map axis governs alone)

Deployed threshold is 100, which is inside the measured separating gap and within one unit of
the optimum. The statistic is computed on the 320x180 stream the robot actually receives, not
on any higher-resolution capture, because the contract has to be evaluable at run time.

**The EMA is part of the statistic, not an implementation detail.** This is the substantive
finding and it revises §2.2 rather than merely confirming it:

| statistic, per frame | dark vs lit overlap |
|---|---|
| mean luma | 37% |
| normalised RMS contrast | 68% |
| grain (mean abs Laplacian) | 78% |
| Michelson contrast | 95% |

Per frame, nothing separates — including the contrast and grain that §2.2 nominated. The
robot's camera is in motion, so scene content (floor, wall, doorway, sofa) dominates any
single frame. Smoothed with the gate's own EMA, mean luma separates perfectly on this
material.

## Evidence

Labels are the operator's notebook (`SESSION_LOG_2026-08-21.md`), written at the time and
independent of any image statistic. Frames are the robot's own, in motion, on the scored
route. The 1587-byte placeholder frames the harness writes when the WebRTC channel delivers
nothing are excluded — they are not images.

| declared state | runs | frames | EMA range | gate active |
|---|---|---|---|---|
| all lights off, both rooms | 2 | 42 | 68.5 – 98.9 | 0% |
| main lab lit, office dark | 1 | 15 | 79.7 – 117.0 | 60% |
| all lights on | 1 | 18 | 101.6 – 114.1 | 100% |

False positives (vision suppressed while dark): **0%**.
False negatives (vision allowed to govern while lit): **0%**.
Best achievable threshold on this statistic: 99.0, at 0% irreducible error.

The mixed state straddling the threshold is not an error: the robot crosses from a lit room
into a dark one, and the gate is expected to change state during that crossing.

A separate, static sweep (`calib_luz/2026-08-21`, 85 frames, 8 tandas) gives a wider clean gap
of 24.8 luma units on the same statistic. It is reported as supporting evidence only, because
that sweep's declared states L1–L5 were never labelled in the data, so its partition had to be
derived from the statistic under test — circular, and not admissible as the primary evidence.

## Declared limitations

1. **The separating gap is narrow: 98.9 dark-max to 101.6 lit-min, 2.7 luma units.** It is
   clean on this material but thin. A different day, a different route, or different
   auto-exposure behaviour could close it. This is the contract's weakest point and it should
   be re-measured whenever the camera pipeline changes.
2. **n is small**: 4 runs on one day, 75 usable frames. The 21-Aug session is the only one with
   both declared light states and retained frames.
3. **No hysteresis.** The gate is a bare comparison (`illum_ema > LUX`). With a 2.7-unit gap,
   chatter near the boundary is plausible, and the mixed-state run shows the EMA crossing the
   threshold during a single traverse. Recommended before the confirmatory tier: separate
   entry and exit thresholds (e.g. suppress above 102, restore below 96) plus a minimum dwell,
   so a role transition needs evidence rather than a single crossing. Ω_t is unaffected either
   way — it takes the operator's declared switch state, not the camera.
4. **Only the door-bearing representative is gated.** The contract does not yet cover object
   detection admissibility generally, which is what the W2 pair proper needs.

## Reproduce

    python3 analysis/contrato_visual.py     # the whole chain, all four analyses

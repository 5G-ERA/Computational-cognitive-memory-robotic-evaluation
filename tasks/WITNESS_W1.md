# Matched witness W1 — free space vs lost lidar coverage

Format: the five elements a valid witness must record (paper V4.3, Supplementary Note 1,
§3.6). Status: **replay tier, real recordings, synthetic staging** (development material).
The physical twin of this witness is the glass panel of the reserved transition
configurations; the staging mechanism is identical (`SIM_GLASS` in the twin, declared-
rectangle erasure in replay).

## 1. The two admissible histories

Same recording (each of the 8 real runs of 2026-08-20, office A↔B through the door):

- **h_a** — the recording as captured. No coverage anomaly staged; the frozen reference
  prescribes the incumbent (`motion`) throughout.
- **h_b** — the same recording with lidar returns erased inside the declared world rectangle
  (−3.75, −0.55)…(−2.65, 0.75) — a wall patch on the door approach chosen as the most-
  predicted surface on these trajectories. The reference prescribes `lidar_coverage` while
  the frontal sector faces the erased patch (window derived from the declared rectangle and
  the recorded trajectory, not from robot output).

## 2. The common declared representation

Under the **original interface I⁰** (`vista()` erases everything outside global/local map +
current readings), h_a and h_b are represented identically at every decision boundary: the
erasure only changes coverage-evidence fields (`cov_missing`, `cov_def`, `cov_n`,
`cov_blind`), and none of them is in I⁰. A missing return and free space produce the same I⁰
value — the aliasing pair of the protocol (§4.1 of the protocol document).

## 3. Independently adjudicated judgement, decision, response set

| | h_a | h_b (while facing the patch) |
|---|---|---|
| Judgement | surfaces normally seen are present | predicted surfaces persistently absent |
| Decision δ_t | `motion` (incumbent governs) | `lidar_coverage` |
| Accepted responses | continue under payload-protecting motion | slow/replan under degraded spatial observation; governed review admissible |

The accepted response sets are disjoint in the strong sense used by Theorem S1: continuing at
incumbent speed is not an accepted response under prescribed coverage loss.

## 4. The omitted distinction

Prospective coverage evidence: whether surfaces that this robot reliably sees from this pose
are returning. Omitted by I⁰ (current readings carry the absence but not the expectation).
Exposed by I¹ as `cov_missing` — cells of the G1-own visibility reference predicted and
absent ≥2 consecutive snapshots (amendment §10 of the pre-registration).

## 5. The interface / unresolved status that preserves its effect

- **C4 (I¹, distributed)**: resolves `lidar_coverage`; the only arm that reaches the correct
  governed transition (delay 7.4–17 s across runs).
- **C3 (I¹, incumbent)**: answers `unresolved` — responsible, and the correct non-adoption
  outcome for a verifier that cannot reconstruct the alternative role.
- **C2 (I⁰, distributed)**: blind — `lidar_coverage→motion` ×43 in one run; fails through
  the interface, not through the resolver (the design requirement "prevent hidden history
  leakage" is what makes this attribution valid).
- **C1 (I⁰, incumbent)**: falsely retains (`lidar_coverage→retain` ×46).

Measured contrast over the 8 runs (unit = run, bootstrap 10k, seed 7): C4−C3 +7.7 pp
[+5.9, +9.5]; C4−C2 +7.4 pp [+5.0, +9.7]. Twin W1 gave the same direction (+5.7 pp).

## Open half

The physical witness with a real glass panel (reserved configurations, C4 vs C3 tier) has not
been run. W2 (object absence vs insufficient illumination) is half-built: lit ground truth
exists (chair at 2.0 m, 25/25 at 0.91); the dark twin is tonight's Block 2.

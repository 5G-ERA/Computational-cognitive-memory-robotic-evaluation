# Development campaign — the Isaac verification package

**Status:** complete, 24 Aug 2026. 3 reps × 11 stageable configurations (T12 needs no
staging; the 12 *reserved* configurations remain untouched — this is all development tier).
31 scoreable runs; the first-batch T1/T2 (staged with the wrong light, caught by the scorer)
were discarded and re-run at low light. Every run carries its reference certificate
alongside (`*_omega_ref.json`), written by the script in the act of staging.

## Primary outcomes: A_meta per configuration

|      | C1  | C2  | C3  | C4  | reps |
|------|-----|-----|-----|-----|------|
| T1   | 20% | 0%  | 15% | 34% | 3 |
| T2   | 59% | 0%  | 9%  | 29% | 3 |
| T3   | 16% | 0%  | 9%  | **87%** | 3 |
| T4   | 77% | 0%  | 12% | 56% | 3 |
| T5   | 85% | 0%  | 16% | 79% | 3 |
| T6   | 91% | 0%  | 13% | 57% | 3 |
| T7   | 17% | 0%  | 9%  | **67%** | 3 |
| T8   | 25% | 40% | 41% | 44% | 3 |
| T9   | 51% | 13% | 13% | 64% | 3 |
| T11  | 20% | 0%  | 1%  | 58% | 3 |
| T10  | 14% | 46% | 52% | 13% | 1 |

**Aggregate (8855 boundaries per condition):** C4 54% · C1 45% · C3 16% · C2 7%.
A_Ω equals A_meta throughout — single-ground stagings; the machinery separates them where a
right answer cites a wrong ground.

## Secondary outcomes (44 reference boundaries)

|      | adopted | delay med | return | unnecessary/min | false persist |
|------|---------|-----------|--------|-----------------|---------------|
| C1   | 18/44   | 0.0 s*    | 7/7    | 1.7 | 0.0 s |
| C2   | 7/44    | 0.0 s     | 1/7    | 0.0 | 0.0 s |
| C3   | 17/44   | 11.9 s    | 5/7    | 2.3 | 0.0 s |
| C4   | **39/44** | **0.8 s** | 6/7  | 7.3 | 0.0 s |

\* C1's zero delays are trivial adoptions (boundaries whose new set contains its constant).

## Readings that survive scrutiny

- **C4's advantage lives where the protocol says it should**: configurations demanding an
  affirmative reconstruction (T3 87%, T5 79%, T7 67%, T9 64%, T11 58%). It adopts 39 of 44
  boundaries at 0.8 s median and returns 6/7.
- **C1 wins where its constant is right by construction** (T4/T5/T6 return-and-motion-heavy
  configs, 77–91%) — which is why single configurations cannot measure the contrasts fairly
  and the full set exists.
- **T8 (joint insufficiency): C3 41% ≈ C4 44%** — abstention with rich information is
  competitive exactly where abstention is the answer. Predicted by §2.2 and measured.
- **T10 (no-use control): the abstainers "win" (C2 46%, C3 52%) and C4 scores 13%** — and
  no condition ever resolves no_use autonomously (D6). §10 anticipates this: permanent
  stopping can look safe without demonstrating continuity. The control is doing its job.
- **C4's unnecessary switching (7.3/min raw resolver)** is the sensitivity premium — the
  twin-side counterpart of the real two-sided ledger. The flown stabiliser removes 93% for
  +0.5 s declared.

## Declared residuals

- T1/T2 C4 at ~30%: the 2.0 m exigibility zone is wider than where cov_n actually collapses;
  instrument sensitivity vs zone-width mismatch, noted rather than tuned away.
- T10 has n=1 (consecutive blocked-door runs return empty — the robot starts at the door).
- Instrument constants declared and subtractable: contract EMA ~0.9 s, stabiliser +0.5 s.
- The C4−C1 aggregate gap (+9 pp) is diluted by motion-dominant derived deltas; the
  per-configuration table is the honest view.

## What this package is

Everything the confirmatory tier needs, running end to end in the twin: staging with
independent records, reference certificates, both primary outcomes, the §9.3 secondaries,
role stabilisation with declared constants, and a decision ledger (D1–D6) for Renxi's
return. What it is NOT: confirmatory data. The reserved configurations are untouched and
the §12 model is unfrozen.

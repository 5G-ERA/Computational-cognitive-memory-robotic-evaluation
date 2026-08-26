# Exclusion log (Note 8 §8.16 / §8.10)

Every run or record excluded from an analysis population, with reason and
disposition. Nothing is silently discarded; excluded material stays in `dataset/`
and in git history.

## Campaign v2 (the current development result set)

| Record | Reason | Disposition |
|---|---|---|
| `dataset/20260825_214225_ours_A.json` | Degenerate leg (2 s, robot already at goal after an interrupted predecessor); fails the pre-declared <30-samples filter | Kept in dataset; replaced by declared top-up `dataset/20260826_034818_ours_A.json` (recorded in the v2 manifest) |
| T10 (all reps) | Not stageable in the kinematic twin: a thin close obstacle evaporates from the belief inside the near-blind radius (ledger **D10**, safety implication declared) | Three diagnostic runs kept (26-Aug); configuration excluded from campaign tables; revisit under D10 or the walker tier (D9) |

## Campaign v1 (24-Aug, superseded by v2)

| Record | Reason | Disposition |
|---|---|---|
| First 6 T1/T2 manifest rows | Warm-up rows predating stable staging | Skipped by the scorer for that manifest only (`analysis/nivel_run.py`) |
| Entire campaign as primary evidence | Predates the validated glass witness (no live coverage reference; sub-resolution pane; pre-facing-gate certificates) | Kept and still scoreable offline; superseded by v2 as the coherent result set |

## Variance campaign (N=30, 25-Aug)

| Record | Reason | Disposition |
|---|---|---|
| `dataset/20260825_183155_ours_B.json` | Known empty-leg artifact (goto to current position) | Dropped by the <20-samples filter; noted in `tasks/VARIANZA_N30.md` |
| First N=30 attempt (22-Aug) | Ran under the undeclared-clock defect (~3.4× time compression) | Kept, labelled failure-mode exhibit; never scored |

## Development diagnostics (excluded from deployment-effect evidence by design, §8.4)

| Record | Reason | Disposition |
|---|---|---|
| 21-Aug real glass witness (first version) | **Solid-wall control invalidated the instrument** (ray-march staleness/aliasing read a visible wall as absent) | The successful diagnostic the paper cites; witness excluded, instrument redesigned (`cov_missing` v2, accumulated base, validated 25-Aug) |
| All staged twin runs | Development-stage by definition | Never counted as deployment-effect evidence |

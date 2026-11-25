# Physical Summit XL · frozen records of 10 November 2026

Three frozen records at the aligned pre-entry point that pass the valid-cut
manifest: `T04`, `T05`, `T06`. **Development data. Not classified, not
independently annotated or checked.** The limits are listed in
[`../README.md`](../README.md#known-limits-of-these-records).

## What each folder holds

| File | Content |
|---|---|
| `MANIFIESTO_CORTE.json` | The valid-cut manifest: ContextID, PairID, CutID, TraversalID, `t_aligned`, `t_freeze`, component timestamps, freshness, file hashes, software versions, perception-validity indicators, known limits |
| `salidas/registro.jsonl` | The chained record. The entry of type `frontera` holds the frozen cut: scan, pose and covariance, laser transform, doorway estimate and DKP projection |
| `salidas/fronteras.jsonl`, `salidas/resumen.json` | The cut as reported, and the shadow's summary |
| `T0N.json`, `T0N.json.acuse` | The alignment mark written by the traversal, and the shadow's acknowledgement |
| `sombra_…_cuenta.json` | A separate listener that counted commands during the run: zero from the shadow |
| `CONFIG.json`, `EPISODIO.json`, `SOMBRA_ARMADA.json`, `SOMBRA_EJECUCION.json` | Configuration and run metadata |

[`manifiesto_corte.py`](manifiesto_corte.py) is the script that wrote the manifests.

## The three records

| | T04 | T05 | T06 |
|---|---|---|---|
| ContextID | `summit_physical` | `summit_physical` | `summit_physical` |
| PairID | `A_a_B_nominal` | `A_a_B_nominal` | `A_a_B_nominal` |
| `t_aligned` to `t_freeze` | 0.88 s | 0.63 s | 0.89 s |
| Pose and scan age at freeze | 0.013 s | 0.024 s | 0.016 s |
| Commands from the shadow to the robot | zero | zero | zero |
| Shadow safety node before the cut | latched `SCAN_CADUCADO` | latched `SCAN_CADUCADO` | latched `SCAN_CADUCADO` |
| Localisation spread, position | 0.09 m | 0.12 m | 0.10 m |
| Localisation spread, heading | 0.067 rad | 0.086 rad | 0.091 rad |

The last three rows are stored indicators. No threshold was applied.

## Room conditions

Declared by the operator, not measured: blinds down, all lights on. Declared
after the fact for T04 and T05, and before T06. Not declared: whether anyone
stood near the door, whether the door moved.

## Not included

- The shadow safety node's full log and the executor's full log for each
  traversal (about 25 MB per record). Their SHA-256 hashes are in each manifest.
- The three attempts that produced no frozen record (`T01`–`T03`).
- The robot's traversal logs: they hold crossing times and clearances, which are
  not the object of this experiment.

Field names and log text inside the records are in Spanish.

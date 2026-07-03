# Meta-Reasoner 2.0 Usage Guide

## 1. Unzip

```bash
unzip meta-reasoner-2.0.zip
cd meta-reasoner-2.0
```

## 2. Run tests

```bash
python -m unittest -v test_meta_reasoner_2_0.py
```

Expected: `OK`.

## 3. Run examples

Open corridor:

```bash
python meta_reasoner_2_0.py --config config_meta_reasoner_2_0.json --input sample_inputs/open_corridor.json
```

Narrow door:

```bash
python meta_reasoner_2_0.py --config config_meta_reasoner_2_0.json --input sample_inputs/narrow_door.json
```

Unsafe:

```bash
python meta_reasoner_2_0.py --config config_meta_reasoner_2_0.json --input sample_inputs/unsafe.json
```

Battery-aware:

```bash
python meta_reasoner_2_0.py --config config_meta_reasoner_2_0_battery.json --input sample_inputs/low_battery_open.json
```

## 4. Generate calibration report

```bash
python meta_reasoner_2_0.py   --config config_meta_reasoner_2_0.json   --calibration calibration_meta_reasoner_2_0.json   --calibration-report reports/door_report.json
```

## 5. Python API

```python
from meta_reasoner_2_0 import MetaReasoner20

reasoner = MetaReasoner20("config_meta_reasoner_2_0.json")
output = reasoner.decide({
    "timestamp": 1,
    "readings": {
        "progression": {"value": 1.2, "reliability": 0.95, "uncertainty": 0.01},
        "safety": {"value": 1.2, "reliability": 0.95, "uncertainty": 0.01},
        "fragility": {"value": 1.0}
    }
})
print(output.to_dict())
```

## 6. Evaluation modes

Disable analogy-level DST:

```json
"evaluation_controls": {
  "analogy_level_dst": {"enabled": false},
  "task_level_dst": {"enabled": true}
}
```

Disable task-level DST:

```json
"evaluation_controls": {
  "analogy_level_dst": {"enabled": true},
  "task_level_dst": {"enabled": false}
}
```

Disable both for a deterministic baseline.

## 7. Output actions

- `KEEP`: continue current analogy.
- `SWITCH`: switch to `switch_to` analogy.
- `FALLBACK`: no deployable analogy.
- `HELP`: hard-veto dangerous state.
- `INSUFFICIENT`: missing non-negligible grounding.

## 8. Key output fields

- `local_analogy_tension`: max ratio-adjusted local tension.
- `task_projected_tension`: raw max base tension over task-positive dimensions.
- `task_stable_fulfillment`: uncertainty-penalised ranking value.
- `required_meta_failures`: required task dimensions that failed.
- `rejection_reason`: why a candidate was rejected.

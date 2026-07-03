# Meta-Reasoner 2.0 Configuration Parameter Guide

This guide explains every major configuration block in `config_meta_reasoner_2_0.json`.

## 1. `runtime_config`
Controls semantic memory for region-direction inference.

```json
"runtime_config": {
  "frequency_hz": 2.0,
  "semantic_memory_window_seconds": 5.0,
  "semantic_memory_window_ticks": null,
  "direction_deadband": 0.02,
  "direction_reference": "median_window",
  "first_tick_direction": "stable"
}
```

- `frequency_hz`: expected call frequency.
- `semantic_memory_window_seconds`: time span used to infer improving/stable/degrading direction.
- `semantic_memory_window_ticks`: fixed tick window; if `null`, it is computed from frequency × seconds.
- `direction_deadband`: value changes below this are treated as stable.

## 2. `evaluation_controls`
Controls DST ablation for evaluation.

```json
"evaluation_controls": {
  "analogy_level_dst": {"enabled": true},
  "task_level_dst": {"enabled": true}
}
```

- Disable `analogy_level_dst` to use current-region-only parameter scoring.
- Disable `task_level_dst` to use current fulfillment without uncertainty penalty/gap gate.
- Tension gate, required-meta gate and hard veto remain active unless explicitly changed in code.

## 3. `global_sensor_reliability`
Defines analogy-level DST uncertainty defaults.

```json
"safety": {
  "default_reliability": 0.95,
  "reliability_sensitivity": 0.12,
  "default_uncertainty": 0.01
}
```

Effective margin:

```text
effective_margin = runtime_uncertainty + reliability_sensitivity × (1 - reliability) + default_uncertainty
```

Then:

```text
belief_value = value - margin
current_value = value
plausibility_value = value + margin
```

## 4. `analogy_tension_model`
Tension is derived after DST region reasoning. It is a semantic concern index, not probability.

```json
"region_direction_tension": {
  "adaptive:stable": 0.40,
  "adaptive:towards_high_concern": 0.55,
  "high_concern:towards_dangerous": 0.85
}
```

Ratio adjustment:

```text
attention_ratio = analogy_attention / max_analogy_attention
effective_ratio = attention_ratio ^ exponent
adjusted_tension = base_tension × effective_ratio
local_analogy_tension = max adjusted_tension
```

Use exponent `1.0` in expected/adaptive states, `0.5` in high-concern states, and `0.25` for near-danger early warning states.

## 5. `analogies`
Each analogy defines:

```text
meta_attentions
qoe
sensor_reliability_override
hard_veto
```

Example:

```json
"Efficient_Nav": {
  "meta_attentions": {"progression": 0.65, "safety": 0.35},
  "qoe": {
    "progression": {"expected": 1.0, "adaptive": 0.6, "dangerous": 0.1},
    "safety": {"expected": 1.0, "adaptive": 0.8, "dangerous": 0.5}
  },
  "hard_veto": {"safety": true, "progression": true}
}
```

## 6. `task_information`
Important task fields:

```json
"task_tension_threshold": 0.50,
"task_fulfillment_threshold": 0.50,
"task_required_meta_thresholds": {"progression": 0.10}
```

A tension threshold of `0.50` allows `adaptive:stable = 0.40` and rejects `adaptive:towards_high_concern = 0.55`.

## 7. Task fulfillment flexibility

```json
"task_fulfillment_flexibility": {
  "enabled": true,
  "band": 0.05,
  "borderline_policy": "use_switch_persistence"
}
```

With threshold `0.50` and band `0.05`:

```text
F >= 0.50        clear pass
0.45 <= F < 0.50 borderline
F < 0.45         fail
```

## 8. `task_dst`
Task-level uncertainty.

```json
"task_dst": {
  "enabled": true,
  "uncertainty_penalty": 0.80,
  "maximum_uncertainty_gap": 0.30,
  "ranking_basis": "stable_fulfillment"
}
```

```text
task_uncertainty_gap = max parameter uncertainty gaps over task-positive dimensions
task_stable_fulfillment = current_fulfillment × (1 - uncertainty_penalty × task_uncertainty_gap)
```

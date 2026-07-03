# Meta-Reasoner 2.0 Output Contract: SWITCH Action

## Is `switch_to` implemented?

Yes. In Meta-Reasoner 2.0, when the output action is `SWITCH`, the target analogy is given explicitly in:

```json
"switch_to": "Cautious_Nav"
```

The same target is also reflected in:

```json
"active_after": "Cautious_Nav"
```

Therefore, downstream code should not infer the switched analogy from the reason text or from candidate scores. It should read the `switch_to` field directly.

---

## Output contract

### SWITCH

When:

```json
"action": "SWITCH"
```

then:

```json
"switch_to" != null
"active_after" == "switch_to"
```

Example:

```json
{
  "action": "SWITCH",
  "active_before": "Efficient_Nav",
  "active_after": "Cautious_Nav",
  "switch_to": "Cautious_Nav",
  "reason": "Decision completed."
}
```

Controller rule:

```python
if output["action"] == "SWITCH":
    target_analogy = output["switch_to"]
    activate_analogy(target_analogy)
```

---

### KEEP

When:

```json
"action": "KEEP"
```

then:

```json
"switch_to" == null
"active_after" == "active_before"
```

Example:

```json
{
  "action": "KEEP",
  "active_before": "Efficient_Nav",
  "active_after": "Efficient_Nav",
  "switch_to": null
}
```

Controller rule:

```python
if output["action"] == "KEEP":
    continue_current_analogy()
```

---

### FALLBACK, HELP, INSUFFICIENT

For non-analogy-transition actions:

```json
"switch_to" == null
```

Examples:

```json
{
  "action": "HELP",
  "active_before": "Efficient_Nav",
  "active_after": "Efficient_Nav",
  "switch_to": null
}
```

```json
{
  "action": "FALLBACK",
  "active_before": "Efficient_Nav",
  "active_after": "Efficient_Nav",
  "switch_to": null
}
```

---

## How the reasoner selects `switch_to`

The reasoner follows this logic:

```text
1. Score every analogy.
2. Remove non-deployable analogies.
3. Select the deployable analogy with highest task_stable_fulfillment.
4. If selected analogy differs from active_before:
       action = SWITCH
       switch_to = selected analogy
       active_after = selected analogy
5. Otherwise:
       action = KEEP
       switch_to = null
       active_after = active_before
```

Simplified pseudocode:

```python
deployable = {
    aid: score
    for aid, score in candidate_scores.items()
    if score["deployable"]
}

candidate = max(
    deployable,
    key=lambda aid: deployable[aid]["task_stable_fulfillment"]
)

if candidate != active_before:
    output["action"] = "SWITCH"
    output["switch_to"] = candidate
    output["active_after"] = candidate
else:
    output["action"] = "KEEP"
    output["switch_to"] = None
    output["active_after"] = active_before
```

---

## Recommended downstream handling

Use this robust controller pattern:

```python
action = output["action"]

if action == "SWITCH":
    assert output["switch_to"] is not None
    target = output["switch_to"]
    activate_analogy(target)

elif action == "KEEP":
    assert output["switch_to"] is None
    continue_current_analogy()

elif action == "FALLBACK":
    enter_fallback_controller()

elif action == "HELP":
    stop_and_request_help()

elif action == "INSUFFICIENT":
    report_missing_grounding_or_capability()
```

---

## Example: Door-crossing SWITCH

In a narrow-door case, the expected output may be:

```json
{
  "action": "SWITCH",
  "active_before": "Efficient_Nav",
  "active_after": "Cautious_Nav",
  "switch_to": "Cautious_Nav",
  "reason": "Decision completed.",
  "candidate_scores": {
    "Efficient_Nav": {
      "deployable": false,
      "rejection_reason": "fulfillment_threshold_failed",
      "task_projected_tension": 0.85,
      "task_stable_fulfillment": 0.392
    },
    "Cautious_Nav": {
      "deployable": true,
      "rejection_reason": null,
      "task_projected_tension": 0.40,
      "task_stable_fulfillment": 0.586
    },
    "Search_Lid": {
      "deployable": false,
      "rejection_reason": "fulfillment_threshold_failed",
      "task_projected_tension": 0.00,
      "task_stable_fulfillment": 0.115
    }
  }
}
```

This means:

```text
The active analogy was Efficient_Nav.
The selected deployable analogy is Cautious_Nav.
The controller should switch to Cautious_Nav.
```

---

## Validation rule for tests

Add this assertion to tests:

```python
if output["action"] == "SWITCH":
    assert output["switch_to"] is not None
    assert output["active_after"] == output["switch_to"]
else:
    assert output["switch_to"] is None
```

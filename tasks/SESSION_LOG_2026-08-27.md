# Session log — 2026-08-27

Appended by `tools/bitacora.py` after each run. Measured facts come from the
dataset; the **operator note** is the part the instrumentation cannot see —
arm contact, spill, a run stopped by hand. `ncol = 0` does not mean clean.

---

## 15:12 → B  ·  `20260827_143446_ours_B`

| | |
|---|---|
| Outcome | **arrived** |
| Arm / flags | `SMOKE` · METASM=1 DOOR_CTR2=1 DOOR_CTR_HOLD=0.15 DOOR_YAW2=1 DOOR_EXIT_CTR=1 DOOR_CTR_TOL=0.07 DOOR_VIS=1 |
| Duration | 53 s |
| Final distance to goal | 0.38 m |
| Collisions (detected) | 0 |
| Door crossings | 0 |
| Lateral offset at the gap | +0.036 m |
| Battery | 84% → 83% |
| META states | NORMAL 91%, BLIND 9% |
| Coverage deficit | median 0.955, p90 1.000, max 1.000 |
| Phases | DWA-F×115, ENG-GO×49, ENG-F×21, ENG-CG×3, ENG-RG×3, ENG-T×2 |


---

## 15:16 → B  ·  `20260827_151354_ours_B`

| | |
|---|---|
| Outcome | **did not arrive** |
| Arm / flags | `CAL_LIT_2` · METASM=1 DOOR_CTR2=1 DOOR_CTR_HOLD=0.15 DOOR_YAW2=1 DOOR_EXIT_CTR=1 DOOR_CTR_TOL=0.07 DOOR_VIS=1 |
| Duration | 75 s |
| Final distance to goal | 6.59 m |
| Collisions (detected) | 1 |
| Door crossings | 0 |
| Battery | 68% → 68% |
| META states | NORMAL 73%, BLIND 27% |
| Coverage deficit | median 1.000, p90 1.000, max 1.000 |
| Phases | DWA-F×146, DOOR-GO×38, BRK-TR×27, DOOR-CTR×18, DWA-T×15, ESC-BK×9 |

**Operator:** robot's head fully up, confused after the start and couldnt go to the destination

---

## 15:23 → B  ·  `20260827_152159_ours_B`

| | |
|---|---|
| Outcome | **did not arrive** |
| Arm / flags | `CAL_LIT_2` · METASM=1 DOOR_CTR2=1 DOOR_CTR_HOLD=0.15 DOOR_YAW2=1 DOOR_EXIT_CTR=1 DOOR_CTR_TOL=0.07 DOOR_VIS=1 |
| Duration | 16 s |
| Final distance to goal | 5.76 m |
| Collisions (detected) | 0 |
| Door crossings | 0 |
| Battery | 65% → 65% |
| META states | NORMAL 100% |
| Coverage deficit | median 1.000, p90 1.000, max 1.000 |
| Phases | DWA-F×55, BRK-BK×2, BRK-TR×1 |

**Operator:** robots head lifted so the vision 1 meter away is at the level of the height of the robot, after executing the AtoB command, robot moved only 50cm and decided to stop

---


# Golden door crossing — 14 Aug 2026

**The frozen reference for crossing the door B→A.** Tag `golden-doorcross`, branch
`feature/door-centring-rate`. Everything here sits behind flags that are **off by default**, so
checking the tag out and running the old command reproduces the old behaviour exactly. *The
configuration is the result — not the code alone.* Use the command below verbatim.

## The command

```bash
cd ~/Documents/G1_UNITREE_ROBOT_META_REASONING && source ~/g1env/bin/activate
G1_DOOR_CTR2=1 G1_DOOR_CTR_HOLD=0.15 G1_DOOR_YAW2=1 G1_DOOR_EXIT_CTR=1 G1_DOOR_CTR_TOL=0.07 \
G1_SETUP="cup=hand,apron=off,fill=dry" G1_ARM=CTR2YE G1_META2=2 \
G1_M2_CFG=config_meta2_g1payload.json G1_M2_STATE=meta2_state_ctr2.json \
G1_M2_STATE_INIT=meta2_state_twin_explored.json G1_M2_REALCAL=1 G1_M2_L3=1 G1_M2_L4=1 \
G1_M2_PROFILES=1 G1_M2_DOORLIB=1 G1_M2_FRAGSPEED=1 G1_DOOR_VIS=1 G1_PERC=127.0.0.1:8008 \
python3 g1_goto.py gotoviz A
```

Abort thresholds stay at their **defaults (75 / 0.4 / 8)**. Passing larger values inflates the
arrival rate and makes results incomparable — that has already happened once on this project.

## What it is made of, and why each piece exists

Three changes, each with its own flag, each answering a failure the previous one exposed.

| Flag | What it does | The failure it fixes |
|---|---|---|
| `G1_DOOR_CTR2=1` + `HOLD=0.15` | Strafe and advance in the **same command**; more than 0.15 m off the axis, strafe only | Runs entered the gap ~0.23 m off-axis and caught the frame. Successes crossed at ≤0.14, failures at −0.23 — a clean split. The earlier attempt to fix this by tightening the tolerance failed, because that changes *when* the correction triggers, not how much of the time it is active |
| `G1_DOOR_YAW2=1` | Turn **while advancing** when centred and under 25° of heading error | With centring fixed, the robot sat *inside* the gap oscillating: advance → yaw drifts past 14° → stop to realign → **drift backwards** → repeat. Eight realignments, never reached the 0.75 m that counts as crossed, killed by the progress watchdog |
| `G1_DOOR_EXIT_CTR=1` | Past the gap centre, **ignore the vision-measured door centre** and servo to the map axis | It crossed, but the **left arm scraped the frame**. It entered clean and drifted to −0.19 on the way out *while believing it was centred*: the measured centre comes from detecting the two jambs, and once inside the gap the jambs sit in the LiDAR's blind band. Measured bias when the guard fired: **−0.11 m**, twice |

## Evidence (14 Aug, real robot, B→A)

| Config | Runs | Arrived | Time | Lateral offset on exit |
|---|---|---|---|---|
| Before (tight tolerance only) | 7 | 3 (43%) | 56–132 s | −0.10 to −0.26 |
| `CTR2` only | 3 | 1 | 97–105 s | −0.10 to −0.26 |
| `CTR2+YAW2` | 2 | 2 | 50–55 s | −0.07 |
| **All three** | 3 | 3 | 57–92 s | −0.07 to **+0.02** |

The last two configurations together: **5 of 5 arrivals, one collision**, while the battery fell
from 52% to 46% — so the improvement is not a fresh-battery artefact.

## What this is NOT

- **n = 5, one session, no interleaved control arm.** The comparison is against the same robot on
  the same afternoon, which is honest but is not an A/B. Before treating the rate as established,
  interleave runs with the flags off.
- **`ncol = 0` does not mean clean.** The collision detector runs on odometry and IMU; a light arm
  scrape does not perturb the base. One arrival scored zero collisions *and* touched the frame —
  the operator saw it, the instrumentation did not. **The arms are the widest part of the robot and
  the least instrumented.** Until that is fixed, a human has to watch the crossing.
- One of the three full-config runs took 92 s and had a collision. It is not uniform yet.

## Open items this leaves

1. Instrument arm contact, or stop using `ncol` as the safety metric for door crossings.
2. Decide whether these flags become defaults. They are off today on purpose — defaults reproduce
   previous behaviour — but a reference configuration nobody enables is one that quietly rots.
3. The measured door centre carries a real bias (−0.11 m). It is now bypassed on exit; whether it
   should be trusted on the *approach* has not been tested.

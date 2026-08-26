# Lab session — 20 Aug 2026: prerequisites, not protocol data

**Label this session DEVELOPMENT.** Nothing here is confirmatory: the reserved transition
configurations are not yet defined, Ω_t does not exist, and C1–C3 are not wired into the robot.
What this session produces are the things that are *blocked without the robot* and that block
everything else.

Branch: **`feature/dcc-integration`** (merged object level + META interface, twin-validated 6/6).

| Block | Why it needs the robot | Blocks |
|---|---|---|
| **D. Re-baseline** | The merge crosses the door in the twin; in the office it is unverified | Everything |
| **C. First real `METASM=1` runs** | 0 of 300 real runs carry the META fields | Pre-registration limitation 4 |
| **B. Real glass** | `cov_def` is validated against *simulated* glass only | W1 |
| **A. Light calibration** | Mean luminance is AGC-pinned; the responding statistic is unknown | W2, and its contract |

Battery: ~1 point per run, **stop walking runs at 60%** (strafe rate halves by 46%). Do the
walking blocks first, the static ones last.

---

## D. Re-baseline the merged branch — 2 runs

```bash
cd ~/Documents/G1_UNITREE_ROBOT_META_REASONING && git fetch -q origin && git checkout feature/dcc-integration && git pull -q --no-rebase && git log --oneline -1
```

> **After checkout `waypoints.json` and `nav_map.json` are no longer in the repo root — they moved
> to `data/`.** Contents are byte-identical; only paths changed. Any script of yours pointing at
> the old path will break.

Then, twice (A→B, then B→A):

```bash
cd ~/Documents/G1_UNITREE_ROBOT_META_REASONING && source ~/g1env/bin/activate
G1_DOOR_CTR2=1 G1_DOOR_CTR_HOLD=0.15 G1_DOOR_YAW2=1 G1_DOOR_EXIT_CTR=1 G1_DOOR_CTR_TOL=0.07 G1_SETUP="cup=hand,apron=off,fill=dry" G1_ARM=REBASE G1_META2=2 G1_M2_CFG=config_meta2_g1payload.json G1_M2_STATE=meta2_state_rebase.json G1_M2_STATE_INIT=meta2_state_twin_explored.json G1_M2_REALCAL=1 G1_M2_L3=1 G1_M2_L4=1 G1_M2_PROFILES=1 G1_M2_DOORLIB=1 G1_M2_FRAGSPEED=1 G1_DOOR_VIS=1 G1_PERC=127.0.0.1:8008 python3 g1_goto.py gotoviz A
```

**Pass criterion:** crosses (`DOOR-ENG CROSSED` in the log) with lateral offset at the gap inside
|0.14|. **If it does not, stop the session and tell me** — the merge changed real behaviour and
nothing further is worth running.

---

## C. First real runs with the META layer — 4 runs

Same command, adding `G1_METASM=1 G1_ARM=METAREAL`, alternating A→B / B→A.

**What to look for**, and it matters more than the arrival:

- `meta_state` must appear in the dataset. Expect **BLIND to dominate** — in the twin it occupies
  39–56% because its predicate is `c0 < 1.0`, i.e. *proximity*, not blindness. Confirming that on
  the real robot settles whether the state is misnamed or genuinely different in the office.
- `laser_trust` should sit at 1.00 unless there is a collision. It only falls *after* being proven
  wrong; if it moves without a collision, my reading of it is wrong and I want to know.
- `cov_def` and `cov_blind` should be present on every sample.

---

## B. Real glass — the W1 witness

`cov_def` rose from p90 0.125 → 0.500 against simulated glass in the twin, and stayed flat where
the glass was not in view. Real glass is the actual witness.

1. Pick a **glazed section that exists in the reference map as wall** (a window, or the glazed part
   of a door). Note its approximate map coordinates.
2. Place the robot ~2 m from it, facing it. Record ~30 s stationary:

```bash
cd ~/Documents/G1_UNITREE_ROBOT_META_REASONING && source ~/g1env/bin/activate
G1_ARM=GLASS G1_METASM=1 G1_PERC=127.0.0.1:8008 python3 g1_goto.py turntest
```

3. Repeat facing a **solid wall at a similar range** — the control.

Expected: high `cov_def` facing glass, ~0 facing the solid wall. If both are high, the field is
measuring something else and W1 is not yet supported by real evidence.

---

## A. Light calibration → the visual-quality contract

Static, low battery cost, do it last. Purpose: find which image statistic tracks illumination
(mean does **not** — the camera's auto-exposure pins it near 105 all day) and where detection
becomes inadmissible.

**Setup.** Robot stationary. The **transport crate** at ~1.5 m, in view, unmoved for the whole
block — it is the known object whose detectability we are measuring.

**Sweep**, ~30 s recording at each declared state, written down in order:

| # | Declared state |
|---|---|
| L1 | All lights on, blinds open |
| L2 | All lights on, blinds closed |
| L3 | Lights off, blinds open (daylight only) |
| L4 | Lights off, blinds closed (darkest achievable) |
| L5 | Back to L1 (repeat, to check the sweep is reversible) |

For each state record with the perception server running, so we capture **both** the frames and the
detector's output:

```bash
cd ~/Documents/G1_UNITREE_ROBOT_META_REASONING && source ~/g1env/bin/activate
G1_ARM=LIGHT_L1 G1_PERC=127.0.0.1:8008 G1_METASM=1 python3 g1_goto.py turntest
```

Change `L1` per state. **Write down the wall-clock time of each state change** — that is the
independent record, and it is what Ω_t will use, not anything derived from the camera.

**What I do with it afterwards:** compute mean, contrast and grain per frame against the declared
state, find the statistic that separates them, and pair it with the detector's confidence on the
crate to fix the admissibility boundary. That pair — responding statistic + detection boundary — is
the visual-quality contract, and W2 is blocked until it exists.

---

## Do not do tomorrow

- Do not stage the twelve transition configurations. They are not defined, and staging them now
  would spend reserved material on development.
- Do not tune anything on the door. The object level is frozen at `golden-doorcross`; any further
  tuning happens on development stagings only.

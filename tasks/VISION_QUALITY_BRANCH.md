# feature/vision-quality — the post-W2 vision package

Branch off `feature/dcc-integration`, created the night of 20 Aug 2026. **Deployment gate: do
NOT merge or run this on the robot until the W2 pair (lit + truly-dark chair ground truth) is
captured with the frozen instrument** — the 25/25 @ 0.91 lit half was measured with the
pipeline as it stands on the session branch, and the pair must be internally consistent.

## Evidence base (all measured, 20 Aug frame audit — 251 frames re-queried)

- Traverse frames: ~4 KB, Laplacian sharpness 52, median 0 detections — identical moving vs
  stopped, so not motion blur: **the WebRTC stream collapses under navigation load**. Static
  sampling: 7 KB, 476, chair 25/25 @ 0.90. Same 320×180 both.
- The floor-clear vision gate is effectively dead during navigation: soft frames → floor mask
  everywhere → empty virtual scan → `free_center` None (server) → gate never engages.
  "Everything is floor" and "I see nothing" were indistinguishable from outside.
- 3-frame detection persistence doubles weak-class recall on traverses
  (couch 18→36%, 33→67%, 25→50%; chair already 100% inside its visibility window).
- Chair range: median 2.00 m returned vs 2.0 declared — intrinsics are fine for the
  320×180 frames actually processed.

## Changes in this branch

| Where | What | Risk |
|---|---|---|
| `perception_server.py` | GPU path additionally emits `floor_pct` (fraction of frame classified floor) so empty-scan frames are diagnosable | Emission-only |
| `g1_goto.py` | `dets_p3`: per-sample detections aggregated over the last 3 perception responses `[label, best conf, bearing, range, n_responses]` | Emission-only |
| `dcc_roles.py` | `dets_p3` added to I¹; `_det_fiable` prefers the window and requires `n ≥ 2` (sustained presence, not a flicker); falls back to instantaneous dets on legacy data; I⁰ unchanged | **Resolver-contract change** — needs a pre-registration note before confirmatory use |
| `g1_nav_v2.py` | `CAM_JS` width/quality parametrized (`G1_CAM_W`, `G1_CAM_Q`), defaults exactly the historic 320/0.5 | Default-inert; raising quality fattens every eval over the already-congested channel, so only for a declared A/B |

## Validation plan (before merge)

1. Twin traverses (the camera there is synthetic, so this validates only that the new fields
   emit and nothing regresses behaviourally): 2 runs, golden flags.
2. Offline: re-score the 20-Aug day with `dets_p3` derived from film frames — expect the
   object-question false-trigger rate to drop (the window kills single-frame flickers).
3. One real A/B block (post-W2): same traverse with and without `G1_CAM_Q=0.8`, judged on
   sharpness + detection rate + channel health (`scan_fresh` streaks must not worsen).

## Explicitly out of scope here

The object-question threshold itself (`c0_hard − c0`) — that is Renxi's contract decision,
tracked in the pre-registration amendment §10.

## The look maneuver (`G1_LOOK=1`) — perception-aware navigation

Adrián's idea (21 Aug): vision should improve if the robot slows and stops. The audit
**corrects the premise in one point and keeps the strategy**: physically stopping does NOT
help by itself — traverse frames measure sharpness 50 stopped vs 52 moving, because what
collapses is the WebRTC channel under the navigation loop's load, not the camera. So the
maneuver stops *and unloads the channel*:

1. Trigger: a weak candidate in the `dets_p3` window (`n == 1`, conf ≥ 0.35, ≤ 3.5 m — a
   flicker worth confirming), or periodic with `G1_LOOK_PERIODIC=1`. Never within 1.8 m of
   the door, never in ENG/ESC/recovery/ASSIST phases, cooldown `G1_LOOK_EVERY` (8 s).
2. Ramped stop (the slew limiter shapes it), phase `LOOK`, total `G1_LOOK_HOLD` (2 s).
3. **Channel quiet** for `G1_LOOK_WARM` (1.2 s): no cloud polling, no perception submits —
   the ABR gets headroom to recover bitrate.
4. One native-resolution JPEG-0.85 capture (`CAM_HQ_JS`), submitted to the perception
   worker, saved as `<run>_lookNN.jpg`, event `look` with the stream's `videoWidth×Height`.

DCC reading: this is an **authorised realisation of `review`** — the meta level halts
object-level progress to acquire decision-critical evidence. `phase_sent` = `LOOK` maps to
authority `meta` in the branch's `authority_of()`.

**Twin-validated** (periodic mode, synthetic perception `G1_SIM_PERC=1 G1_NOVIS=0
G1_PERC=127.0.0.1:8010`): 4 looks with HQ capture + photo per traverse, 80 actuated LOOK
samples, arrival, 0 collisions, door untouched (looks correctly suppressed during the
crossing). The flicker trigger cannot fire in the twin (synthetic perception only emits door
detections, and the door zone is excluded) — its semantics are unit-tested only.

**The gate for the whole strategy is measurable on the first real run**: compare the
sharpness of `lookNN.jpg` frames against the same run's film frames. If channel quiet does
not recover sharpness (film ≈ 50, static ≈ 476 — anything meaningfully above film validates),
the maneuver buys nothing and is dropped. This A/B costs one traverse with `G1_LOOK=1
G1_LOOK_PERIODIC=1 G1_FILM=1`, post-W2.

## Offline model bench (morning of 21 Aug, 202 traverse frames, frozen ruler)

Ruler declared before testing: presence windows per run (bounded by sightings ±2 s — relative
recall only, limitation declared), the 25-frame chair ground truth, the door bearing **by
geometry** (recorded pose + door centre: strong truth), and an FP proxy (chair/couch
detections outside every window; same yardstick for all methods, ranking only).
Harness: `analysis/banco_vision.py` (A = preprocessing, B = gate×window sweep, C = YOLO-World).

| Method (in-window recall) | chair % | couch % | FP proxy % |
|---|---|---|---|
| Current server (YOLO11x, gate 0.45, single frame) | 56 | 32 | 1.9 |
| + upscale/unsharp/CLAHE preprocessing | 50–56 | **0–21** | 0.6 |
| YOLO11x, gate 0.35, single frame | 67 | 36 | 1.9 |
| **YOLO11x, gate 0.35 + 3-frame window** | **83** | **64** | 6.8 |
| YOLO-World x, gate 0.30, single frame | 72 | 61 | 11.7 |

Three measured verdicts:

1. **Preprocessing hurts.** Sharpening/CLAHE amplify the compression artifacts of the soft
   channel and destroy the weak class (couch 32→0–21%). Discarded by measurement.
2. **The winning package needs no new model**: drop the server det gate 0.45→0.35 and use the
   3-frame window already built in this branch. Nearly doubles recall (56→83 / 32→64) at an
   FP proxy of 6.8% — and the resolver's `n ≥ 2` rule is stricter than the union metric used
   here, so effective FP control is better than the table shows.
3. **YOLO-World is not worth the switch**: better single-frame than current, worse than the
   gate+window package on everything, 6× the FPs — and **0/48 on doors** where geometry
   guarantees the door is in frame. The heuristic door channel stays.

Deployment note: the det gate also feeds the camera-clamp obstacle logic (`YOLO_FURNITURE`),
so 0.45→0.35 can change navigation behaviour — it ships with this branch's A/B validation,
never as a quiet default change. Launch flag: `--det-conf 0.35`.

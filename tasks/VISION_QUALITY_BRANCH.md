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

# Night session runbook — 2026-08-21 (after dark)

Development session. Goal: the **real-darkness half of the W2 pair**, the **per-session
coverage reference**, and the online distribution needed to pick `K_online`. The 12 reserved
transition configurations are NOT staged tonight.

**Standing rules.** Vision sampling and navigation are mutually exclusive (one debug client
per page). Log every run with `python3 tools/bitacora.py "<operator note>"` — the note is
mandatory. Darkness is verified **by measurement, not by the light switch** (lesson of Aug 20:
daylight at 17:15 made lights-off unmeasurable).

## Block 0 — setup (lights still on)

1. Perception server (tmux session `perc`), with the floor overlay ON so the operator sees it:

       PYTHONPATH=/home/ros/g1env/lib/python3.10/site-packages /usr/bin/python3.10 \
         perception_server.py --debug --floorcolor ...

   Check the web view shows frames AND the floor-segmentation overlay.
2. Calibration laps for the session coverage reference (2 laps A→B→A, no staging, dense
   snapshots, no session ref yet):

       G1_LASER_SNAP=0.5 <golden flags> python3 g1_goto.py <B> ; <A>

3. Build tonight's reference and use it for every later run:

       python3 tools/mapa_visibilidad.py --desde 20260821 --hasta 20260821 --reales \
         --min-opp 15 --min-runs 2 --salida dataset/visibilidad_ses_20260821.json

   (`--reales` matters: real and twin runs share the dataset and can share a date; the date
   filter alone does not separate them — caught in the 20-Aug twin rehearsal.)

       export G1_COVREF=dataset/visibilidad_ses_20260821.json

## Block 1 — lit baseline (2 runs)

Golden flags + `G1_COVREF` set. Purpose: online `cov_missing` distribution under normal,
same-session conditions → this picks `K_online` (replay K=4 was tuned on accumulated-map
evidence; the online field sees instantaneous scans and needs its own threshold).
Also: static vision sample pair, label `"luces encendidas"` (5 samples), chair at 2.0 m.

## Block 2 — dark (the point of the night)

1. Lights off. **Verify darkness by measurement**: `python3 tools/vision_sample.py "luces
   apagadas, noche" --n 5` → grain must rise clearly over the lit baseline (degradation model
   predicts the YOLO break around grain 65–87; that threshold is PROVISIONAL and tonight
   confirms or refutes it — if the night disagrees, the night wins).
2. Chair ground-truth dark twin: chair at 2.0 m exactly as Aug 20 (25/25 at 0.91 lit),
   `--n 25`, label `"luces apagadas, noche, silla 2m"`. This completes the W2 pair.
3. Dark traverses (2–3 runs, golden flags + `G1_COVREF`), bitácora after each.
4. Floor overlay vs YOLO under real darkness (Adrián's hypothesis Aug 20: light hits the
   floor overlay harder than YOLO; the degradation model says the OPPOSITE — floor cells
   stable 32–34 at all levels, YOLO 0.91→0.26). Judge on the dark chair samples + web view.

## Block 3 — checks before closing

- `free_center` during dark runs: it sat at 1.00 through every simulated degradation level —
  if it never moves tonight either, it is a blind spot to flag, not evidence.
- Online `cov_missing` distribution (lit vs dark, from tonight's samples) → freeze `K_online`.
- Dump bitácora + `calib_luz/` copies; leave the dataset untouched (non-rewriting).

## What tonight does NOT claim

No confirmatory outcome. Everything feeds the frozen contracts (visual-quality statistic,
`K_online`, session-reference procedure) that the pre-registration needs before the 12
reserved configurations are run.

## Rehearsed in the twin, night of 20 Aug — findings that change tomorrow

The full choreography ran end-to-end in the twin (calibration laps → session reference →
`G1_COVREF` live emission → staged `SIM_GLASS` on the same declared rectangle as the replay
validation → bitácora → guion → scoring). Machinery verdict: **everything runs**. Five
findings, all now versioned:

1. **`--reales` is mandatory when building the session reference** (real and twin runs share
   the dataset and can share a date; the date filter alone does not separate them).
2. **Live emission works and the session reference is dramatically cleaner than the
   historical map**: 447/447 samples carried `cov_missing`; normal traverse gave {0:×401,
   1:×46}, staged glass peaked at 4 exactly while facing the patch. **Recommended
   `K_online = 3`** (`G1_DCC_COV_MISSING=3` when scoring online-emitted fields) — confirm
   against tomorrow's real lit-baseline distribution before freezing.
3. **Per-tick strict-identity scoring dilutes the contrast at high sample rates** (twin
   samples at ~9 Hz): the per-cell evidence is intermittent tick to tick, so C4 only hit ~25%
   of the staged window raw. A trailing 2-s evidence window (scorer `--cov-ventana 2`) is
   legitimate I¹ construction (historical readings; I⁰ arms cannot do it) and fixes the
   in-window misses, but smears the boundaries of the Ω window.
4. Therefore: **the primary reading is at EPISODE level, as the approved protocol already
   says** (unit = run; correct authorised realisation and renewal; switching delay as
   efficiency). Rehearsal episode read: C4 1/1 correct governed transition; C1 false
   retention, C2 blind, C3 responsibly unresolved — 0/1 each. Per-sample accuracy stays as a
   descriptive secondary.
5. **The Ω-window construction needs a frozen rule** before the reserved configurations: "≥N
   reference cells of the staged surface inside the sensing sector" changes the window edges
   (N=1 gave t2–26, N=3 gave t13–21 on the same run). Decision for Renxi alongside the
   object-question threshold.

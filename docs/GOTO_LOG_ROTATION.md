# goto.log rotation

`goto.log` is an **append-only log shared between the Mac and GPUEDGE**, versioned in git.
Every run appends to it, and merges between the two machines are resolved by **union**
(automatic since 2026-07-31 via `.gitattributes`: `goto.log merge=union`).

## The problem

GitHub warns on every push for files above **50 MB**. On 2026-07-31 `goto.log` reached
~53–56 MB (on both `main` and the feedback branch) and each new commit of the log created
another blob over 50 MB → a warning on every push.

## Automatic rotation (since 2026-07-31)

`g1_goto.py::_open_goto_log()` is used at the three points that open the log (GOTO, TURNTEST,
GOTOVIZ). When a run starts, if `goto.log` exceeds **`G1_GOTO_LOG_MAX_MB` (default 25 MB)** it:

1. compresses the whole log to `archive/goto_hasta_<YYYYmmdd_HHMMSS>.log.gz`
   (**gitignored** — the archive stays ONLY on the machine that rotated);
2. starts a fresh `goto.log` with a `=== ROTADO … ===` header line.

If rotation fails for any reason it warns and keeps writing to the large log — it never blocks
a run.

## Mac ↔ GPUEDGE coordination protocol

Rotation truncates a file that the other machine may still hold in full. With `merge=union` no
lines are lost (the other machine's appends survive the merge), but if that machine never
pulls, its large copy is merged back in and the log grows again. After any rotation:

1. **The machine that rotated**: `git add goto.log && git commit && git push`
   (the push no longer warns: the new blob is small).
2. **The other machine, BEFORE its next run**: if it has uncommitted local appends, commit them
   first; then `git pull`. Its new lines survive the union merge, landing at the head of the
   rotated log — they are not in the other machine's `.gz` archive, but they are in git.
3. Each active branch carries its own `goto.log`: rotation applies per branch.

## Where the history lives

- **In git**: everything committed is still in the history —
  `git log --oneline -- goto.log` and `git show <commit>:goto.log`.
  (Old large blobs remain in the repository; GitHub only warns about *new* blobs on each push,
  so after rotation the warnings stop without rewriting history.)
- **In `archive/`** (local, gitignored): the `.log.gz` of each rotation. First rotation
  (2026-07-31, on the Mac):
  - `archive/goto_hasta_20260731_101916.log.gz` (56.8 MB → 5.1 MB): the feedback branch,
    produced by the automatic rotation itself on the first run after the change.
  - `archive/goto_hasta_20260731_main.log.gz` (52.2 MB → 5.2 MB): the contents of `goto.log` on
    `main` (archived by hand from the `main:goto.log` blob; it is **not** a prefix of the other
    one — the union merges reordered the tails — which is why both versions are kept).

Git LFS was deliberately **not** adopted: it needs installing and configuring on both machines
plus GitHub quota, and the git history together with the local `.gz` archives already covers
traceability.

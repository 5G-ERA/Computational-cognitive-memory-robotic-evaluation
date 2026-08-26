# The twin is not reproducible — measured, 20 Aug 2026

`sim/README.md` says the environment is now reproducible because the recipe was rescued and
`rosbridge-suite` pinned. **That is true of the recipe and false of the simulator.** Rebuilding
from the `Dockerfile` does not reproduce the twin: it produces a different one, and the difference
is measurable in behaviour, not just in package versions.

## What was measured

Same laptop, same configuration, same world, same waypoints — **only the image changes**. Three
runs B→A per arm, calibrated noise, full door configuration:

| Image | Arrived | Durations | Collisions |
|---|---|---|---|
| `g1sim:humble` (original, June build) | 3/3 | 90, 92, 89 s | 0 |
| `g1sim:humble-rb` (rebuilt from the recipe) | 2/3 | 115, 137, 102 s | **5** |
| original, restored after the detour | 3/3 | 86, 109, 107 s | 0 |

The rebuilt image is slower and collides. The restored original recovers arrivals and collisions;
its timing spread is wider (23 s vs 3 s), which could be machine load or the recreated container —
n=3 does not separate them.

## Why the recipe cannot reproduce the image

```dockerfile
FROM tiryoh/ros2-desktop-vnc:humble     # moving tag, no digest
RUN apt-get install -y ros-humble-navigation2 ...   # no version pins
```

A build in August pulls August's base image and August's ROS packages. The rebuilt image installed
`ros-humble-rosbridge-server 2.0.7-1jammy.20260729` — a July 2026 build that did not exist when the
original image was made. Gazebo, nav2 and slam_toolbox moved too.

**So the twin was never reproducible, and fixing the missing-package bug does not make it so.** The
original image is an artefact that can be preserved but not recreated.

## What follows

1. **The image is the artefact.** `g1sim:humble`, digest
   `sha256:89f16c69e2685bfd8fdd7e20636314124f9421999ad80b92e7999cbef6403f9a`, preserved at
   `~/Documents/g1_sim_artefactos/g1sim_humble_ORIGINAL.tar.gz` (2.6 GB compressed). Every twin
   result in this project — the W1 witness, the merge regression, the three door fixes — was
   measured on it. Cite the digest wherever those results are reported, exactly as
   `golden-doorcross` is cited for the object level.
2. **The original container is gone.** It carried a hand-installed `rosbridge-suite` in its
   writable layer and was destroyed during this investigation. The image survives; the container's
   exact delta does not. The rebuilt container reinstalls rosbridge and recovers arrival and
   collision behaviour, which is the check that matters, but byte-identity cannot be claimed.
3. **Duration is a fragile metric.** It varies with image *and* machine: the same configuration ran
   42–64 s on a Mac Studio versus 86–92 s on the laptop. Renxi's efficiency endpoint is a time —
   *time to the correct governed transition* — so the substrate must be fixed and declared in the
   pre-registration alongside the battery floor and the abort thresholds. **Never compare times
   across machines or images.**
4. **If a reproducible recipe is ever wanted**, it needs the base pinned by digest and every apt
   package version-pinned. Neither is available retrospectively for the June image.

## What this does not invalidate

Nothing measured so far is wrong. Every twin result stands *for the image it was measured on*, and
that image is preserved. What changes is the claim: results are conditional on an artefact, not on
a recipe, and that condition has to be stated rather than assumed away.

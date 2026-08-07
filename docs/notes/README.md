# docs/notes/ — historical engineering records

Working notes from earlier phases of the project, kept for provenance. **They are not current
documentation** — for how the system works today start at the repository [`README`](../../README.md),
and for what to do next at [`tasks/`](../../tasks/).

Most of these were written in Spanish while the work was happening and are preserved in their
original language on purpose: they are dated records of how a problem was actually solved, and
rewriting them would make them less faithful, not more useful. The current operational
documentation — runbook, protocols, twin guides — is all in English.

| File | What it records | Language |
|---|---|---|
| `HANDOFF_2026-07-02.md` | Session-by-session project state as of 2 Jul 2026. **Superseded**: the current state lives in the repository README, `tasks/`, and the migration package in the thesis repo. | ES |
| `AUTONOMOUS_NAVIGATION.md` | The perception / control / exploration algorithm of the reactive-exploration era, plus the roadmap that pointed at meta-reasoning | ES/EN |
| `PROBLEMS.md` | The supervisor's problem list from the early campaign, one fix per run, with evidence | ES |
| `G1_Air_SLAM_SOLVED.md` (+ `.pdf`) | How the robot's SLAM stream was reached at all: the WebRTC/WebView reverse-engineering write-up | ES |
| `G1_Air_SLAM_toolkit/` | The bundle that goes with the above: findings, live WebView inspection steps, APK unpacking notes and scripts | ES |
| `APK_decompile_plan.md` | Plan followed to inspect the vendor application | ES |
| `guia_g1_ros2_navegacion.md` | Early guide to the ROS 2 navigation ideas behind the current stack | ES |
| `G1_Autonomous_Navigation.pdf`, `G1_Navigation_Cheatsheet.pdf`, `G1_Robot_Test_Protocol.pdf` | Earlier printed versions of the above material | ES |
| `AGENTS.md` | Stub left by a tooling import; kept only so its absence is not mistaken for a deletion | — |

**Why the reverse-engineering notes matter.** The G1 "Air" exposes no ROS, no SDK and only one
WebRTC session, held by the vendor app. Everything this project does rests on the discovery
recorded in `G1_Air_SLAM_SOLVED.md`: that the decoded LiDAR cloud is reachable inside the app's
own WebView over USB, and that velocity commands can be published on the app's existing data
channel. That is the foundation the entire navigation and meta-reasoning stack is built on.

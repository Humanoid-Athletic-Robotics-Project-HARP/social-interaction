# Memory Index

- [Team context](team-context.md) — shared team repo: several people, mixed OSes, Claude and Codex; don't assume one user or a Mac
- [Beginner guide](beginner-guide.md) — NEWCOMERS ONLY (just cloned / "how do I get started"): sim first, then robot; GR00T start-up order. Otherwise skip

- [Project overview](project-overview.md) — G1 EDU with 29 joints; end goal (approach and talk to people); smooth curved moves with GR00T (sim-tested only); no ROS 2 yet, Nav2 later; HARP safety layer open question
- [Unitree resources](unitree-resources.md) — official/community G1 repos; cloned next to the repo, never vendored; unitree_ros2 currently unused
- [Minimal setup](feedback-minimal-setup.md) — build only what the goal needs; prefer open code we can read over Unitree's closed services; ROS 2 undecided
- [Memory style](feedback-memory-style.md) — memories are direct instructions and facts, not notes on one-off mistakes
- [G1 official docs](g1-official-docs.md) — every Unitree G1 developer doc URL; PC2 IP/login; sensors (MID-360, D435i)
- [G1 built-in APIs](g1-builtin-apis.md) — Unitree's G1 services (sport, arm, voice, motion_switcher, odometry, SLAM) with api ids and topics; mostly unavailable in debug mode
- [MuJoCo sim](mujoco-sim.md) — our sim/run_sim.py (per-step PD, real-time); ALWAYS run it with the viewer (mjpython), never --headless; Mac facts (lo0)
- [Off-robot testing](off-robot-testing.md) — how others test G1 code without the robot: own-controller vs sport split, sim options, gantry practice
- [GR00T WBC](gr00t-wbc.md) — our walking controller gr00t/walk.py: policy I/O, which NVIDIA config, licenses, real-robot consequences

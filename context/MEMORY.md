# Memory Index

- [Team context](team-context.md) — shared team repo: several people, mixed OSes, Claude and Codex; don't assume one user or a Mac
- [Beginner guide](beginner-guide.md) — NEWCOMERS ONLY (just cloned / "how do I get started"): staged setup for robot and sim (sim tested on Mac only). Otherwise skip

- [Project overview](project-overview.md) — G1 EDU humanoid with 29 joints; draft goals (social, walking, voice+LLM, lidar avoidance); research phase
- [Unitree resources](unitree-resources.md) — official/community G1 repos; unitree_ros2 used as an external dependency, not forked
- [Minimal setup](feedback-minimal-setup.md) — build only what the current goal needs; official Unitree code + ROS 2 only
- [G1 official docs](g1-official-docs.md) — every Unitree G1 developer doc URL; PC2 IP/login; sensors (MID-360, D435i)
- [G1 built-in APIs](g1-builtin-apis.md) — every G1 service (sport, arm, voice, motion_switcher, robot_state, vui) with api ids, JSON formats, client classes
- [MuJoCo sim](mujoco-sim.md) — unitree_mujoco working on one teammate's Mac (lo0, timerfd patch PR 139; other OSes untested); low-level only, no sport service; rl_gym deploy_mujoco alternative

# Memory Index

- [Team context](team-context.md) — shared team repo: several people, mixed OSes, Claude and Codex; don't assume one user or a Mac
- [Beginner guide](beginner-guide.md) — NEWCOMERS ONLY (just cloned / "how do I get started"): sim first, then robot; GR00T start-up order. Otherwise skip

- [Project overview](project-overview.md) — G1 EDU with 29 joints; goals + status ONLY in README "Goals and progress"; end goal (approach and talk to people); decisions: GR00T walking, ROS 2 Jazzy, Nav2, sim first (nothing on the robot yet); HARP safety layer open question
- [Unitree resources](unitree-resources.md) — official/community G1 repos; cloned next to the repo, never vendored; unitree_ros2 only for its messages (ros2/); plan for ROS 2 Humble/Jazzy, PC2's version depends on its JetPack
- [README and memory guidelines](readme-and-memory-guidelines.md) — READ BEFORE EDITING README OR MEMORY: README style, Goals and progress rules, memory style, one place per fact, updated (UTC)/covers front matter; run context/check_memory.py before relying on memory, act only on relevant flagged ones
- [Minimal setup](feedback-minimal-setup.md) — build only what the goal needs; prefer open code we can read over Unitree's closed services; ROS 2 adopted, add its pieces only as goals need them
- [Run logging and testing](run-logging.md) — READ BEFORE TESTING: `course WORLD --tag T --repeat N` runs and records a Nav2 course test (prints PASS/FAIL); `runs check` / `runs compare`; run_recorder for any other session; runs/ layout, summary.json, criteria, sizes; replay ONLY when a person asks, with a reason
- [Nav2 in the sim](nav2-sim.md) — our Nav2 setup (launch, params, BT) and why; footprint/inflation; bugs fixed (tilted floor → live lidar TF, tabletop erased → floor-only clearing); slalom first-gap recoveries = planner acknowledge timeout (fix next); course results; inflation 1.0/2 tried (slalom detour); still to do
- [Self-filter trial](self-filter-trial.md) — box self-filter node tried 2026-10-26 and set aside (local branch self-filter, commit 93058bd, not pushed); results, first-gap question (likely the planner timeout), how to bring it back
- [Lidar and Nav2 research](lidar-nav2-research.md) — MID-360 sim with mj_multiRay, boxes via MjSpec, real lidar (IP, topics, inverted mount, ~1 m floor blind ring), Nav2 packages on RoboStack (no STVL/docking), Graceful setup
- [G1 official docs](g1-official-docs.md) — every Unitree G1 developer doc URL; PC2 IP/login; sensors (MID-360, D435i)
- [G1 built-in APIs](g1-builtin-apis.md) — Unitree's G1 services (sport, arm, voice, motion_switcher, odometry, SLAM) with api ids and topics; mostly unavailable in debug mode
- [MuJoCo sim](mujoco-sim.md) — our sim, harp_sim's run_sim.py (per-step PD, real-time); ALWAYS run it with the viewer (mjpython), never --headless, and rviz2 in a window too; --box and --lidar (simulated MID-360); publishes true pose and events for run logging; Mac facts (lo0)
- [move_distance](move-distance.md) — flags and limits, how moves run and abort, curve (--to) rules, leg odometry method, what it logs into a run, measured sim results
- [ROS 2 workspace](ros2-mac.md) — ros2/ is the one environment (pixi, RoboStack Jazzy, Python 3.12) and colcon workspace with harp_control/harp_sim/harp_ros; executables and launch files; walker_node topics and params; ROS 2 on DDS domain 2; use `pixi run --as-is` (plain pixi run reinstalls unitree_sdk2py and races); Mac-only gotchas
- [Off-robot testing](off-robot-testing.md) — how others test G1 code without the robot: own-controller vs sport split, sim options, gantry practice
- [GR00T WBC](gr00t-wbc.md) — our walking controller, harp_control's gr00t/walk.py: policy I/O, which NVIDIA config, licenses, real-robot consequences
- [RL ideas](rl-ideas.md) — brainstorm, not a goal yet: /cmd_vel-level RL (social nav, approaching people; cheap, start here) vs joint-level RL (gait style, gestures; needs NVIDIA GPU); tools, deployment path, bigger-scope ideas, order

# Memory Index

- [Team context](team-context.md) — shared team repo: several people, mixed OSes, Claude and Codex; don't assume one user or a Mac
- [Beginner guide](beginner-guide.md) — NEWCOMERS ONLY (just cloned / "how do I get started"): sim first, then robot; GR00T start-up order. Otherwise skip

- [Project overview](project-overview.md) — G1 EDU with 29 joints; goals + status live in README "Goals and progress"; end goal (approach and talk to people); smooth curved moves with GR00T (sim-tested only); next goal Nav2 obstacle avoidance; ROS 2 adopted, code in ROS 2 packages, walker node (/cmd_vel, /odom, /tf) sim-tested; HARP safety layer open question
- [Unitree resources](unitree-resources.md) — official/community G1 repos; cloned next to the repo, never vendored; unitree_ros2 only for its messages (ros2/); plan for ROS 2 Humble/Jazzy, PC2's version depends on its JetPack
- [README style](feedback-readme-style.md) — README is a short front page: what the robot does and has (no "not X"), commands + bullets; detail goes in context/
- [Goals workflow](goals-workflow.md) — how to edit README "Goals and progress": wide unnumbered goals, bullets only, sim/robot status, update in the same commit; goal changes need the team
- [Minimal setup](feedback-minimal-setup.md) — build only what the goal needs; prefer open code we can read over Unitree's closed services; ROS 2 adopted, add its pieces only as goals need them
- [Memory style](feedback-memory-style.md) — memories are direct instructions and facts, not notes on one-off mistakes
- [Nav2 in the sim](nav2-sim.md) — our Nav2 setup (launch, params, BT) and why; bugs fixed (tilted floor → live lidar TF, tabletop erased → floor-only clearing); course results; inflation 1.0/2 tried (slalom detour); scripted course tests (--as-is, --auto-release 15, start the policy only after a fresh "Feet down")
- [Self-filter trial](self-filter-trial.md) — box self-filter node tried 2026-10-26 and set aside (branch self-filter, commit 93058bd); results, open slalom first-gap question, how to bring it back
- [Lidar and Nav2 research](lidar-nav2-research.md) — MID-360 sim with mj_multiRay, boxes via MjSpec, real lidar (IP, topics, inverted mount, ~1 m floor blind ring), Nav2 packages on RoboStack (no STVL/docking), Graceful setup
- [G1 official docs](g1-official-docs.md) — every Unitree G1 developer doc URL; PC2 IP/login; sensors (MID-360, D435i)
- [G1 built-in APIs](g1-builtin-apis.md) — Unitree's G1 services (sport, arm, voice, motion_switcher, odometry, SLAM) with api ids and topics; mostly unavailable in debug mode
- [MuJoCo sim](mujoco-sim.md) — our sim, harp_sim's run_sim.py (per-step PD, real-time); ALWAYS run it with the viewer (mjpython), never --headless, and rviz2 in a window too; --box and --lidar (simulated MID-360); Mac facts (lo0)
- [move_distance](move-distance.md) — flags and limits, how moves run and abort, curve (--to) rules, leg odometry method, measured sim results (moved out of the README)
- [ROS 2 workspace](ros2-mac.md) — ros2/ is the one environment (pixi, RoboStack Jazzy, Python 3.12) and colcon workspace with harp_control/harp_sim/harp_ros; walker_node topics; ROS 2 on DDS domain 2; use `pixi run --as-is` (plain pixi run reinstalls unitree_sdk2py and races); Mac-only and testing gotchas
- [Off-robot testing](off-robot-testing.md) — how others test G1 code without the robot: own-controller vs sport split, sim options, gantry practice
- [GR00T WBC](gr00t-wbc.md) — our walking controller, harp_control's gr00t/walk.py: policy I/O, which NVIDIA config, licenses, real-robot consequences
- [RL ideas](rl-ideas.md) — brainstorm, not a goal yet: /cmd_vel-level RL (social nav, approaching people; cheap, start here) vs joint-level RL (gait style, gestures; needs NVIDIA GPU); tools, deployment path, bigger-scope ideas, order

---
name: safety
description: Read before planning a real-robot session, changing what stops the robot, or adding a safety check
metadata:
  type: reference
  updated: 2026-10-30T23:01:59Z
  covers: [submission/entrypoint.sh, submission/harp_node.py, submission/harp/moves.py]
---

**HARP's harness stops the robot, never our code** ([[him-sim2real]]). Our image only proposes joint commands through it; it never writes `rt/lowcmd`.
- **Buttons** (the operator's remote, HIM's, replacing our old map; the teammate who set that rule was told, 2026-10-28): double Start damps at any time; double F3 switches to HIM's Holosoma recovery; D-pad Left + B hands control to our image once start-up balance is stable. Ctrl-C or `./submit.sh --mode real stop` ends the session.
- **Automatic:** tilt past 42° or pelvis below 0.49 m (recovery if standing, else damping), torque, joint speed, base spin 6 rad/s, thrashing (enforced on the robot), 0.2 s command and observation deadlines, stale telemetry, battery and motor temperature, and a stability gate before handover. A stop latches until double Start damps and the robot is back in ready mode.
- HIM's own docs: physical stopping is not yet accepted (a jerky T-pose stop shut down a joint).

**Our part:** answer every `/safety/policy_state` within 0.2 s (keep heavy work out of the policy process and at lower priority: [[submission-image]]); `/cmd_vel` drives the walker and goes to zero after 0.5 s of silence; moves and Nav2 stop by sending zero, and GR00T balances.

**Gap:** a lost lidar doesn't stop Nav2 on Foxy: nothing in it acts on a stale costmap. `harp/approach_stop.py` stands the robot after 0.5 s without a scan ([[nav2-sim]]), in the sim's course runs and wherever the behavior loop runs, the robot included ([[behavior-loop]]); nothing else on the robot does.

**To check on the robot:** what `wireless_remote` reads with the remote off or out of range (if all zeros, the remote can't stop it then; only the laptop and HIM's deadlines can), and whether anything else reacts to HIM's button gestures.

**Sessions:** an operator owns power, the gantry, the remote and stopping; someone else runs the laptop and the recording. Gantry with feet bearing weight, Unitree's stock controller in locked standing mode (HIM's profile accepts only FSM 4), Start and F3 released. HIM's stages: preflight (`./setup.sh`), dry run (motors off), real (`./submit.sh --mode real`), README "On the robot". First a stationary recording with HIM's recorder, then HIM's own walker example standing, then ours standing (`G1_TASK_COMMAND='[0,0,0]'`; without a task it walks its default move), then small reversible moves. Never two motor controllers at once. Stopping a recording doesn't stop the robot.

**What others do:** Unitree `g1_ctrl` (unitree_rl_mjlab): an FSM starting in Passive, LT+B to Passive from any state, Passive on lost state, refuses to start while another program sends `rt/lowcmd`; its fall check is written but always returns false. NVIDIA GR00T WBC: `JointSafetyMonitor` (arm speed over 6 rad/s shuts down) and a deploy checklist (sim first, check state with no commands, first test at 2–5× lower kp, an e-stop ready). humanoid_bridge: latches damping on lost state, NaN, overspeed or a second `rt/lowcmd` sender. Go2 projects report motor power cut about 20 ms after `rt/lowcmd` stops; unconfirmed for the G1.

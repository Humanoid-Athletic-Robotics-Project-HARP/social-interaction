---
name: controls
description: Read before changing a key or remote button, how driving by hand takes over from a program, or planning how the operator controls the robot in a session
metadata:
  type: project
  updated: 2026-10-28T03:10:45Z
  covers: [ros2/src/harp_control/harp_control/remote.py, ros2/src/harp_control/harp_control/gr00t/walk.py, ros2/src/harp_control/harp_control/move_distance.py, ros2/src/harp_ros/harp_ros/walker_node.py]
---

The table of keys and buttons is in `remote.py`'s docstring and the README's "Controls". Every program built on `Walker` gets the same controls; remote presses become the keyboard key they stand for inside `Walker.read_remote`, so there is one path for each action.

**The rule (a teammate's call, 2026-10-28; ask before breaking it):** urgent actions take one button (stop: B / Space; take over: hold L1 / a drive key; stop moving: let go of L1 / `z`), deliberate ones take two (R1 + a button, Shift + a key): start the policy, hand control back, height.

**Why it's built this way**
- B alone stops, because the fastest press matters most and a stop only ever moves toward the safe state. L2+B (Unitree `g1_ctrl`'s LT+B, which HARP operators know) still works since it contains B. It's checked every 5 ms, whatever else is held.
- R1 combos fire only on the other button's press, with nothing but R1 held, so Unitree's chords (L1+A damping, L2+R2 debug mode) never trigger one.
- L1 is a dead-man: the sticks drive only while it's held, and letting go zeroes the command, so a dropped remote stops the robot walking. A takeover needs a fresh L1 press after the policy is on; drive keys before the policy are refused, so nothing walks off the gantry at policy start.
- Taking over (`Walker.take_over`) sets `manual`; at each policy step the hand command replaces whatever the program set. `walker_node` then ignores `/cmd_vel`, and `move_distance` cancels the moves in progress (before its first Shift+G, driving by hand only positions the robot). Nothing resumes until Shift+G or R1+A, which also zeroes the command.
- Full stick is `walk.py`'s speed limits (0.6 m/s, 0.4 m/s, 1 rad/s), with no ramp, as for the keys and `/cmd_vel`.

**Tested** in the sim on 2026-10-28 through scripted remote events against `walker_node`: R1+start starts, start alone doesn't, L1 overrides `/cmd_vel` and the robot stays put after letting go, L1 + left stick at 0.5 walks about 0.24 m/s, R1+A hands back, B stops. Keyboard path through `move_distance` in a pty. Not run on the robot.

**To check on the robot:** what `wireless_remote` reads with the remote off or out of range (if it reads all zeros, B can't stop it then; only the laptop and the state timeout can), whether anything in debug mode reacts to our buttons, and the stick signs (we follow unitree_mujoco's mapping: forward is +ly, right is +lx and +rx).

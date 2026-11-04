---
name: wave
description: Read before changing or running the wave (harp/wave.py, the {"wave": {}} task), or adding another arm gesture
metadata:
  type: project
  updated: 2026-11-04T20:41:43Z
  covers: [submission/harp/wave.py, submission/tests/test_wave.py]
---

**What it is (Lam's design, 2026-11-04):** a scripted right-arm hello wave under HIM's GR00T walker: our code sets only the right arm's 7 joints (indices 22–28) while GR00T keeps the legs and waist. Standalone for now; the behavior loop ([[behavior-loop]]) is to trigger it once person detection exists. Picked over a learned BeyondMimic wave (HIM's `dance/` method: GPU training, policy switching mid-run) and Unitree's built-in arm action 26 (needs Unitree's motion control, which HIM releases, [[g1-builtin-apis]]). **When balance and looks conflict, balance wins** (Lam): give up swing speed and size first, lower the hand last, never counter-move the left arm or waist.

**Motion** (`harp/wave.py`, a pure function of time): a 1 s minimum-jerk raise from the measured arm pose to `HAND_UP`, 3 elbow swings at 2 Hz (0.4 rad each way) with wrist roll a quarter of that and 0.15 cycle behind, faded in and out over half a swing, then a 1 s minimum-jerk lower to the walker's zeros. Wrist pitch and yaw never move: their 5 Nm caps leave about 0.11 rad of tracking room at HIM's kp 40. `HAND_UP` (0, −1.6, −1.4, 0, 0, 0, 0) is from forward kinematics on HIM's G1 model: wrist 0.21 m above, 0.23 m out and 0.03 m ahead of the shoulder, forearm upright, an elbow swing moving it sideways; `tests/test_wave.py` checks this on `/harp/model/g1_robot.xml`. Which way the palm faces is unconfirmed (the model has no hand).

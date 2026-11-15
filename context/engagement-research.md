---
name: engagement-research
description: Read before changing whom the behavior loop chooses to approach, when it waits for people to come instead, or when it gives up on someone
metadata:
  type: reference
  updated: 2026-11-15T22:27:13Z
  covers: []
---

Web research 2026-11-15, not tried by us; overview in [[social-behavior-research]].

Findings
- Bohus and Horvitz (SIGDIAL 2009, full text, 3274 kiosk traces, http://erichorvitz.com/Bohus_Horvitz_Predict_Engagement.pdf): adding trajectory to position cut false positives about 3 times, 3 s or more before engagement; face direction added little.
- Intention to interact from skeletons (https://arxiv.org/html/2309.07477, 3422 sequences): AUROC 0.94; distance and its change carried most of it (distance alone about 0.92), torso orientation alone 0.56 to 0.67. Labels were automatic (5 s within 1 m counts as an interaction, the 10 s before are positive), and a robot learned from its own encounters, 0.5 to 0.94 AUROC in 3 days.
- Satake et al. 2013 (https://www.dylanglas.com/research/pdf/satake-tro2013-preprint.pdf): picks people who don't look busy (idle-walking, wandering, stopping, not busy-walking) and reachable from the front.
- Inviting can beat approaching: in a mall with over 65,000 passers-by a humanoid that drew attention stopped 7.7 to 10.6%, greeting only 5.8%; hurried people rarely stopped, groups and children more (https://arxiv.org/html/2109.02771v1). A polite mix of waiting and approaching beat either alone (Kato et al. 2015, snippet).
- Datasets: ATC approach set, 130 ten-second trajectories labelled intent or not, research use only (https://dil.atr.jp/sets/approach_robot/); PAR-D (Yale, ICMI 2024, licence unknown).
- Leg-based orientation from 2D lidar: 6 to 14° mean error but assumes the person starts facing the sensor (Gao et al. 2023, MIT code reported, unverified).
- No study found on phones, headphones or carried items.

What our sensors give: speed, heading, slowing, closing on the robot, dwell and groups from lidar tracks (good); torso orientation only within 3 to 4 m and not in the wedges (weak); speech addressed to the robot from the mic (useful, no direction). Never: faces, phones, headphones.

Suggested rule (starting values): score each confirmed person. Closing on the robot over 0.3 m/s for 1 s +3; slowed by half in 2 s within 6 m +2; standing still +1, over 3 s +1 more; turned over 30° toward the robot +1; spoke near it +3; walking away over 1.0 m/s or hurrying -3; in a group or conversation -2; confidently facing away -1. Approach at 2 or more (relax to 1 if too few qualify). Someone closing or pausing within 5 m: stop, face them and wait 6 to 8 s. Give up when they leave over 0.5 m/s, after about 10 s without closing, or when someone walks between. Idle: park 10 to 20 s with a slow turn and a visible cue before wandering on. Log features and outcomes to fit a small classifier later.

Open: acceptance by cue; waiting against approaching on campus; lidar torso orientation on the MID-360; false triggers of a spoken-address detector in corridors.

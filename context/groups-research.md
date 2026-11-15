---
name: groups-research
description: Read before adding groups to the behavior loop: detecting them from lidar tracks, choosing among several people, where to stand, talking with more than one person
metadata:
  type: reference
  updated: 2026-11-15T22:27:13Z
  covers: []
---

Web research 2026-11-15, not tried by us; overview in [[social-behavior-research]]. The loop now handles one person at a time ([[behavior-loop]]).

Findings
- Kendon's F-formation: members face a shared empty centre (o-space), stand in a ring (p-space), outsiders stay outside (r-space). Survey: https://arxiv.org/pdf/2108.06181.
- Detection from position plus head orientation: GCFF F1 0.89 over five datasets, Hough voting 0.74 to 0.77, dominant sets 0.82 (Setti et al. 2015, https://arxiv.org/html/1409.2702; MATLAB code, MIT, https://github.com/franzsetti/GCFF). With noisy or no orientation, dominant sets did as well or better. No study reports position-only accuracy on lidar tracks.
- Vázquez et al.: lower-body orientation estimated together with groups (IROS 2015); in group talk, the robot's body orientation and gaze behaviours both gave a similar sense of inclusion (HRI 2017, abstract), which matters since our only "gaze" is the body.
- Stopping by a group: 45 cm drew attention, 120 cm was barely noticed in face-to-face pairs, both disrupted L-shapes (HRI 2026 companion, snippet only, https://doi.org/10.1145/3776734.3794523). Pairs liked directions where both could see the robot (Ball et al. 2017).
- Bohus and Horvitz multiparty kiosk (2009): tracks engagement per person, opens with "Excuse me" when attention is elsewhere, can invite a bystander (53% accepted over 20 days). The hard parts were identity swaps and knowing whom speech was addressed to.
- Datasets with group labels: JRDB-Act and JRDB-Social (robot lidar and video, now hosted at Monash, licence unchecked), SALSA, Cocktail Party, Coffee Break.

What position-only tracks give: who stands close together and still, group size, centre, who is alone. Not without orientation: a conversation versus strangers waiting side by side, where the gap in the ring is, whether the group is closed.

Suggested rules (starting values): link two still tracks (under 0.3 m/s) within 1.5 m (2.0 m for three or more) held 2 to 3 s; groups are connected sets. Prefer a lone person or a still group of 2 to 3; skip moving groups, groups of 4 or more, and tight face-to-face pairs; never cut between members. For a group, stop 1.5 to 2.0 m from the nearest member on their open side, facing the centre; open with "Excuse me" from 2.5 to 3 m. Greet everyone, answer whoever spoke last or is nearest, turn toward the speaker; end when all have left or 15 s of silence. If a crowd gathers: stop wandering, speak to all, take one person at a time.

Open: grouping accuracy on our own logs or JRDB-Act; campus spacing thresholds; group stop distance with no gaze; whether the mic array can tell speakers apart at 1.5 to 2.5 m.

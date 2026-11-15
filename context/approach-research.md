---
name: approach-research
description: Read before changing how the behavior loop walks up to a person: stop distance, approach angle, speed profile, when to abort
metadata:
  type: reference
  updated: 2026-11-15T22:27:13Z
  covers: []
---

Web research 2026-11-15, not tried by us; overview in [[social-behavior-research]]. Lab studies with small or wheeled robots, one mall field trial.

Findings
- Hall's zones as used in HRI: intimate under 0.45 m, personal 0.45 to 1.2 m, social 1.2 to 3.6 m, public beyond.
- People let robots come closer than strangers: "too close" averaged 0.44 to 0.68 m for a PeopleBot (Walters et al. 2007, 42 people, small cells, https://cdn.aaai.org/Symposia/Spring/2007/SS-07-07/SS07-07-022.pdf).
- Zones grow with robot height: PR2 at 1.33 m (our height) about 0.5 to 0.6 m, at 1.65 m about 0.6 to 0.75 m; women keep more room, more so for the tall robot (UNR, RO-MAN 2019, 40 students, https://rrl.cse.unr.edu/media/documents/2019/Factors_Influencing_the_human_preferred_interaction_distance_Roman_19_final.pdf). Prior robot experience and pet ownership shrink it (Takayama and Pantofaru 2009).
- Direction: front-left or front-right was most comfortable, from behind least, dead centre worse for someone against a wall (Walters 2007). A robot that circled to a person's side and then spoke was ignored (Satake 2013).
- Satake et al. 2009 (https://www.dylanglas.com/research/pdf/satake-hri2009.pdf, mall, 1.2 m Robovie): predicting the path and appearing in front at public distance got 33 of 59 to stop and listen (56%) against 20 of 57 (35%). Of the rest: 27% refused, 18% unsure it meant them, 4% unaware, 3% unreachable. Readings of its speed differ (0.3 m/s or about 0.8 m/s).
- Conversation initiation preferred 1.6 to 1.8 m and a front approach (https://doi.org/10.3390/info11030147, abstract only).
- Path shape: straight, steady motion that slows near the person read as most legible; sharp curves raised workload (VR and warehouse studies).

For our loop (starting values to tune)
- Stop at 1.1 to 1.2 m for one standing person (as now), never under 0.9 m. Seated people: about 1.5 m, or skip them at first.
- End 10 to 20° off their line of sight, from their front half; never pass behind. Our blind wedges and slow sidestep make this harder, so most arrivals will be frontal.
- Turn the body onto them from 3 to 4 m as the "coming to you" signal; straighten the path for the last 2 m; slow from 0.23 m/s to about 0.1 m/s between 1.5 and 1.1 m with no sudden changes.
- Abort: they walk off (range growing over 0.3 m/s for 1 s): stop, wait 3 to 5 s, give up, don't chase. They step back or turn away within 2 m: stop, wait 3 to 5 s, then retreat to about 2 m and move on. They come toward the robot: stop and let them close. Someone walks between: abort. After visible discomfort, wait about 300 s before trying them again.

Open: A/B stop distances 0.9, 1.1, 1.4 m and count step-backs; off-axis against dead centre; whether a wave at 3 m cuts the unsure share; whether a body turn reads as intent.

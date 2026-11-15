---
name: nonverbal-research
description: Read before adding gestures, facing rules or LED signals to the behavior loop or speech, or picking gestures from the LLM's replies
metadata:
  type: reference
  updated: 2026-11-15T22:27:13Z
  covers: []
---

Web research 2026-11-15, thin (search was rate-limited), not tried by us; overview in [[social-behavior-research]]. The wave we have: [[wave]]; Unitree's built-in arm actions and LED: [[g1-builtin-apis]] (its action ids are the service's; the SDK example's numbers 0 to 15 are a menu index).

Findings
- Gestures while talking made ASIMO more humanlike and likeable, with more wish for future contact, most with partly mismatched gestures; random gestures didn't replicate it (Salem et al. 2013, https://link.springer.com/article/10.1007/s12369-013-0196-9, lab).
- Pointing gestures helped people remember what a robot said (Huang and Mutlu 2013). A NAO's iconic gestures were recognised 80% of the time against 94% for a person; beat gestures may add little (Bremner and Leonards 2016).
- An LLM composing expressive behaviours from a robot's existing skills was rated better than an animator's for some (excuse me, approach) and worse for others (GenEM, HRI 2024, https://arxiv.org/abs/2401.14673; video studies, no arm gestures).
- Legible motion commits early and visibly to its goal (Dragan et al. 2013).
- Small G1 projects with gestures and voice, examples only, not vetted: infodriver/g1-pilot (MIT), yutoAb/g1-work-coach (MIT), yizhiyu196/g1-mediapipe-gesture-recognition (Apache-2.0).
- Not found: idle motion, LED signals, handshake safety, culture, the uncanny valley for faceless humanoids.

What the body can say: facing (turning the whole body is its only gaze), arm gestures, path and speed, the head LED, stillness. Not: gaze, nods, any facial expression. Turning 180° reads as a strong "done".

Suggestions
- First gestures: a shorter wave (about 3 s) at 2 to 3 m before speaking; a rest pose; a low one-arm open palm for "I don't know" or an offer; a small goodbye wave before turning away.
- The LLM adds one tag from a fixed list (none, offer, thanks, goodbye) to each reply, default none, at most one a turn, started with the speech; greeting and goodbye gestures come from rules, not the LLM.
- Face the person throughout; turn again when off by more than 20 to 30°; in a group, turn toward whoever speaks.
- LED colours for idle, approaching, listening, speaking, leaving (untested).
- Arm safety at about 1 m: hand below face height, wrist at least 0.5 m from the person, nothing that sweeps toward them (no hug, high five or handshake until tried with a trained volunteer), skip if they're under 0.8 m away, always release posed actions.

Open: whether Unitree's arm service works while HIM's walker runs (probably not, [[g1-builtin-apis]]); a wave at 1 m against 2 m; gesture against no gesture on walk-aways; whether a slow turn with a wave feels less rude than a plain 180°.

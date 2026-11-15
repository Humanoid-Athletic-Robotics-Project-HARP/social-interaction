---
name: social-behavior-research
description: Read first when designing how the robot behaves with people (whom to approach, how, talking, gestures, groups, public sessions); it points to the detailed research memories
metadata:
  type: reference
  updated: 2026-11-15T22:27:13Z
  covers: []
---

Web research 2026-11-15, nothing tried by us. Eight research passes with rate-limited search: many numbers come from abstracts or summaries, and each memory says which. Almost all evidence is from small lab studies or Japanese malls with wheeled robots about 1.2 m tall; nothing measured a walking 1.3 m humanoid approaching strangers. Treat every value as a starting point to test. The loop it applies to: [[behavior-loop]]; the speech side: [[talking]].

Detail by topic:
- Approaching: distance, angle, speed, abort rules ([[approach-research]]).
- Whom to approach and when to wait instead ([[engagement-research]]).
- Groups, and choosing among several people ([[groups-research]]).
- Opening, sustaining and closing the talk ([[conversation-flow-research]]).
- Gestures, facing, the LED ([[nonverbal-research]]).
- What happened when robots met the public ([[field-deployments-research]]).
- Law, IRB, safety norms and a checklist for public sessions ([[public-sessions-ethics]]).
- Testing it in the sim and scoring real sessions ([[social-testing-research]]).

What most of the research agrees on, and how it differs from our loop now:
- Signal intent before arriving. In the best field study (Satake et al. 2009) a plain approach and greeting got 35% of people to stop and listen; predicting their path, coming from the front and signalling at public distance got 56%. The main failures were people who didn't notice the robot or weren't sure it meant them. Our loop greets only on arrival at 1.1 m. Add a visible turn toward the person and a wave or short call at about 3 m.
- Trajectory beats a snapshot. Slowing down, curving toward the robot and closing on it predict who wants to interact better than position or orientation. Our only rule is "standing still". Score people on these cues, and wait for someone coming toward the robot instead of walking at them.
- Expect few takers. About 6 to 17% of passers-by engaged in mall deployments; many only watch. Log every attempt, not only talks.
- Leave politely. A pre-closing ("Anyway, I should get going"), a wish and thanks beat an abrupt end; finish the goodbye before turning 180°. Make the 30 s cap soft.
- Speech recognition in public noise was the weak point of every deployment (92.5% word accuracy in the lab fell to 21.3% in a mall).
- Before any public session: an IRB determination, EH&S and building sign-off, signage, no saved raw audio by default (California's all-party recording consent law), two operators.

Where the research differs: whether to ask permission first ("may I talk with you?") or greet straight away; stop distance for one person (1.1 to 1.2 m) versus groups (1.5 to 2.0 m).

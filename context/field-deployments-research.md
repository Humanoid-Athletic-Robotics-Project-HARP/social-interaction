---
name: field-deployments-research
description: Read before planning public sessions or behaviour for crowds, children, obstruction and people who decline: what happened when social robots met the public
metadata:
  type: reference
  updated: 2026-11-15T22:27:13Z
  covers: []
---

Web research 2026-11-15, not tried by us; overview in [[social-behavior-research]]. Rules for running sessions: [[public-sessions-ethics]].

Findings
- Shopping mall, four 1.2 m Robovies (Shiomi et al. 2009, read in full, https://www.dylanglas.com/research/pdf/shiomi-iros2009.pdf): 414 of 2411 passers-by interacted (17%); many only watched. Robots approached only people stopping or walking idly. No collisions with over 20 people around. Speech recognition fell from 92.5% word accuracy in 75 dBA lab noise to 21.3% in the mall, so an operator did it. One operator and one safety person stayed close.
- Children's abuse (Brscić et al. 2015, https://dl.acm.org/doi/10.1145/2696454.2696468, abstract and press): children obstructed, insulted and sometimes hit the robot, mostly in groups with no adult near. Risk was estimated from interaction time, crowd density and the number of people under about 1.4 m; the robot then moved toward adults or crowds. It asked people to move aside once, then turned and went another way.
- Novelty (Valerie roboceptionist, CMU, Gockley et al. 2005): interest fell over the first weeks then levelled; few talks passed 30 s; repeat visitors stayed longer.
- Pepper in shops and banks: unreliable, didn't recognise returning people, mostly dropped (press). MuMMER (mall in Finland, 14 weeks): broadly liked, replies lagged from offloaded computing.
- Delivery robots on a campus: 40 dangerous and 60 moderate near-misses in 5 days of video (https://www.metrans.org/assets/research/psr_finalreport_21-16.pdf); sudden braking was hard for people to anticipate.
- Surveys (industry-linked, 2026): a third positive about humanoids, 40% negative, far more positive after meeting one; 57.5% more comfortable with a trained person watching; most discomfort near children.
- Not found: data on public humanoid demos (G1, Figure, Ameca), photo-taking crowds, JackRabbot findings.

What matters most for us: a walking approach is ambiguous, so signal intent early; expect low uptake and many onlookers; speech recognition in noise will fail often; a falling 35 kg humanoid is a bigger hazard than a wheeled robot; abrupt stops cause near-misses.

Suggestions
- Don't approach groups of children, crowds over 3 to 4, or fast walkers.
- Blocked: ask once to pass, then turn and go another way. Crowded closer than the stop distance: stop, say it will wait, back off; never walk into anyone.
- When recognition keeps failing, use short fixed lines instead of repeated "pardon?".
- Sessions: weekday daytime in staff and student areas first; no tours or children until tested; check noise and Wi-Fi first; run the LLM locally; plan battery swaps.
- Log every attempt, including passers-by counts for rates: time, place, people near, start and stop distance, acknowledged or not, outcome, length, recognition failures, operator actions, near-contacts, children or groups present.

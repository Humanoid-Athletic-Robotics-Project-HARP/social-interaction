---
name: conversation-flow-research
description: Read before changing how a talk opens, handles silence or refusal, how long it lasts and how it ends, or the LLM prompt's social rules
metadata:
  type: reference
  updated: 2026-11-15T22:27:13Z
  covers: []
---

Web research 2026-11-15, not tried by us; overview in [[social-behavior-research]]. Our design: [[talking]]; speech options: [[talking-research]].

Findings
- Greeting in phases (Kendon; Heenan et al. 2014 on a Nao, weak evidence): a distance salutation, then the close greeting within about 1.6 m. Satake 2009 started talk at social distance after signalling from public distance ([[approach-research]]). Shi, Kanda et al. (RSS 2011) model when and where to say the first greeting.
- Latency: human gaps are about 0.2 s; a Furhat field study measured 1.35 s mean (SD 0.55) and latency was the second biggest barrier (https://arxiv.org/html/2503.15496v2). Better turn-taking cut median response from 2.7 to 1.5 s and interruptions from 16.6% to 6.9% (HRI 2025, https://arxiv.org/html/2501.08946v1, secondhand numbers). Tolerance falls past about 2 s (older work, unverified). No numbers found on fillers.
- Closings: summary or goal, a positive comment, a wish, thanks, rated more likeable and polite than a plain end (Sci. Reports 2024, https://www.nature.com/articles/s41598-024-81353-7). Conversation analysis: a pre-closing ("well", "anyway") comes before the goodbye; without it an end sounds abrupt.
- LLM robots: Pepper with ChatGPT claimed it could set medication reminders (https://arxiv.org/html/2410.00434v2); Furhat with GPT-3.5 and older adults: interruptions, slow or repetitive replies, made-up facts (https://link.springer.com/article/10.1007/s10514-025-10190-y).
- No study found on saying "I'm a robot" or "I may be recording" up front.

Fit: our 1.1 to 1.6 s to first sound matches the Furhat field mean. Risks: a 1.2 s filler can collide with a reply at 1.3 s; a hard 30 s end with no pre-closing; a 4B model claiming abilities.

Suggestions (starting values)
- A short call from about 3 m ("Hi there!") while facing them, then at stop: who it is, one easy question and an out ("I'm HARP, Stanford's robot. How's your day going? No worries if you're busy.").
- Latency: aim for 1.0 to 1.5 s, 2 s at most. Filler only if nothing by about 1.0 s, kept short, dropped if the reply is ready within 0.15 s of it. Stream by sentence.
- After the greeting wait 4 to 5 s; one lighter retry; about 3 s more; then "Have a good one!" and leave. "No thanks", "busy" or walking away: a goodbye of at most five words, no persuasion. Log the outcome: ignored, declined, walked away, finished.
- Mishearing: one "Sorry, could you say that again?", then move on. If they talk over it, stop and listen.
- Length: pre-close at 20 to 25 s ("Anyway, I should get going. Nice chatting, enjoy your day!"), extend while they give long answers, hard end about 40 s. Turn away only after the goodbye has played.
- Prompt: never claim abilities it lacks (reminders, remembering people, looking things up, going places); at most one question a turn; no names or personal details; light deflection of rude input; truthful about recording; simpler and shorter with children; no invented campus facts. Safety topics: [[public-sessions-ethics]].

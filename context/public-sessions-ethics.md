---
name: public-sessions-ethics
description: Read before the robot talks with or records anyone outside the team, or before planning a session in a public space at Stanford: recording law, IRB, safety norms, a checklist
metadata:
  type: reference
  updated: 2026-11-15T22:27:14Z
  covers: []
---

Web research 2026-11-15, not legal advice; the questions at the end are for Stanford's offices. Overview in [[social-behavior-research]]; operating the robot safely: [[safety]].

Findings
- California Penal Code 632: recording a confidential conversation without everyone's consent is a crime (up to $2,500 and a year); a civil claim is $5,000 per violation (637.2). Confidential means people reasonably expect nobody is listening or recording; public gatherings are excluded. A one-on-one talk with the robot is more likely confidential than hall noise; bystanders' speech on the mic is the biggest risk. https://law.justia.com/codes/california/code-pen/part-1/title-15/chapter-1-5/section-632/
- CCPA covers businesses, likely not a university lab; FERPA likely not, unless student-identifiable records are kept (both unverified).
- Stanford IRB: research means a systematic study for generalisable knowledge. Engineering tests likely aren't; analysing how people react in order to publish likely is. Get a written "not human subjects research" determination or approval through eProtocol or irbeducation@stanford.edu. https://irb.stanford.edu/for-researchers/do-i-need-irb-review
- Stanford Outdoor Robot Policy: outdoors needs written Provost approval and insurance ($2M general liability, $1M auto, workers' comp); it doesn't cover inside buildings. https://transportation.stanford.edu/outdoor-robot-policy . No indoor rule found: ask EH&S, Risk Management and building managers.
- Standards: none sets humanoid speed or distance near the public. ISO 13482 (personal care robots, revision in progress) is the closest risk-assessment template. ISO/TS 15066 contact limits (now in ISO 10218-2:2025) are for fixed arms, not a falling 35 kg biped. ISO 25785-1 (dynamically stable legged robots) is still a draft. The IEEE humanoid study group's 2025 report names falls, especially onto children, as the main concern.
- HRI ethics (Riek and Howard): be open about being a machine; people forget they're being recorded by something humanlike.

Checklist before the first public session
- IRB determination, EH&S or Risk Management sign-off, the building's permission, insurance confirmed.
- Team members first, then invited volunteers, then strangers.
- Signs where people enter: an autonomous research robot that listens and talks, what is kept, a contact. The robot says it too.
- No raw audio saved by default; transcripts only if needed, without names; debugging recordings only with consenting people, encrypted, deleted on a schedule (for example 30 days); no face or voice identification.
- Two operators: one on the stop remote within reach, one watching people and answering questions. A written risk assessment and a fall test; an incident and near-miss log. Stop on any complaint.
- Keep away from stairs, narrow corridors, sleeping people; children only with an adult present, never as a target.
- Prompt: always a robot and an AI, never claims feelings or memory of people, truthful about recording; turns down medical, legal, sexual, violent, political and self-harm topics with a pointer to help for self-harm; no pressure to keep talking; ends on request. Test with adversarial and child-like inputs, and filter input and output.

Ask Stanford: IRB (testing or research, saved transcripts, minors); EH&S and Risk Management (indoor robots, expected risk assessment, falls); counsel and the Privacy Office (whether 632 covers the robot's mic in halls and lobbies, whether signs and a spoken notice are enough, retention, California's chatbot disclosure laws).

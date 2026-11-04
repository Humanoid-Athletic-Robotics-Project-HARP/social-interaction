---
name: speech-tuning
description: Read before changing listen.py's turn settings, talk.py's prompt, grammar, goodbye rules or fillers, or choosing the voice — what synthetic tests found and the changes they point to
metadata:
  type: project
  updated: 2026-11-04T21:48:25Z
  covers: [speech/listen.py, speech/talk.py]
---

Synthetic tests on an M1 MacBook Air's CPU (2026-11-03/04), with no mic or speakers: 222 clips from macOS `say` voices with noise, room echo, a rough robot-mic filter, loudness levels and pauses (each clip scored on turns, word errors, false starts and turn-end lag), and 17 scripted conversations through `talk.py --no-play`. The scripts aren't in the repo. Designs: [[listening]], [[talking]].

**Listening**
- Made in `listen.py` from these tests: `MADE_UP` keeps "Bye." and "Thank you." (Parakeet made up no phrase on noise alone: pink, fan, music, footsteps, clicks, bangs); `FLOOR_MARGIN` 1.0; a start count that drops by one on a frame without voice instead of resetting; `PREROLL` 0.8 s. Before them, in noise a quiet syllable delayed or lost the start ("Not much, I'm…" lost its first words). Results on the clips: in noise 10% word errors (47% before), every noisy and quiet clip one turn (23 of 80 noisy clips gave none before), all short words kept, false starts unchanged. Raising `START_PROB` or `END_PROB` made things worse.
- A pause of 0.9 s or more inside a turn splits it at `SILENCE` 0.7; 0.9 keeps those but adds 0.2 s to every reply. Smart Turn is the planned fix.
- Background talk: Silero hears other people's voices as speech, so with talkers nearby the turn starts on their voices and doesn't end until `MAX_SPEECH`. A fixed minimum level (about -35 dBFS when the person is at -22) together with "quieter than the person by 15 dB counts as silence" fixed talk 20 dB below the person, not 10 dB. The level must be set from the robot's mic recordings: it drops quiet speakers.
- Parakeet hears "HARP" as heart, hard or Hart in about half of lines. sherpa-onnx hotwords didn't help (weight 3 added wrong words). Handle it in talk.py's prompt instead.
- STT time: about 0.11 s for a 1–3 s turn, 0.2 s for 3–6 s.

**Talking (proposed changes, not made yet)**
- `GOODBYE` on the person's words ends the conversation on "I have to go to the dentist tomorrow" or "I said goodbye to my parents", and misses "I should get going" or "catch you later". The model's `<bye>` plus `FAREWELL` got the real goodbyes right.
- The grammar keeps replies to two sentences (without it, 21 of 57 replies were longer), but the forced `<bye>` after two sentences came 20 times in 34 conversations, and those replies lose their follow-up question. It also cuts decimals: "about 1. meter tall".
- `speakable` turns "CS106B" into "sixB", a word the voice skips; "1:30" becomes "one:thirty".
- The prompt needs: what HARP stands for and that heart/hard mean HARP, the robot's height, English only (it answered in Spanish, with words the voice can't say), a new wording when asked "Huh?" twice (it repeated itself), no ending just because someone shares a detail it won't take (it said goodbye after being given an address).
- Fillers played before 42 of 112 replies, since the LLM's first sentence took a median 1.17 s on this CPU; with audio playing, the reply waits behind the filler.
- The echo check misses one- or two-word leftovers ("Day going?").
- Voices, judged by Parakeet as the listener: Kokoro 6% word errors (8% through room and robot mic), real-time factor 0.54; Piper 14.5% (16%), factor 0.06.

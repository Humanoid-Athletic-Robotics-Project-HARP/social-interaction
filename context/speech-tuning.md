---
name: speech-tuning
description: Read before changing listen.py's turn settings, talk.py's prompt, grammar, goodbye rules or fillers, or choosing the voice — what synthetic tests found and the changes they point to
metadata:
  type: project
  updated: 2026-11-04T22:44:59Z
  covers: [speech/listen.py, speech/talk.py, speech/bench/*]
---

Synthetic tests on an M1 MacBook Air's CPU (2026-11-03/04), with no mic or speakers: 222 clips from macOS `say` voices with noise, room echo, a rough robot-mic filter, loudness levels and pauses (each clip scored on turns, word errors, false starts and turn-end lag), and 17 scripted conversations through `talk.py --no-play`. The scripts are in `speech/bench/` (`speech/README.md`); results vary by about 3 points between noise draws. Designs: [[listening]], [[talking]].

**Listening**
- Made in `listen.py` from these tests: `MADE_UP` keeps "Bye." and "Thank you." (Parakeet made up no phrase on noise alone: pink, fan, music, footsteps, clicks, bangs); `FLOOR_MARGIN` 1.0; a start count that drops by one on a frame without voice instead of resetting; `PREROLL` 0.8 s. Before them, in noise a quiet syllable delayed or lost the start ("Not much, I'm…" lost its first words). Results on the clips: in noise 10% word errors (47% before), every noisy and quiet clip one turn (23 of 80 noisy clips gave none before), all short words kept, false starts unchanged. Raising `START_PROB` or `END_PROB` made things worse.
- Turn end, on Kokoro's voices (`end_` groups; Smart Turn calls `say`'s flat voices unfinished, so those groups don't test it): Smart Turn ends a finished line a median 0.21 s after the person stops, against 0.69 s for a fixed 0.7 s silence, and all 60 still gave one turn. Its judgement takes about 90 ms on an M1 CPU, so text is ready about 0.45 s after the person stops (0.85 s before).
  - A pause after "um" or "uh" holds the turn: never split, even at 1.6 s (the fixed silence splits every pause of 0.8 s or more).
  - A fragment that Kokoro reads with a falling, finished-sounding end ("I went to the library,", "Well, I think") is cut at a 0.5 s pause. A real person pausing mid-thought usually doesn't sound finished, so only recordings of people can settle this.
  - The 1.5 s fallback caps a misjudged finished line (2 of 60) at 1.55 s; 2.5 s cut fewer 1.6 s pauses but made those lines wait 2.6 s. Judging after 0.1 s instead of 0.2 s split 17 of 80 lines in noise.
- Background talk: Silero hears other people's voices as speech, so with talkers nearby the turn starts on their voices and doesn't end until `MAX_SPEECH`. A fixed minimum level (about -35 dBFS when the person is at -22) together with "quieter than the person by 15 dB counts as silence" fixed talk 20 dB below the person, not 10 dB. The level must be set from the robot's mic recordings: it drops quiet speakers.
- Parakeet hears "HARP" as heart, hard or Hart in about half of lines. sherpa-onnx hotwords didn't help (weight 3 added wrong words). Handle it in talk.py's prompt instead.
- STT time: about 0.11 s for a 1–3 s turn, 0.2 s for 3–6 s.

**Talking** (`bench/talk_bench.py`: 17 scripts, and 6 of open small talk since 2026-11-09)
- Made in `talk.py` from these tests (what each does: [[talking]]): the conversation ends only on a goodbye reply, digits and letters split for the voice, times in words, a "." after a digit not cut as a sentence end while streaming ("about 1. 3 metres"), the short-echo check, and the prompt's facts and rules. Right endings went from 26 to 32 of 34 (the other two said goodbye to someone answering only "No." and "I don't know.", which the prompt asks for).
- The bench also counts what makes it sound canned: replies opening with the same two words, near copies of the prompt's examples, talk about itself nobody asked for, and how often it asks a question.
- With the two-sentence grammar (2026-11-09, Kokoro, 23 scripts twice): 41% of replies opened with the same few words ("I'm sorry", "Oh, nice", "I don't"), 9 copied the prompt's examples, 56% asked a question, 24 were pushed into a `<bye>`, a joke lost its punchline and open questions got dodges ("I don't watch movies… What's your favorite movie?"). So the grammar is gone ([[talking]]).
- Free text needs a length cap: with "one or two sentences" in the prompt alone, the median reply was 23 words and half had three sentences or more. `MAX_SENTENCES` 2 brings it to 13–18 words, none over two sentences.
- With the free prompt (8 scripts, Kokoro): every ending right, 5 of 30 replies asked a question, 24 of 30 opened differently, no copies of the examples, no ignored `<bye>`. It picks favourites and opinions lightly ("I'd pick The Grand Budapest Hotel").
- Still wrong: asked "Huh?" then "Sorry, what did you say?", it gives the same rephrasing twice; when insulted it stays kind but no longer says goodbye.
- Speed on an M1's CPU, free text with Kokoro: LLM's first text 0.6–1.7 s (3.7 s after a 40-word turn), first sentence a median 2.2 s, its audio a median 4.1 s and up to 9.8 s: Kokoro synthesises while the LLM still writes, both on the CPU. Live, through the mic (2026-11-09): first sound 2.9–5.5 s after the person stopped. Test live on a laptop with Piper; Kokoro needs PC2's GPU.
- Fillers came before about half the replies, since the LLM's first sentence took a median 1.2 s on this CPU (`FILLER` is 1.2 s). Recheck on PC2's GPU before changing it; out loud, the reply waits behind the filler.
- Voices, judged by Parakeet as the listener (`bench/voice_bench.py`): Kokoro 6% word errors (7% through room and robot mic), real-time factor 0.56; Piper 14.5% (18%), factor 0.06. Kokoro, if it reaches its estimated 0.35 s on PC2's GPU.

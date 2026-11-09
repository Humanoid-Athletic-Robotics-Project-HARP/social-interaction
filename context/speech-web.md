---
name: speech-web
description: Read before changing or using the speech web page (speech/web.py, web.html), or what listen.py and talk.py print for it
metadata:
  type: project
  updated: 2026-11-09T03:43:53Z
  covers: [speech/web.py, speech/web.html]
---

`speech/web.py` serves one page (`speech/web.html`) to run and watch a conversation. Use: `speech/README.md`. Designs of the two halves: [[listening]], [[talking]].

**What the page shows:** a line saying what to do now; a strip of the four stages (sound in, listen.py, talk.py and the LLM, sound out) with each one's live state; a Now card with the turn in progress (its speech and silence against `CHECK`, `SILENCE` and `WAIT`, and Smart Turn's verdict) and then HARP's wait (against `FILLER` and `GIVE_UP`), and the turn and time limits; loudness, Silero and a 12 s chart; median speeds; the conversation, where the person's bubble fills in while they speak and HARP's piece by piece as it speaks, each reply with a bar of hearing, thinking, voice and waiting time; the programs' logs; a note with the sounds of each word talk.py had to guess; warnings for a silent mic, a missing model, no LLM or a crash. The limits come from `listen.py` and `talk.py` (`LIMITS` in `web.py`), so the page follows their settings.

**How it runs them**
- It starts `listen.py --events` and `talk.py --heard` as its own child processes and is the pipe between them; it doesn't attach to copies started in a terminal. It passes talk.py only the turns, never listen.py's event lines.
- Audio stays on the computer running `web.py`: the mic, the robot's stream or a WAV in, its speakers out. The browser only shows and sends text. On macOS the mic permission belongs to the app that started `web.py`.
- A typed line goes to talk.py as `{"text": ...}`, the same as a heard turn, so typing and speaking mix in one conversation.
- Turns are passed on only after talk.py's greeting (its first `said`). With a recording and talking, listen.py starts once the greeting has played: a recording can't wait for the robot the way a person does.
- The page's settings only pick the flags for the next Start; nothing on the robot changes. "The robot's mic" only reads its stream; the voice assistant's mode is switched separately.
- One session at a time. A `listen.py` started elsewhere holds the speaking signal's port (28732), so the page refuses to start listening then and says why.
- `/api/status` includes a process instance ID so the unified viewer can reset history after a service restart.
- Events reach the page as server-sent events; a page opened later gets the session so far (levels excepted).
- The Start button for the LLM runs the README's `llama-server` command on the `--llm` address's port, and `web.py` stops it on exit. Only its startup lines reach the log: on an M1 it prints Metal compile errors that are harmless.

**Ports and safety:** the page is on 28733, beside 28731 (LLM) and 28732 (speaking signal), and only on 127.0.0.1 unless `--host 0.0.0.0`. Commands must be JSON, so another site's page can't post them without a CORS check, which `web.py` never passes. `--llm` and `--mute-port` let a second copy run beside a terminal conversation without muting its listener or sharing its LLM cache.

Plain stdlib Python, so it also runs on PC2's 3.8. No outside scripts, styles or fonts: the robot network may be offline.

Tested on an M1 MacBook Air (2026-11-08): listening only and a whole conversation from a `say` recording at real speed, and typed lines, at desktop and phone widths. Not yet with a live mic or speakers, or on PC2.

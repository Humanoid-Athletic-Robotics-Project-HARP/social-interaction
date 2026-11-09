---
name: speech-web
description: Read before changing or using the speech web page (speech/web.py, web.html), or what listen.py and talk.py print for it
metadata:
  type: project
  updated: 2026-11-09T00:37:23Z
  covers: [speech/web.py, speech/web.html]
---

`speech/web.py` serves one page (`speech/web.html`) to run and watch a conversation: start and stop listening and talking, see listening's state, loudness and Silero live, the conversation with each turn's timings, both programs' logs, and type a line for the person. Use: `speech/README.md`. Designs of the two halves: [[listening]], [[talking]].

**How it runs them**
- It starts `listen.py --events` and `talk.py --heard` as its own child processes and is the pipe between them; it doesn't attach to copies started in a terminal. It passes talk.py only the turns, never listen.py's event lines.
- Audio stays on the computer running `web.py`: the mic, the robot's stream or a WAV in, its speakers out. The browser only shows and sends text. On macOS the mic permission belongs to the app that started `web.py`.
- A typed line goes to talk.py as `{"text": ...}`, the same as a heard turn, so typing and speaking mix in one conversation.
- Turns are passed on only after talk.py's greeting (its first `said`).
- One session at a time. A `listen.py` started elsewhere holds the speaking signal's port (28732), so the page refuses to start listening then and says why.
- Events reach the page as server-sent events; a page opened later gets the session so far (levels excepted).
- The Start button for the LLM runs the README's `llama-server` command on the `--llm` address's port, and `web.py` stops it on exit.

**Ports and safety:** the page is on 28733, beside 28731 (LLM) and 28732 (speaking signal), and only on 127.0.0.1 unless `--host 0.0.0.0`. Commands must be JSON, so another site's page can't post them without a CORS check, which `web.py` never passes. `--llm` and `--mute-port` let a second copy run beside a terminal conversation without muting its listener or sharing its LLM cache.

Plain stdlib Python, so it also runs on PC2's 3.8. No outside scripts, styles or fonts: the robot network may be offline.

Tested on an M1 MacBook Air (2026-11-08) with `say` recordings at real speed; not yet with a live mic or on PC2.

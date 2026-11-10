---
name: speech-web
description: Read before changing the speech supervisor, its embedded UI, event stream or listen.py/talk.py handshake; use unified-viewer for viewing
metadata:
  type: project
  updated: 2026-11-10T04:30:58Z
  covers: [speech/web.py, speech/web.html]
---

Use [[unified-viewer]] to show speech and listening. `speech/web.py` is the underlying supervisor and serves the presentation reused by the dashboard. Setup belongs in `speech/README.md`; speech design belongs in [[listening]] and [[talking]].

- The supervisor owns `listen.py --events` and `talk.py --heard`, forwards only transcripts between them, and publishes their events/logs. It cannot attach to independently launched terminal copies.
- An LLM server alone does not provide a speech feed. Keep the supervisor running separately. Starting the supervisor opens no microphone or conversation; its page's Start launches the selected processes.
- Audio belongs to the supervisor's machine, not the browser. macOS microphone permission belongs to the app that launched it. Typed lines reach talk.py through the same conversation pipe.
- Transcripts are forwarded only after talk.py's greeting. For a WAV conversation the listener starts after the greeting, since a recording cannot wait like a person.
- Settings choose child-process flags for the next Start. Selecting robot mic reads its stream; changing Unitree's assistant mode is a separate action. Another listener holding the mute port prevents a duplicate session.
- The page's Turns and Time follow `talk.py`'s own counts: Time starts at `talk_ready` (the greeting, after loading), freezes at `done`, and is corrected for the browser's clock against event times. The reply bar runs from the turn's end to the first non-canned `speaking` event and explains `talk.py`'s `filler` event (said, or why not).
- **Done talking** posts `/api/end_turn`: web.py writes a line to `listen.py`'s stdin (it gets a pipe), which ends the turn being heard at once (`ended_early` on its line).
- `/api/status` has a process instance ID for dashboard history resets. SSE provides live updates and bounded history for late subscribers; level events are not kept in history. Preserve these contracts when editing the programs' output.
- Its LLM Start uses the documented llama-server command. Exit stops only an LLM it started itself. A person's separately launched LLM remains theirs.

The supervisor binds loopback by default; explicit network binding enables a remote source. Commands require JSON. It uses the existing LLM and mute ports; the authoritative defaults are in the scripts and README. The page is self-contained for offline robot networks.

Validated with recordings and typed conversations on an M1 Mac, including dashboard embedding. Live microphone/speaker conversations and deployment on PC2 remain unverified.

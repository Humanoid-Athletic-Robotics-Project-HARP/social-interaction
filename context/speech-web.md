---
name: speech-web
description: Read before changing the speech supervisor, its embedded UI, event stream or listen.py/talk.py handshake; use unified-viewer for viewing
metadata:
  type: project
  updated: 2026-11-11T23:31:24Z
  covers: [speech/web.py, speech/web.html]
---

Use [[unified-viewer]] to show speech and listening. `speech/web.py` is the underlying supervisor and serves the presentation reused by the dashboard. Setup belongs in `speech/README.md`; speech design belongs in [[listening]] and [[talking]].

- The supervisor owns `listen.py --events` and `talk.py --heard`, forwards only transcripts between them, and publishes their events/logs. It cannot attach to independently launched terminal copies.
- An LLM server alone does not provide a speech feed. Keep the supervisor running separately. Starting the supervisor opens no microphone or conversation; its page's Start launches the selected processes.
- Audio belongs to the supervisor's machine, not the browser. macOS microphone permission belongs to the app that launched it. Typed lines reach talk.py through the same conversation pipe.
- Transcripts are forwarded only after talk.py's greeting. For a WAV conversation the listener starts after the greeting, since a recording cannot wait like a person.
- Settings choose child-process flags for the next Start. Selecting robot mic reads its stream; changing Unitree's assistant mode is a separate action. Another listener holding the mute port prevents a duplicate session.
- The page's Turns and Time follow `talk.py`'s own counts: Time starts at `talk_ready` (the greeting, after loading), freezes at `done`, and is corrected for the browser's clock against event times. The reply bar runs from the turn's end to the first non-canned `speaking` event and explains `talk.py`'s `filler` event (said, or why not).
- Stop lets `talk.py` finish its line (up to `STOP_S`). Start answers 409 until that session is `done` (its programs ended and their exit events published, then `stopped`), whether it was stopped or its programs ended by themselves, or two would speak at once and the old events would land in the new history. `speech/tests/test_web.py` checks this with a stand-in `talk.py`.
- A Start with `"wait": true` (talking only) runs `talk.py --wait`: it loads, sends `loaded` and holds its greeting until `/api/go`, which web.py keeps until `loaded` if it comes first (`go_asked`), and sends once (a `go` event). `/api/status` `waiting` is true until then. Start's 409 while the last session stops has `"stopping": true`. `speech/bridge.py` uses these to greet the moment the robot arrives ([[talking]]); the page doesn't.
- **Done talking** posts `/api/end_turn`: web.py writes a line to `listen.py`'s stdin (it gets a pipe), which ends the turn being heard at once (`ended_early` on its line).
- `/api/status` has a process instance ID for dashboard history resets. SSE provides live updates and bounded history for late subscribers; level events are not kept in history. Preserve these contracts when editing the programs' output.
- Its LLM Start uses the documented llama-server command. Exit (Ctrl-C, or SIGTERM or SIGHUP, as when its terminal or SSH session closes; further ones are ignored during clean-up) waits for a Stop under way and stops only an LLM it started itself. A person's separately launched LLM remains theirs.

The supervisor binds loopback by default; explicit network binding enables a remote source. Bound to loopback, it refuses a request whose Host is not loopback (`loopback_host`: localhost or a loopback IP), which stops a page on another site that re-resolves its own name to 127.0.0.1 (DNS rebinding) from reading transcripts or starting the mic; with a network binding it accepts any Host, so keep it on a trusted network. Commands require JSON. It uses the existing LLM and mute ports; the authoritative defaults are in the scripts and README. The page is self-contained for offline robot networks.

Validated with recordings and typed conversations on an M1 Mac, including dashboard embedding, and live mic and speaker conversations through it (2026-11-09). Deployment on PC2 remains unverified.

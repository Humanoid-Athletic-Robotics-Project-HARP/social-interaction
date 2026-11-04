"""How clearly each voice comes through (context/speech-tuning.md): HARP's lines in each
voice, as they are and through a room, the robot's mic and its fan, judged by Parakeet as
the listener. Also each voice's real-time factor on this computer.

  python3 bench/voice_bench.py      (from speech/, in its venv)
"""

import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
import listen  # noqa: E402
import talk  # noqa: E402
from clips import PERSON_DB, at_db, fan, reverb, robot_mic  # noqa: E402
from listen_bench import errors  # noqa: E402

LINES = [talk.GREETING, talk.CLOSE, "Oh, nice! What was the class about?",
         "Ha, that sounds tough. Do you have more classes today?",
         "I am! I'm a Unitree G1 humanoid running software the team wrote.",
         "No problem, it was lovely meeting you. Have a great day!",
         "That sounds like a fun weekend. Are you doing anything tonight?",
         "Wow, twenty-one! Happy birthday, Jordan!"]


def main():
    text = listen.Text(listen.models_dir() / listen.PARAKEET, "cpu")
    for name in sorted(talk.VOICES):
        voice = talk.Voice(name, "cpu")
        wrong = {"as it is": [0, 0], "room and robot mic": [0, 0]}
        making = length = 0.0
        for line in LINES:
            began = time.perf_counter()
            audio = voice(talk.speakable(line))
            making += time.perf_counter() - began
            audio = np.interp(np.arange(0, len(audio) / voice.rate, 1 / listen.RATE),
                              np.arange(len(audio)) / voice.rate, audio).astype(np.float32)
            length += len(audio) / listen.RATE
            for way in wrong:
                heard = at_db(audio, PERSON_DB)
                if way != "as it is":
                    heard = at_db(robot_mic(reverb(heard, 0.5)), PERSON_DB)
                    heard = heard + fan(len(heard)) * 10 ** ((PERSON_DB - 12) / 20)
                said = text(heard)
                e, n = errors(line, said)
                wrong[way][0] += e
                wrong[way][1] += n
                if e and way == "as it is":
                    print(f"  {name}: {said!r} for {line!r}")
        print(f"{name}: real-time factor {making / length:.2f}; word errors "
              + ", ".join(f"{way} {100 * e / n:.1f}%" for way, (e, n) in wrong.items()))


if __name__ == "__main__":
    main()

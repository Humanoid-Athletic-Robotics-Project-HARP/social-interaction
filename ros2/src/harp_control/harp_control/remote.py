"""The G1 remote, as rt/lowstate reports it in wireless_remote (40 bytes):
bytes 2 and 3 hold one bit per button, then the sticks as little-endian
floats (lx, rx, ry, an unused L2 value, ly). Layout from unitree_sdk2_python's
example/wireless_controller.

Every action has one remote button and one keyboard key, and the walker
(gr00t/walk.py) turns remote presses into those keys, so both do the same
thing in every program. Urgent actions take one button; deliberate ones
(starting, handing control back, height) take R1 plus a button, as Shift
plus a key on the keyboard:

  Action                         Remote              Keyboard
  Stop: damp every joint         B                   Space (or o)
  Drive by hand, overriding      hold L1 + sticks    w/s a/d q/e
    the program                  (left: move, right: turn)
  Stop moving, keep balancing    let go of L1        z
  Start the policy (balance)     R1 + start          Shift+P
  Hand control back to the       R1 + A              Shift+G
    program (moves, /cmd_vel)
  Raise / lower                  R1 + up / down      Shift+R / Shift+F

The sim (harp_sim) writes presses and stick positions into its rt/lowstate
the same way.
"""

import math
import struct

BUTTONS = {   # name: (byte, bit)
    "R1": (2, 0), "L1": (2, 1), "start": (2, 2), "select": (2, 3),
    "R2": (2, 4), "L2": (2, 5), "F1": (2, 6), "F3": (2, 7),
    "A": (3, 0), "B": (3, 1), "X": (3, 2), "Y": (3, 3),
    "up": (3, 4), "right": (3, 5), "down": (3, 6), "left": (3, 7),
}
STICKS = {"lx": 4, "rx": 8, "ry": 12, "ly": 20}   # name: byte offset of its float
STICK_DEADZONE = 0.1      # sticks read zero inside this, so a resting stick doesn't creep

# One button, checked every 5 ms whatever else is held: damps the robot from any
# state. (Unitree's g1_ctrl uses L2+B, which includes it.)
ESTOP = "B"
# Held: manual drive with the sticks, overriding whatever the program commands.
MANUAL = "L1"
# Held with one of COMBOS for a deliberate action.
COMBO = "R1"
COMBOS = {"start": "P", "A": "G", "up": "R", "down": "F"}   # button: the keyboard key it stands for

# Keyboard keys, as the terminal reads them (Shift+P arrives as "P").
STOP_KEYS = (" ", "o")
START_KEY, HAND_BACK_KEY, RAISE_KEY, LOWER_KEY = "P", "G", "R", "F"
DRIVE_KEYS = {"w": (0, 1), "s": (0, -1), "a": (1, 1), "d": (1, -1), "q": (2, 1), "e": (2, -1)}
HALT_KEY = "z"


def pressed(wireless_remote):
    """The names of the buttons held down."""
    return {name for name, (byte, bit) in BUTTONS.items() if (wireless_remote[byte] >> bit) & 1}


def sticks(wireless_remote):
    """Stick positions, -1 to 1, zero inside the deadzone: lx, ly (left stick right,
    forward) and rx, ry (right stick)."""
    values = {}
    for name, offset in STICKS.items():
        value = struct.unpack("<f", bytes(wireless_remote[offset:offset + 4]))[0]
        if not math.isfinite(value) or abs(value) < STICK_DEADZONE:
            value = 0.0
        values[name] = max(-1.0, min(1.0, value))
    return values


def combo_keys(held, before):
    """Keys for the R1 combos newly pressed: R1 held, the other button just
    pressed, and nothing else held."""
    if COMBO not in held:
        return []
    others = held - {COMBO}
    return [COMBOS[b] for b in others - before if b in COMBOS and len(others) == 1]


def encode(names, wireless_remote, stick_values=None):
    """Sets the bits for these buttons (and clears the others), and the sticks
    (zero unless given), in a 40-byte list."""
    wireless_remote[2] = wireless_remote[3] = 0
    for name in names:
        byte, bit = BUTTONS[name]
        wireless_remote[byte] |= 1 << bit
    stick_values = stick_values or {}
    for name, offset in STICKS.items():
        wireless_remote[offset:offset + 4] = list(struct.pack("<f", float(stick_values.get(name, 0.0))))

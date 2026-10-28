"""The G1 remote's buttons, as rt/lowstate reports them in wireless_remote
(40 bytes). Bytes 2 and 3 hold one bit per button; the sticks follow as
floats and aren't used here. Layout from unitree_sdk2_python's
example/wireless_controller.

The walker reads the e-stop chord from here, and the sim (harp_sim) writes
presses into its rt/lowstate the same way.
"""

BUTTONS = {   # name: (byte, bit)
    "R1": (2, 0), "L1": (2, 1), "start": (2, 2), "select": (2, 3),
    "R2": (2, 4), "L2": (2, 5), "F1": (2, 6), "F3": (2, 7),
    "A": (3, 0), "B": (3, 1), "X": (3, 2), "Y": (3, 3),
    "up": (3, 4), "right": (3, 5), "down": (3, 6), "left": (3, 7),
}
# Damps the robot from any state. Unitree's g1_ctrl uses the same chord (LT+B,
# LT being L2) for its Passive state, so HARP operators know it.
ESTOP = frozenset({"L2", "B"})


def pressed(wireless_remote):
    """The names of the buttons held down."""
    return {name for name, (byte, bit) in BUTTONS.items() if (wireless_remote[byte] >> bit) & 1}


def encode(names, wireless_remote):
    """Sets the bits for these buttons (and clears the others) in a 40-byte list."""
    wireless_remote[2] = wireless_remote[3] = 0
    for name in names:
        byte, bit = BUTTONS[name]
        wireless_remote[byte] |= 1 << bit

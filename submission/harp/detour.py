"""Waiting for a blocked short way instead of turning round onto Nav2's long detour.

With no map, Nav2's planner takes unseen space as free. When something still stands in the
way (a cart waiting to give way, a person stopped in a corridor), it is in Nav2's map and the
next path is a long detour, often through rooms or wall gaps it hasn't seen. In HIM's office
(run 2026-11-03_04-03-24Z) the robot stood 3 m from the goal when a waiting cart and a person
closed the corridor: Nav2's path jumped from 3 m to 11-16 m and the robot walked back 15 m;
the 3 m way was open again 12 s later. At the corner (2026-11-03_01-22-16Z) the path flipped
15 times in 70 s between the corner and a 23 m detour, each flip a turn on the spot.

A person waits a little for a cart to pass. DetourPatience watches the lengths of Nav2's
paths: one RATIO times and EXTRA m longer than the shortest of the last MEMORY s is a detour,
and the robot stands (every usual check still runs) until a path about as short as before
comes back, or for at most PATIENCE s; then the detour is taken. A new goal starts afresh.
"""

import math

RATIO = 1.5       # a path this many times as long as the recent shortest...
EXTRA = 3.0       # m ... and this much longer is a detour round something in the way
BACK = 1.0        # m: a path at most this much longer than the short one again ends the wait
MEMORY = 10.0     # s of paths the shortest is taken from
PATIENCE = 15.0   # s standing at most (Nav2's progress checker allows 20 s without 0.3 m)


def path_length(points):
    """Length in m of a path given as (x, y) points."""
    return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:]))


class DetourPatience:
    def __init__(self):
        self.reset()

    def reset(self):
        """A new goal: no paths seen yet."""
        self.history = []     # (time, length) of recent paths
        self.since = None     # when the current wait began
        self.short = None     # the short way's length the wait is for

    def update(self, now, length):
        """A new path of this length at time now (s). Returns 'wait', 'resume' or 'give_up'
        when the wait starts, ends because the short way is back, or ends after PATIENCE;
        else None."""
        if not math.isfinite(length) or length <= 0:
            return None
        self.history = [(t, l) for t, l in self.history if 0 <= now-t <= MEMORY]
        change = None
        if self.since is None:
            shortest = min((l for _, l in self.history), default=None)
            if shortest is not None and length > RATIO*shortest and length-shortest > EXTRA:
                self.since, self.short, change = now, shortest, "wait"
        elif length <= self.short+BACK:
            self.since, change = None, "resume"
        elif now-self.since >= PATIENCE:
            # Take the detour: it is the way now, not a jump from what came before.
            self.since, self.history, change = None, [], "give_up"
        self.history.append((now, length))
        return change

    def waiting(self, now):
        """True while the robot should stand for the short way."""
        return self.since is not None and now-self.since < PATIENCE

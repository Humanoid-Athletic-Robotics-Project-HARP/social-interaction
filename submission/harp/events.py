"""Our events: one JSON object per line on the container's output, and on /harp/events
(std_msgs/String, transient local) for the other programs in the image.

The container's output is the only thing that leaves it (HIM's policy.log in the sim;
course.py copies our lines into the run's events.jsonl). Every record starts the same:
  {"source": the program, "event": its name, "wall": Unix time (s),
   "sim_time": the sim's clock (s), only from programs that read HIM's state, ...}
"""

import json
import time

TOPIC = "/harp/events"


def record(source, name, sim_time=None, **fields):
    """One event as a line of JSON."""
    head = {"source": source, "event": name, "wall": round(time.time(), 3)}
    if sim_time is not None:
        head["sim_time"] = round(float(sim_time), 3)
    return json.dumps({**head, **fields})


def emit(source, name, sim_time=None, **fields):
    """Print one event (for programs that don't publish on /harp/events)."""
    print(record(source, name, sim_time, **fields), flush=True)


def latched():
    """The QoS of /harp/events: late joiners get every earlier event."""
    from rclpy.qos import DurabilityPolicy, QoSProfile
    return QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Events:
    """A program's events: events(name, sim_time=None, **fields) prints and publishes one."""

    def __init__(self, node, source):
        from std_msgs.msg import String
        self.source, self.String = source, String
        self.pub = node.create_publisher(String, TOPIC, latched())

    def __call__(self, name, sim_time=None, **fields):
        text = record(self.source, name, sim_time, **fields)
        self.pub.publish(self.String(data=text))
        print(text, flush=True)

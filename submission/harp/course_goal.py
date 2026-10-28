"""Send the world's goal to Nav2 once, report how it went, then exit. entrypoint.sh
starts it in a world's image (submission/sim.sh --world NAME).

The goal is the world's goal site, in odom, which in HIM's sim is the world frame
(harp/state.py uses the sim's true position). It goes out once HIM's sim has
handed control to our policy (harp_node's control_ready event), Nav2 is active,
and SETTLE s have passed. Events on /harp/events: goal_sent (x, y, yaw) and
goal_result (status, recoveries, seconds, remaining). If Nav2 isn't active
within NAV2_START s, the result is NAV2_NOT_ACTIVE.
"""

import json
import math
import sys
import time

import mujoco
import rclpy
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

WORLD = "/harp/model/lidar_scene.xml"
NAV2_START = 60.0     # s for Nav2 to be active (it takes about 20 with STVL)
SETTLE = 2.0          # s after the handoff before the goal goes out
TIMEOUT = 240.0       # s to wait for Nav2's result before cancelling
STATUS = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "FAILED",
          GoalStatus.STATUS_CANCELED: "CANCELED"}


def goal_site():
    model = mujoco.MjModel.from_xml_path(WORLD)
    return model.site("goal").pos[:2].tolist()


def wait_for_active(node, name, limit):
    """True once the lifecycle node is active, False after limit seconds."""
    client = node.create_client(GetState, name + "/get_state")
    deadline = time.monotonic() + limit
    while time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=1.0):
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
            if future.result() is not None and future.result().current_state.label == "active":
                return True
        time.sleep(0.5)
    return False


def spin_until(node, done, limit=math.inf):
    deadline = time.monotonic() + limit
    while not done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return done()


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("course_goal")
    latched = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    events_pub = node.create_publisher(String, "/harp/events", latched)

    def event(name, **fields):
        text = json.dumps({"source": "goal", "event": name, **fields})
        events_pub.publish(String(data=text))
        print(text, flush=True)

    ready = []
    node.create_subscription(
        String, "/harp/events",
        lambda m: ready.append(True) if json.loads(m.data).get("event") == "control_ready" else None,
        latched)
    x, y = goal_site()
    if not wait_for_active(node, "bt_navigator", NAV2_START):
        event("goal_result", status="NAV2_NOT_ACTIVE", recoveries=0, seconds=0.0, remaining=None)
        return 1
    spin_until(node, lambda: bool(ready))
    spin_until(node, lambda: False, SETTLE)

    nav = ActionClient(node, NavigateToPose, "navigate_to_pose")
    nav.wait_for_server()
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "odom"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
    goal.pose.pose.orientation.w = 1.0
    feedback = {}
    start = time.monotonic()
    event("goal_sent", x=x, y=y, yaw=0.0)
    sent = nav.send_goal_async(goal, feedback_callback=lambda f: feedback.update(f=f.feedback))
    spin_until(node, sent.done)
    handle = sent.result()
    status = None
    if not handle.accepted:
        status = "REJECTED"
    else:
        result = handle.get_result_async()
        if not spin_until(node, result.done, TIMEOUT):
            handle.cancel_goal_async()
            status = "TIMEOUT"
        else:
            status = STATUS.get(result.result().status, "UNKNOWN")
    f = feedback.get("f")
    remaining = getattr(f, "distance_remaining", None)
    event("goal_result", status=status, recoveries=getattr(f, "number_of_recoveries", 0),
          seconds=round(time.monotonic() - start, 1),
          remaining=None if remaining is None else round(remaining, 2))
    nav.destroy()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

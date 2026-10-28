"""Send the world's goal, or a waypoint route, to Nav2, report how it went, then exit.
entrypoint.sh starts it in a world's image (submission/sim.sh --world NAME).

The goal is the world's goal site, in odom (harp/state.py's leg odometry, which
starts at the world's origin with the robot). A route in G1_TASK_COMMAND
({"route": {...}}, harp/route.py; course.py --route sets it) replaces it: each
point in turn, laps times, moving on only once Nav2 reports the point reached.
A rejection, failure or timeout ends the route there. Everything goes out once
HIM's sim has handed control to our policy (harp_node's control_ready event),
Nav2 is active, and SETTLE s have passed.

Events on /harp/events: goal_sent (x, y, yaw: the last point; and for a route,
waypoints and laps) and goal_result (status, recoveries, seconds, remaining; and
for a route, completed_waypoints and completed_laps), around the whole goal or
route. A route also logs waypoint_sent (lap, waypoint, x, y, yaw) and
waypoint_result (lap, waypoint, status, seconds, recoveries) for each point. If
Nav2 isn't active within NAV2_START s, the result is NAV2_NOT_ACTIVE.

Each point gets TIMEOUT s, finding Nav2's action server and its accepting the
goal included, so no wait here is unbounded. A timeout cancels the goal and
waits up to CANCEL_TIMEOUT s for Nav2 to confirm (CANCEL_TIMEOUT if it doesn't).
"""

import json
import math
import os
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

from harp.route import Route, route_from_data

WORLD = "/harp/model/lidar_scene.xml"
NAV2_START = 60.0     # s for Nav2 to be active (it takes about 20 with STVL)
SETTLE = 2.0          # s after the handoff before the goal goes out
TIMEOUT = 240.0       # s for each point, from sending it to Nav2's result
CANCEL_TIMEOUT = 5.0  # s for Nav2 to confirm a cancelled goal has ended
STATUS = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "FAILED",
          GoalStatus.STATUS_CANCELED: "CANCELED"}


def goal_site():
    model = mujoco.MjModel.from_xml_path(WORLD)
    return model.site("goal").pos[:2].tolist()


def route_from_env():
    """The route in G1_TASK_COMMAND, or None; ValueError if it is malformed."""
    try:
        task = json.loads(os.environ.get("G1_TASK_COMMAND") or "null")
    except ValueError:
        return None
    if not isinstance(task, dict) or "route" not in task:
        return None
    return route_from_data(task["route"])


def is_control_ready(text):
    """Other programs share /harp/events: a line that isn't our JSON is skipped."""
    try:
        record = json.loads(text)
    except ValueError:
        return False
    return isinstance(record, dict) and record.get("event") == "control_ready"


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


def cancel(node, handle, result):
    """Cancel a goal; True once Nav2 has ended it."""
    cancelling = handle.cancel_goal_async()
    spin_until(node, lambda: cancelling.done() and result.done(), CANCEL_TIMEOUT)
    return result.done()


def go_to(node, nav, point, deadline):
    """One Nav2 goal at (x, y, heading deg) by the monotonic deadline: its status, and
    its last feedback (or None)."""
    if not nav.wait_for_server(timeout_sec=max(0.0, deadline - time.monotonic())):
        return "TIMEOUT", None
    x, y, heading = point
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "odom"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
    yaw = math.radians(heading)
    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    feedback = {}
    sent = nav.send_goal_async(goal, feedback_callback=lambda f: feedback.update(f=f.feedback))
    if not spin_until(node, sent.done, deadline - time.monotonic()):
        return "TIMEOUT", None
    handle = sent.result()
    if not handle.accepted:
        return "REJECTED", None
    result = handle.get_result_async()
    try:
        if spin_until(node, result.done, deadline - time.monotonic()):
            return STATUS.get(result.result().status, "UNKNOWN"), feedback.get("f")
    except BaseException:
        cancel(node, handle, result)    # don't leave Nav2 driving to a point nobody watches
        raise
    return ("TIMEOUT" if cancel(node, handle, result) else "CANCEL_TIMEOUT"), feedback.get("f")


def follow(node, nav, route, event, waypoint_events):
    """Every point of the route in turn; the goal_result's fields."""
    x, y, yaw = route.waypoints[-1]
    plan = {"waypoints": [list(p) for p in route.waypoints], "laps": route.laps} if waypoint_events else {}
    event("goal_sent", x=x, y=y, yaw=yaw, **plan)
    start = time.monotonic()
    status, recoveries, remaining, completed, message = "SUCCEEDED", 0, None, 0, None
    try:
        while route.current() is not None:
            point, lap, waypoint = route.current(), route.completed_laps + 1, route.index + 1
            if waypoint_events:
                event("waypoint_sent", lap=lap, waypoint=waypoint, x=point[0], y=point[1], yaw=point[2])
            began = time.monotonic()
            status, f = go_to(node, nav, point, began + TIMEOUT)
            recoveries += getattr(f, "number_of_recoveries", 0)
            remaining = getattr(f, "distance_remaining", None)
            if waypoint_events:
                event("waypoint_result", lap=lap, waypoint=waypoint, status=status,
                      seconds=round(time.monotonic() - began, 1),
                      recoveries=getattr(f, "number_of_recoveries", 0))
            if status != "SUCCEEDED":
                break
            completed += 1
            route.advance()
    except Exception as e:
        status, message = "ERROR", str(e)
    result = {"status": status, "recoveries": recoveries, "seconds": round(time.monotonic() - start, 1),
              "remaining": None if remaining is None else round(remaining, 2)}
    if waypoint_events:
        result.update(completed_waypoints=completed, completed_laps=route.completed_laps)
    if message:
        result["message"] = message
    event("goal_result", **result)
    return status


def run(node):
    latched = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    events_pub = node.create_publisher(String, "/harp/events", latched)

    def event(name, **fields):
        text = json.dumps({"source": "goal", "event": name, **fields})
        events_pub.publish(String(data=text))
        print(text, flush=True)

    try:
        route = route_from_env()
    except ValueError as e:
        event("goal_result", status="ERROR", recoveries=0, seconds=0.0, remaining=None,
              message=f"bad route: {e}")
        return 1
    waypoint_events = route is not None
    if route is None:
        x, y = goal_site()
        route = Route([(x, y, 0.0)])

    ready = []
    node.create_subscription(
        String, "/harp/events", lambda m: ready.append(True) if is_control_ready(m.data) else None, latched)
    if not wait_for_active(node, "bt_navigator", NAV2_START):
        event("goal_result", status="NAV2_NOT_ACTIVE", recoveries=0, seconds=0.0, remaining=None)
        return 1
    spin_until(node, lambda: bool(ready))
    spin_until(node, lambda: False, SETTLE)

    nav = ActionClient(node, NavigateToPose, "navigate_to_pose")
    try:
        return 0 if follow(node, nav, route, event, waypoint_events) == "SUCCEEDED" else 1
    finally:
        nav.destroy()


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("course_goal")
    try:
        return run(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())

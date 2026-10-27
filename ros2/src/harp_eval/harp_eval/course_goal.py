"""Send one Nav2 goal for a course test and report how it went, then exit.
course.launch.py starts it and shuts the run down when it exits.

The goal is the course's goal site (from /sim/info), in odom, the frame
that starts where the policy started, so a goal at (7.5, 0) is 7.5 m ahead
of the start. Parameters:
  goal      "X Y" or "X Y YAW_DEG" to override the course's goal
  timeout   seconds to wait for Nav2 before cancelling (default 240)
Events on /harp/events: goal_sent (x, y, yaw) and goal_result (status,
recoveries, seconds, distance remaining). If Nav2 isn't active within
NAV2_START s of the policy starting (a plugin failed to load, say), the
result is NAV2_NOT_ACTIVE and the run ends instead of waiting forever.

The sim holds the band (run_sim --hold-band) until this asks for its release
(a release_band event), once Nav2 is active, no other course run is still
starting up (course.py), and the sim hasn't lagged for QUIET s: start-ups lag
the sims, which only harms a robot off the band. The goal goes out
RELEASE_SETTLE s after the release.
"""

import json
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from harp_control import runlog
from harp_eval.course import set_state, starting

SETTLE = 3.0              # s to wait after the result, so the robot comes to rest on record
NAV2_START = 60.0         # s after the policy starts for Nav2 to be active (it takes about 5)
RELEASE_SETTLE = 2.0      # s after the band release before the goal goes out
QUIET = 5.0               # s without a sim lag event before asking for the release
QUIET_MAX = 60.0          # s to wait for that quiet before asking anyway


def wait_for_active(node, name, limit):
    """True once the lifecycle node is active, False after limit seconds."""
    client = node.create_client(GetState, f"{name}/get_state")
    deadline = time.monotonic() + limit
    while time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=1.0):
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
            if future.result() is not None and future.result().current_state.label == "active":
                return True
        time.sleep(0.5)
    return False


def run():
    rclpy.init(args=sys.argv)
    nav = BasicNavigator(node_name="course_goal")
    nav.declare_parameter("goal", "")
    nav.declare_parameter("timeout", 240.0)
    override = nav.get_parameter("goal").value.split()
    timeout = nav.get_parameter("timeout").value
    latched = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    events_pub = nav.create_publisher(String, "/harp/events", latched)
    runlog.attach(lambda text: events_pub.publish(String(data=text)))

    info, policy_on, released, lags = {}, [], [], [0.0]
    nav.create_subscription(String, "/sim/info", lambda m: info.update(json.loads(m.data)),
                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_event(msg):
        e = json.loads(msg.data)
        if e["source"] == "walker" and e["event"] == "policy_on":
            policy_on.append(e)
        elif e["source"] == "sim" and e["event"] == "band_released":
            released.append(e)
        elif e["source"] == "sim" and e["event"] == "lag":
            lags[0] = max(lags[0], e["t"])
    nav.create_subscription(String, "/harp/events", on_event, latched)

    # The goal, then the walker running GR00T, then Nav2 up (it starts once the policy is on).
    while not (override or info) or not policy_on:
        rclpy.spin_once(nav, timeout_sec=0.2)
    if override:
        x, y = float(override[0]), float(override[1])
        yaw = math.radians(float(override[2])) if len(override) > 2 else 0.0
    elif info.get("goal"):
        (x, y), yaw = info["goal"], 0.0
    else:
        nav.get_logger().error("No goal: the world has no goal site and no goal parameter was given.")
        return 1
    # Not waitUntilNav2Active(): it waits for AMCL, which we don't run. Nor
    # _waitForNodeToActivate(): it waits forever if Nav2's bringup aborted.
    if not wait_for_active(nav, "bt_navigator", NAV2_START):
        runlog.event("goal", "goal_result", status="NAV2_NOT_ACTIVE", recoveries=0, seconds=0.0,
                     remaining=None)
        print(f"Result: NAV2_NOT_ACTIVE: bt_navigator wasn't active {NAV2_START:.0f} s after "
              "the policy started; see nav2.log", flush=True)
        nav.destroy_node()
        rclpy.try_shutdown()
        return 1

    # Release together with the other course runs, after the start-ups' lag.
    lock = Path(os.environ["HARP_DDS_LOCK"]) if os.environ.get("HARP_DDS_LOCK") else None
    active = time.monotonic()
    if lock:
        set_state(lock, "ready")
        said = False
        while starting(lock):
            if not said:
                print("Waiting for the other course runs to finish starting up.", flush=True)
                said = True
            rclpy.spin_once(nav, timeout_sec=0.5)
    quiet_from = time.monotonic()
    while time.time() - lags[0] < QUIET and time.monotonic() - quiet_from < QUIET_MAX:
        rclpy.spin_once(nav, timeout_sec=0.2)
    if lock:
        set_state(lock, "walking")
    runlog.event("goal", "release_band", waited=round(time.monotonic() - active, 1))

    # The goal only once the band is off and the robot has settled.
    while not released:
        rclpy.spin_once(nav, timeout_sec=0.2)
    while time.time() < released[0]["t"] + RELEASE_SETTLE:
        rclpy.spin_once(nav, timeout_sec=0.1)

    goal = PoseStamped()
    goal.header.frame_id = "odom"
    goal.header.stamp = nav.get_clock().now().to_msg()
    goal.pose.position.x, goal.pose.position.y = x, y
    goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    start = time.monotonic()
    runlog.event("goal", "goal_sent", x=x, y=y, yaw=round(math.degrees(yaw), 1))
    nav.goToPose(goal)
    recoveries, remaining, last_print, status = 0, None, 0.0, None
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            recoveries, remaining = feedback.number_of_recoveries, feedback.distance_remaining
            if time.monotonic() - last_print > 5:
                last_print = time.monotonic()
                p = feedback.current_pose.pose.position
                print(f"{time.monotonic() - start:5.0f} s  odom ({p.x:+.2f}, {p.y:+.2f})  "
                      f"remaining {remaining:.2f} m  recoveries {recoveries}", flush=True)
        if time.monotonic() - start > timeout:
            nav.cancelTask()
            status = "TIMEOUT"
            break
        time.sleep(0.1)
    if status is None:
        status = TaskResult(nav.getResult()).name
    seconds = round(time.monotonic() - start, 1)
    runlog.event("goal", "goal_result", status=status, recoveries=recoveries, seconds=seconds,
                 remaining=None if remaining is None else round(remaining, 2))
    print(f"Result: {status} after {seconds:.0f} s, {recoveries} recoveries", flush=True)
    time.sleep(SETTLE)
    nav.destroy_node()
    rclpy.try_shutdown()
    return 0


def main():
    try:
        return run()
    except KeyboardInterrupt:
        # The run ended (a fall, Ctrl-C) before Nav2 gave a result: nothing to report.
        print("Stopped before a result.", flush=True)
        return 0


if __name__ == "__main__":
    raise SystemExit(main())

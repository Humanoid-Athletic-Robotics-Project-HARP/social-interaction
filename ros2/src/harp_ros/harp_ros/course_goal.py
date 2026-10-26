"""Send one Nav2 goal for a course test and report how it went, then exit.
course.launch.py starts it and shuts the run down when it exits.

The goal is the course's goal site (from /sim/info), in odom, the frame
that starts where the policy started, so a goal at (7.5, 0) is 7.5 m ahead
of the start. Parameters:
  goal      "X Y" or "X Y YAW_DEG" to override the course's goal
  timeout   seconds to wait for Nav2 before cancelling (default 240)
Events on /harp/events: goal_sent (x, y, yaw) and goal_result (status,
recoveries, seconds, distance remaining).
"""

import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from harp_control import runlog

SETTLE = 3.0              # s to wait after the result, so the robot comes to rest on record


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

    info, policy_on = {}, []
    nav.create_subscription(String, "/sim/info", lambda m: info.update(json.loads(m.data)),
                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_event(msg):
        e = json.loads(msg.data)
        if e["source"] == "walker" and e["event"] == "policy_on":
            policy_on.append(e)
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
    # Not waitUntilNav2Active(): it waits for AMCL, which we don't run.
    nav._waitForNodeToActivate("bt_navigator")

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

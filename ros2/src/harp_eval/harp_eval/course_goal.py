"""Send a course goal or a finite waypoint route, report the result, then exit.
course.launch.py starts it and shuts the run down when it exits.

The goal is the course's goal site (from /sim/info), in odom, the frame
that starts where the policy started, so a goal at (7.5, 0) is 7.5 m ahead
of the start. Parameters:
  goal      "X Y" or "X Y YAW_DEG" to override the course's goal
  timeout   seconds to wait for Nav2 before cancelling (default 240)
  route     optional YAML route file; replaces the single goal
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
from nav2_msgs.action import NavigateToPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from harp_control import runlog
from harp_eval.course import set_state, starting
from harp_eval.waypoint_route import WaypointRoute, load_route

SETTLE = 3.0              # s to wait after the result, so the robot comes to rest on record
NAV2_START = 60.0         # s after the policy starts for Nav2 to be active (it takes about 5)
RELEASE_SETTLE = 2.0      # s after the band release before the goal goes out
QUIET = 5.0               # s without a sim lag event before asking for the release
QUIET_MAX = 60.0          # s to wait for that quiet before asking anyway
CANCEL_TIMEOUT = 5.0      # s to confirm a cancellation before ending the run


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


def send_navigation_goal(nav, goal, deadline):
    """Bound server discovery and acceptance as well as the navigation itself."""
    nav.feedback = None
    nav.result_future = None
    nav.goal_handle = None
    client = nav.nav_to_pose_client
    while not client.wait_for_server(timeout_sec=min(0.2, max(0.0, deadline - time.monotonic()))):
        if time.monotonic() >= deadline:
            return "TIMEOUT"
    if time.monotonic() >= deadline:
        return "TIMEOUT"
    request = NavigateToPose.Goal()
    request.pose = goal
    future = client.send_goal_async(request, lambda msg: setattr(nav, "feedback", msg.feedback))
    rclpy.spin_until_future_complete(nav, future, timeout_sec=max(0.0, deadline - time.monotonic()))
    if not future.done():
        return "TIMEOUT"
    nav.goal_handle = future.result()
    if nav.goal_handle is None or not nav.goal_handle.accepted:
        return "REJECTED"
    nav.result_future = nav.goal_handle.get_result_async()
    return "ACCEPTED"


def cancel_navigation(nav):
    """Bound both the cancellation response and its terminal result."""
    deadline = time.monotonic() + CANCEL_TIMEOUT
    try:
        future = nav.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(nav, future, timeout_sec=CANCEL_TIMEOUT)
        if not future.done() or future.result() is None:
            return False
        while not nav.isTaskComplete():
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.05)
        return True
    except Exception:
        return False


def observe_event(msg, policy_on, released, lags, control):
    """Read shared events without letting unrelated or malformed JSON end the course."""
    try:
        record = json.loads(msg.data)
    except (TypeError, ValueError):
        return
    if not isinstance(record, dict):
        return
    source, name = record.get("source"), record.get("event")
    if source == "walker":
        if name == "policy_on":
            policy_on.append(record)
        elif name == "manual":
            control["manual"] = True
        elif name == "hand_back":
            control["manual"] = False
        elif name in ("stop", "estop", "fall", "error"):
            control["stopped"] = True
    elif source == "sim":
        stamp = record.get("t")
        if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
            return
        if name == "band_released":
            released.append(record)
        elif name == "lag":
            lags[0] = max(lags[0], stamp)


def follow_route(nav, route, timeout, *, waypoint_events=False, control=None):
    """Advance only on success; one terminal goal event covers the whole run."""
    start = time.monotonic()
    final_x, final_y, final_yaw = route.waypoints[-1]
    details = {"x": final_x, "y": final_y, "yaw": final_yaw}
    if waypoint_events:
        details.update(waypoints=list(route.waypoints), laps=route.lap_limit)
    runlog.event("goal", "goal_sent", **details)
    control = {} if control is None else control
    completed = total_recoveries = 0
    total_paused = 0.0
    remaining, status, message = None, "SUCCEEDED", ""
    active = False
    try:
        while route.current() is not None:
            x, y, yaw_deg = route.current()
            lap, waypoint = route.completed_laps + 1, route.index + 1
            goal = PoseStamped()
            goal.header.frame_id = "odom"
            goal.header.stamp = nav.get_clock().now().to_msg()
            goal.pose.position.x, goal.pose.position.y = x, y
            yaw = math.radians(yaw_deg)
            goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            if waypoint_events:
                print(f"Lap {lap}/{route.lap_limit}, waypoint {waypoint}/{len(route.waypoints)}: "
                      f"({x:+.2f}, {y:+.2f}) m, {yaw_deg:+.0f} deg", flush=True)
                runlog.event("goal", "waypoint_sent", lap=lap, waypoint=waypoint,
                             x=x, y=y, yaw=yaw_deg)
            began = time.monotonic()
            recoveries, previous_recoveries, remaining, last_print = 0, 0, None, began
            paused, status = 0.0, None
            while status is None:
                if control.get("stopped"):
                    canceled = cancel_navigation(nav) if active else True
                    active = not canceled
                    status = "CANCELED" if canceled else "CANCEL_TIMEOUT"
                    break
                if control.get("manual"):
                    pause_start = time.monotonic()
                    canceled = cancel_navigation(nav) if active else True
                    active = not canceled
                    if not canceled:
                        status = "CANCEL_TIMEOUT"
                        break
                    previous_recoveries += recoveries
                    recoveries = 0
                    runlog.event("goal", "waypoint_paused", lap=lap, waypoint=waypoint,
                                 remaining_timeout=round(max(0.0, began + timeout + paused - pause_start), 2))
                    while control.get("manual") and not control.get("stopped"):
                        rclpy.spin_once(nav, timeout_sec=0.1)
                    duration = time.monotonic() - pause_start
                    paused += duration
                    total_paused += duration
                    if control.get("stopped"):
                        status = "CANCELED"
                        break
                    runlog.event("goal", "waypoint_resumed", lap=lap, waypoint=waypoint,
                                 paused_seconds=round(duration, 1))
                    # Retry the same route.current(); the canceled attempt never advances it.
                    remaining = None
                if not active:
                    # Simple Commander retains feedback/result state between requests.
                    submission = send_navigation_goal(nav, goal, began + timeout + paused)
                    status = None if submission == "ACCEPTED" else submission
                    active = submission == "ACCEPTED"
                    if not active:
                        break
                complete = nav.isTaskComplete()
                feedback = nav.getFeedback()
                if feedback:
                    recoveries = feedback.number_of_recoveries
                    remaining = feedback.distance_remaining
                # A takeover received during the action poll takes precedence over its result.
                if control.get("manual") or control.get("stopped"):
                    continue
                if complete:
                    active = False
                    status = TaskResult(nav.getResult()).name
                else:
                    if feedback:
                        if time.monotonic() - last_print > 5:
                            last_print = time.monotonic()
                            p = feedback.current_pose.pose.position
                            print(f"{time.monotonic() - start:5.0f} s  odom ({p.x:+.2f}, {p.y:+.2f})  "
                                  f"remaining {remaining:.2f} m  recoveries {recoveries}", flush=True)
                    if time.monotonic() - began - paused >= timeout:
                        canceled = cancel_navigation(nav)
                        active = not canceled
                        status = "TIMEOUT" if canceled else "CANCEL_TIMEOUT"
                        break
                    time.sleep(0.1)
            recoveries += previous_recoveries
            total_recoveries += recoveries
            if waypoint_events:
                runlog.event("goal", "waypoint_result", lap=lap, waypoint=waypoint,
                             x=x, y=y, yaw=yaw_deg, status=status,
                             seconds=round(time.monotonic() - began, 1), recoveries=recoveries)
            if status != "SUCCEEDED":
                break
            completed += 1
            route.advance()
    except KeyboardInterrupt:
        status = "CANCELED"
        if active:
            active = not cancel_navigation(nav)
            if active:
                status = "CANCEL_TIMEOUT"
    except Exception as exc:
        status, message = "ERROR", str(exc)
        if active:
            try:
                cancel_navigation(nav)
            except Exception:
                pass
    seconds = round(time.monotonic() - start, 1)
    result = {"status": status, "recoveries": total_recoveries, "seconds": seconds,
              "remaining": None if remaining is None else round(remaining, 2)}
    if waypoint_events:
        result.update(completed_waypoints=completed, completed_laps=route.completed_laps)
    if total_paused:
        result["paused_seconds"] = round(total_paused, 1)
    if message:
        result["message"] = message
    runlog.event("goal", "goal_result", **result)
    print(f"Result: {status} after {seconds:.0f} s, {total_recoveries} recoveries"
          + (f" ({message})" if message else ""), flush=True)
    return result


def run_session(nav):
    nav.declare_parameter("goal", "")
    nav.declare_parameter("timeout", 240.0)
    nav.declare_parameter("route", "")
    override = nav.get_parameter("goal").value.split()
    timeout = nav.get_parameter("timeout").value
    route_path = nav.get_parameter("route").value
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be a positive finite number")
    if route_path and override:
        raise ValueError("use either goal or route")
    route = load_route(route_path) if route_path else None
    latched = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    events_pub = nav.create_publisher(String, "/harp/events", latched)
    runlog.attach(lambda text: events_pub.publish(String(data=text)))

    info, policy_on, released, lags = {}, [], [], [0.0]
    control = {"manual": False, "stopped": False}
    nav.create_subscription(String, "/sim/info", lambda m: info.update(json.loads(m.data)),
                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    nav.create_subscription(String, "/harp/events",
                            lambda msg: observe_event(msg, policy_on, released, lags, control), latched)

    # The goal, then the walker running GR00T, then Nav2 up (it starts once the policy is on).
    while not (route or override or info) or not policy_on:
        rclpy.spin_once(nav, timeout_sec=0.2)
    if route is None:
        if override:
            if len(override) not in (2, 3):
                raise ValueError("goal takes X Y or X Y YAW_DEG")
            x, y = float(override[0]), float(override[1])
            yaw = float(override[2]) if len(override) > 2 else 0.0
        elif info.get("goal"):
            (x, y), yaw = info["goal"], 0.0
        else:
            nav.get_logger().error("No goal: the world has no goal site and no goal parameter was given.")
            return 1
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError("goal values must be finite")
        route = WaypointRoute([(x, y, yaw)], laps=1)
    # Not waitUntilNav2Active(): it waits for AMCL, which we don't run. Nor
    # _waitForNodeToActivate(): it waits forever if Nav2's bringup aborted.
    if not wait_for_active(nav, "bt_navigator", NAV2_START):
        runlog.event("goal", "goal_result", status="NAV2_NOT_ACTIVE", recoveries=0, seconds=0.0,
                     remaining=None)
        print(f"Result: NAV2_NOT_ACTIVE: bt_navigator wasn't active {NAV2_START:.0f} s after "
              "the policy started; see nav2.log", flush=True)
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

    result = follow_route(nav, route, timeout, waypoint_events=bool(route_path), control=control)
    if result["status"] == "SUCCEEDED":
        time.sleep(SETTLE)
    return 0 if result["status"] == "SUCCEEDED" else 1


def run():
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    nav = BasicNavigator(node_name="course_goal")
    try:
        return run_session(nav)
    finally:
        nav.destroy_node()
        rclpy.try_shutdown()


def main():
    try:
        return run()
    except KeyboardInterrupt:
        # The run ended (a fall, Ctrl-C) before Nav2 gave a result: nothing to report.
        print("Stopped before a result.", flush=True)
        return 130
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

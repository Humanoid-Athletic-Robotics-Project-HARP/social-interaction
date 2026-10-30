"""Wait until Nav2 is configured, up to the seconds given, then exit (1 if it wasn't).
entrypoint.sh runs it in a world's image before our node arms, so Nav2's start-up
(about ten processes joining DDS, and its costmaps' memory) is over before HIM times our
commands. Configured, not active: Nav2 activates only once our node's odometry arrives.
Prints a nav2_ready event (seconds waited, and whether Nav2 was ready) to policy.log.
"""

import json
import sys
import time

import rclpy

from harp.course_goal import wait_for_active


def main():
    started = time.monotonic()
    rclpy.init()
    node = rclpy.create_node("harp_nav2_ready")
    # bt_navigator is the last server Nav2's lifecycle manager configures.
    ready = wait_for_active(node, "bt_navigator", float(sys.argv[1]), ("inactive", "active"))
    print(json.dumps({"source": "entrypoint", "event": "nav2_ready", "ready": ready,
                      "seconds": round(time.monotonic() - started, 1)}), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ready else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env bash
# Our node, plus at most one thing driving /cmd_vel, each its own process (and the lidar
# relay on the robot):
#  - moves in G1_TASK_COMMAND ({"moves": [...]} or {"to": ...}): harp/moves.py;
#  - otherwise, in a world's image in the sim (submission/sim.sh --world): the
#    simulated lidar, Nav2 and the course goal (the world's goal, or the route in
#    G1_TASK_COMMAND, {"route": ...}: course.py --route).
# Everything prints to the container's output (HIM's policy.log in the sim).
set -e
NAV2_WAIT=15   # s our node waits at most for Nav2 to be configured (HIM waits 30 s)
source /opt/ros/foxy/setup.bash
source /example/install/setup.bash
source /harp_ws/install/setup.bash
# HIM's sim profile finds only 5 participants; ours lists peers for 32 (harp/fastdds_profile.py).
if [[ "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" == /fastdds.xml ]]; then
  python3 /harp/harp/fastdds_profile.py > /tmp/harp_fastdds.xml
  export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/harp_fastdds.xml
fi
# On the robot (dry-run and hardware): its head lidar into ROS 2 (harp/lidar_relay.py),
# through Unitree's SDK and its own CycloneDDS.
if [[ "${G1_EXECUTION_MODE:-sim}" != sim ]]; then
  LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib:$LD_LIBRARY_PATH" PYTHONPATH="/harp:$PYTHONPATH" \
    nice -n 10 python3 /harp/harp/lidar_relay.py &
fi
# No task, outside a world's image: the image's default moves (HARP_DEFAULT_TASK, the
# Dockerfile). HIM always passes G1_TASK_COMMAND, empty if unset, so the image can't set it.
# '[0,0,0]' stands still.
if [[ -z "${G1_TASK_COMMAND:-}" && ! -f /harp/model/world.xml ]]; then
  export G1_TASK_COMMAND="${HARP_DEFAULT_TASK:-}"
fi
if [[ "${G1_TASK_COMMAND:-}" == *'"to"'* || "${G1_TASK_COMMAND:-}" == *'"moves"'* ]]; then
  PYTHONPATH="/harp:$PYTHONPATH" python3 /harp/harp/moves.py &
elif [[ -f /harp/model/world.xml && "${G1_EXECUTION_MODE:-sim}" == sim ]]; then
  # Before our node arms, and at lower priority: starting about ten processes beside it
  # (DDS discovery, and about a GB more memory on a full host) made our joint commands
  # answer states over 0.2 s old, and HIM stopped the run (observation_timeout,
  # checked_command_expired). HIM times nothing before our first command, and waits 30 s
  # for it, so our node waits up to NAV2_WAIT s for Nav2 to be configured.
  # ROS's log folder, made before its processes start together: two making it at once
  # can fail one ("Failed to create log directory").
  mkdir -p "${ROS_LOG_DIR:-${ROS_HOME:-$HOME/.ros}/log}"
  PYTHONPATH="/harp:$PYTHONPATH" nice -n 10 python3 /harp/harp/lidar.py &
  nice -n 10 ros2 launch /harp/nav2/nav2.launch.py &
  PYTHONPATH="/harp:$PYTHONPATH" nice -n 10 python3 /harp/harp/course_goal.py &
  PYTHONPATH="/harp:$PYTHONPATH" python3 /harp/harp/nav2_ready.py "$NAV2_WAIT" || true
fi
exec python3 /harp/harp_node.py

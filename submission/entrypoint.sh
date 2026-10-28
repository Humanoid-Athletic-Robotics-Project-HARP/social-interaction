#!/usr/bin/env bash
# Our node, plus at most one thing driving /cmd_vel, each its own process:
#  - a {"to": ...} target in G1_TASK_COMMAND: walk there (harp/move_to_point.py);
#  - otherwise, in a world's image in the sim (submission/sim.sh --world): the
#    simulated lidar, Nav2 and the course goal.
# Everything prints to the container's output (HIM's policy.log in the sim).
set -e
source /opt/ros/foxy/setup.bash
source /example/install/setup.bash
# HIM's sim profile finds only 5 participants; ours lists peers for 32 (harp/fastdds_profile.py).
if [[ "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" == /fastdds.xml ]]; then
  python3 /harp/harp/fastdds_profile.py > /tmp/harp_fastdds.xml
  export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/harp_fastdds.xml
fi
if [[ "${G1_TASK_COMMAND:-}" == *'"to"'* ]]; then
  PYTHONPATH="/harp:$PYTHONPATH" python3 /harp/harp/move_to_point.py &
elif [[ -f /harp/model/world.xml && "${G1_EXECUTION_MODE:-sim}" == sim ]]; then
  PYTHONPATH="/harp:$PYTHONPATH" python3 /harp/harp/lidar.py &
  ros2 launch /harp/nav2/nav2.launch.py &
  PYTHONPATH="/harp:$PYTHONPATH" python3 /harp/harp/course_goal.py &
fi
exec python3 /harp/harp_node.py

#!/usr/bin/env bash
# In a world's image (submission/sim.sh --world): the simulated lidar, Nav2 and the
# course goal beside our node, each its own process.
# Everything prints to the container's output, which HIM's sim keeps in policy.log.
set -e
source /opt/ros/foxy/setup.bash
source /example/install/setup.bash
# HIM's sim profile finds only 5 participants; ours lists peers for 32 (harp/fastdds_profile.py).
if [[ "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" == /fastdds.xml ]]; then
  python3 /harp/harp/fastdds_profile.py > /tmp/harp_fastdds.xml
  export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/harp_fastdds.xml
fi
if [[ -f /harp/model/world.xml && "${G1_EXECUTION_MODE:-sim}" == sim ]]; then
  PYTHONPATH="/harp:$PYTHONPATH" python3 /harp/harp/lidar.py &
  ros2 launch /harp/nav2/nav2.launch.py &
  PYTHONPATH="/harp:$PYTHONPATH" python3 /harp/harp/course_goal.py &
fi
# A {"to": ...} target in G1_TASK_COMMAND: walk there (harp/move_to_point.py).
if [[ "${G1_TASK_COMMAND:-}" == *'"to"'* ]]; then
  PYTHONPATH="/harp:$PYTHONPATH" python3 /harp/harp/move_to_point.py &
fi
exec python3 /harp/harp_node.py

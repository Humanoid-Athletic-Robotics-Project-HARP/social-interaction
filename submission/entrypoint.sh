#!/usr/bin/env bash
# Our node, plus at most one thing driving /cmd_vel, each its own process (and the lidar
# relay on the robot):
#  - moves in G1_TASK_COMMAND ({"moves": [...]} or {"to": ...}): harp/moves.py;
#  - otherwise, in a world's image in the sim (submission/sim.sh --world), or on the robot
#    for a "robot": true task with a route (harp/avoidance_config.py): the simulated lidar
#    (in the sim; on the robot the lidar relay), Nav2, the course goal (the world's goal, or
#    the route in G1_TASK_COMMAND, {"route": ...}: course.py --route), and harp/approach_stop.py
#    between Nav2's commands and our node (standing while something comes at the robot),
#    and harp/view_feed.py (the lidar, pose, people and costmap for submission/lidar_view.py);
#    with "mapping" or "map" in G1_TASK_COMMAND, harp/mapper.py (a map for Nav2);
#    with {"behavior": ...} in G1_TASK_COMMAND, the behavior loop (harp/behavior.py: wander
#    and walk up to people) and its stand-in person detector (harp/sim_people.py) in place
#    of the course goal.
# Everything prints to the container's output (HIM's policy.log in the sim).
set -e
NAV2_WAIT=15   # s our node waits at most for Nav2 to be configured (HIM waits 30 s)
NAV2_TRIES=3   # times Nav2 is started at most
source /opt/ros/foxy/setup.bash
source /example/install/setup.bash
source /harp_ws/install/setup.bash
# Validate before starting any command producer. Select one filter, never two.
# It prints the filter, then "robot" if Nav2 runs on the robot.
selection=$(PYTHONPATH="/harp:$PYTHONPATH" python3 -m harp.avoidance_config)
avoidance_filter=${selection%% *}
export HARP_AVOIDANCE_FILTER="$avoidance_filter"   # nav2.launch.py follows it
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
elif [[ ( -f /harp/model/world.xml && "${G1_EXECUTION_MODE:-sim}" == sim ) || "$selection" == *" robot" ]]; then
  # Before our node arms, and at lower priority: starting about ten processes beside it
  # (DDS discovery, and about a GB more memory on a full host) made our joint commands
  # answer states over 0.2 s old, and HIM stopped the run (observation_timeout,
  # checked_command_expired). HIM times nothing before our first command, and waits 30 s
  # for it, so our node waits up to NAV2_WAIT s for Nav2 to be configured.
  # ROS's log folder, made before its processes start together: two making it at once
  # can fail one ("Failed to create log directory").
  mkdir -p "${ROS_LOG_DIR:-${ROS_HOME:-$HOME/.ros}/log}"
  if [[ "${G1_EXECUTION_MODE:-sim}" == sim ]]; then   # on the robot the relay (above) is the lidar
    PYTHONPATH="/harp:$PYTHONPATH" nice -n 10 python3 /harp/harp/lidar.py &
  fi
  # A map to build or use ("mapping" or "map" in the task): harp/mapper.py, before Nav2,
  # whose costmaps then wait for its map and its map -> odom transform.
  if [[ "${G1_TASK_COMMAND:-}" == *'"mapping"'* || "${G1_TASK_COMMAND:-}" == *'"map"'* ]]; then
    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH="/harp:$PYTHONPATH" \
      nice -n 10 python3 /harp/harp/mapper.py &
  fi
  # A Nav2 server that dies ends the launch (nav2.launch.py): start it again, NAV2_TRIES times
  # in all (Foxy's servers sometimes crash as they start).
  ( for try in $(seq "$NAV2_TRIES"); do
      nice -n 10 ros2 launch /harp/nav2/nav2.launch.py
      printf '{"source": "entrypoint", "event": "nav2_stopped", "wall": %s, "try": %s}\n' "$(date +%s.%N)" "$try"
    done ) &
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH="/harp:$PYTHONPATH" \
    nice -n 10 python3 "/harp/harp/${avoidance_filter}.py" &
  if [[ "${G1_TASK_COMMAND:-}" == *'"behavior"'* ]]; then
    PYTHONPATH="/harp:$PYTHONPATH" nice -n 10 python3 /harp/harp/sim_people.py &
    PYTHONPATH="/harp:$PYTHONPATH" nice -n 10 python3 /harp/harp/behavior.py &
  else
    PYTHONPATH="/harp:$PYTHONPATH" nice -n 10 python3 /harp/harp/course_goal.py &
  fi
  # What the robot sees, a few times a second on our output, for submission/lidar_view.py.
  PYTHONPATH="/harp:$PYTHONPATH" nice -n 19 python3 /harp/harp/view_feed.py &
  PYTHONPATH="/harp:$PYTHONPATH" python3 /harp/harp/nav2_ready.py "$NAV2_WAIT" || true
fi
exec python3 /harp/harp_node.py

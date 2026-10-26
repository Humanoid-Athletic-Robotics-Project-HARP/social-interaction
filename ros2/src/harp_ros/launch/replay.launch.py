"""Play a recorded run back into Nav2, to try Nav2 settings on the same
recorded lidar, /tf and /odom without the robot:
  ros2 launch harp_ros replay.launch.py run:=2026-10-27_10-00-00_robot_hall_tuning
  ros2 launch harp_ros replay.launch.py run:=RUN params_file:=/path/to/other_params.yaml record:=true tag:=inflation1.0

The run must have been recorded with a replay recording (course --replay
--replay-reason ..., or run_recorder replay:=true replay_reason:=...). run is
a run folder, or a unique part of its name under runs/; bag is a replay
folder, for one kept elsewhere (replay_dir).

The recording plays with simulated time (ros2 bag play --clock), into
nav2.launch.py with use_sim_time and params_file, which starts 4 s in, once
the recorded /tf is there. Nothing walks: Nav2 builds
its costmaps from the recorded lidar, and plans if given a goal in rviz2.
The recorded /cmd_vel is played as /recorded/cmd_vel, apart from Nav2's own.
record:=true also records the replay as a run (kind replay, with Nav2's
costmaps in its bag), so settings can be compared afterwards with runs.
The replay ends when the recording does.
"""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
                            TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from harp_control import runlog

NAV2_DELAY = 4.0          # s after the recording starts playing


def find_bag(run, bag):
    if bag:
        return Path(bag)
    path = Path(run)
    if not path.exists():
        matches = [p for p in runlog.runs_dir().iterdir() if run in p.name]
        if len(matches) != 1:
            raise RuntimeError(f"{len(matches)} runs match {run!r}; give more of the name")
        path = matches[0]
    meta = yaml.safe_load((path / "run.yaml").read_text())
    if not meta.get("replay"):
        raise RuntimeError(f"{path.name} has no replay recording (it wasn't recorded with --replay)")
    return Path(meta["replay"]["path"])


def actions(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    bag = find_bag(get("run"), get("bag"))
    share = get_package_share_directory("harp_ros")
    play = ExecuteProcess(cmd=["ros2", "bag", "play", str(bag), "--clock", "--rate", get("rate"),
                               "--remap", "/cmd_vel:=/recorded/cmd_vel"],
                          name="bag_play", output="both")
    # Nav2 gives up starting if odom -> base_link doesn't exist within a few seconds,
    # so it starts once the recording is playing (the recording's first seconds are lost).
    nav2 = TimerAction(period=NAV2_DELAY, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "nav2.launch.py")),
        launch_arguments={"params_file": get("params_file"), "use_sim_time": "true",
                          "rviz": get("rviz")}.items())])
    out = [play, nav2, RegisterEventHandler(OnProcessExit(target_action=play, on_exit=[
        EmitEvent(event=Shutdown(reason="the recording ended"))]))]
    if get("record") == "true":
        out.append(Node(package="harp_ros", executable="run_recorder", name="run_recorder",
                        output="both", sigterm_timeout="15", parameters=[{
                            "kind": "replay", "tag": get("tag"), "extras": "nav",
                            "extras_reason": "compare Nav2 settings on a replayed run",
                            "nav2_params_file": get("params_file"),
                            "note": f"replay of {bag} with {get('params_file')}"}]))
    return out


def generate_launch_description():
    share = get_package_share_directory("harp_ros")
    return LaunchDescription([
        DeclareLaunchArgument("run", default_value="", description="run folder or unique part of its name"),
        DeclareLaunchArgument("bag", default_value="", description="a replay folder, instead of run"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "nav2_params.yaml")),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("record", default_value="false"),
        DeclareLaunchArgument("tag", default_value="replay"),
        OpaqueFunction(function=actions),
    ])

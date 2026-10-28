"""One recorded course test in the sim: the sim (MuJoCo window) with the head
lidar and a course, walker_node, Nav2 (rviz2 window) and the course's goal,
all recorded by run_recorder. Start it with the course command, which makes
the run folder first and can repeat it:
  ros2 run harp_eval course slalom --tag rangecut --repeat 3

Order: the recorder, sim, walker_node, lidar_relay and course_goal start
together. The sim publishes its lidar as the robot does (--lidar-dds) and
lidar_relay brings it into ROS 2, so the run uses the robot's lidar path.
The sim lowers the robot after 15 s (time for walker_node to reach its stand pose);
walker_node starts the policy by itself on the sim's "feet down" (auto_start);
Nav2 starts once walker_node prints "Policy on". The band stays on until
course_goal asks for its release (Nav2 active, and other course runs ready:
see course.py), then course_goal sends the goal. The run ends when course_goal finishes (result or
timeout) or walker_node stops (a fall), and everything shuts down.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

AUTO_RELEASE = "15"       # s of sim time before the sim lowers the robot onto its feet
# String launch arguments passed straight to run_recorder's parameters.
RECORDER_ARGS = ["run_dir", "world", "tag", "note", "extras", "extras_reason", "command",
                 "replay_reason", "replay_dir", "lidar_fields", "floor_surface", "tether",
                 "battery_percent", "people_nearby", "lighting", "nav2_params_file"]


def generate_launch_description():
    arg = LaunchConfiguration
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("harp_ros"), "launch", "nav2.launch.py")),
        launch_arguments={"rviz": arg("rviz"), "params_file": arg("nav2_params_file")}.items())
    recorder = Node(package="harp_eval", executable="run_recorder", name="run_recorder",
                    output="both", sigterm_timeout="15", parameters=[{
                        # As text, or "80" (battery_percent) would arrive as a number.
                        **{name: ParameterValue(arg(name), value_type=str) for name in RECORDER_ARGS},
                        "replay": PythonExpression(["'", arg("replay"), "' == 'true'"]),
                        "lidar_rate": PythonExpression(["float('", arg("lidar_rate"), "' or 0)"]),
                        "summarize": False}])
    sim = Node(package="harp_sim", executable="run_sim", output="both",
               arguments=["--lidar-dds", "--world", arg("world"), "--auto-release", AUTO_RELEASE, "--hold-band", "--log",
                          "--domain", arg("sim_domain")])
    walker = Node(package="harp_ros", executable="walker_node", name="walker", output="both",
                  parameters=[{"auto_start": True,
                               "domain": ParameterValue(arg("sim_domain"), value_type=int)}])
    relay = Node(package="harp_ros", executable="lidar_relay", name="lidar_relay", output="both",
                 parameters=[{"domain": ParameterValue(arg("sim_domain"), value_type=int)}])
    goal = Node(package="harp_eval", executable="course_goal", name="course_goal", output="both",
                parameters=[{"goal": arg("goal"), "timeout": arg("timeout"),
                             "route": arg("route")}])

    started = []

    def start_nav2(event):
        if not started and b"Policy on" in event.text:
            started.append(True)
            return [nav2]

    return LaunchDescription([
        DeclareLaunchArgument("world", description="a course in harp_sim/worlds, e.g. slalom"),
        DeclareLaunchArgument("run_dir", default_value="", description="run folder (course makes it)"),
        DeclareLaunchArgument("tag", default_value=""),
        DeclareLaunchArgument("note", default_value=""),
        DeclareLaunchArgument("extras", default_value="", description="e.g. nav,lidar (logging.yaml)"),
        DeclareLaunchArgument("extras_reason", default_value=""),
        DeclareLaunchArgument("command", default_value=""),
        DeclareLaunchArgument("goal", default_value="", description='"X Y [YAW_DEG]" in odom'),
        DeclareLaunchArgument("route", default_value="", description="optional waypoint YAML file"),
        DeclareLaunchArgument("timeout", default_value="240.0"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("sim_domain", default_value="1",
                              description="DDS domain of the sim's robot topics (course picks one)"),
        DeclareLaunchArgument("nav2_params_file", description="Nav2 parameters", default_value=os.path.join(
            get_package_share_directory("harp_ros"), "config", "nav2_params.yaml")),
        DeclareLaunchArgument("replay", default_value="false",
                              description="record for replay (only when a person asks, with replay_reason)"),
        DeclareLaunchArgument("lidar_rate", default_value=""),
        *[DeclareLaunchArgument(name, default_value="") for name in
          ("replay_reason", "replay_dir", "lidar_fields", "floor_surface", "tether",
           "battery_percent", "people_nearby", "lighting")],
        SetEnvironmentVariable("PYTHONUNBUFFERED", "1"),
        recorder, sim, walker, relay, goal,
        RegisterEventHandler(OnProcessIO(target_action=walker, on_stdout=start_nav2)),
        RegisterEventHandler(OnProcessExit(target_action=goal, on_exit=[EmitEvent(event=Shutdown(
            reason="course_goal finished"))])),
        RegisterEventHandler(OnProcessExit(target_action=walker, on_exit=[EmitEvent(event=Shutdown(
            reason="walker_node stopped"))])),
        # Without the recorder (it refuses a replay without room on the disk) the run is pointless.
        RegisterEventHandler(OnProcessExit(target_action=recorder, on_exit=[EmitEvent(event=Shutdown(
            reason="run_recorder stopped"))])),
    ])

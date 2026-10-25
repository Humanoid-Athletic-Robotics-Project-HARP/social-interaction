"""Start every HARP node with one config file.

  ros2 launch harp harp.launch.py              # config/sim.yaml (default)
  ros2 launch harp harp.launch.py config:=real # config/real.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NODES = ["robot_bridge", "perception", "voice", "behavior"]


def make_nodes(context):
    config_name = LaunchConfiguration("config").perform(context)
    params = os.path.join(get_package_share_directory("harp"), "config", config_name + ".yaml")
    return [
        Node(package="harp", executable=name, name=name, parameters=[params], output="screen")
        for name in NODES
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="sim", description="sim or real"),
        OpaqueFunction(function=make_nodes),
    ])

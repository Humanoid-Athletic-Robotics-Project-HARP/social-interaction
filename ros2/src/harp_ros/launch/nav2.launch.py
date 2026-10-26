"""Nav2 for the G1, plus rviz2 to watch it and give it goals.

Start it after walker_node's policy is running (it needs /odom and the
odom -> base_link transform), with the sim's lidar or the real one on
/utlidar/cloud_livox_mid360:
  ros2 launch harp_ros nav2.launch.py
  ros2 launch harp_ros nav2.launch.py rviz:=false

Only the Nav2 servers we use, each as its own node (nav2_bringup's launch
files also start docking_server, which RoboStack doesn't have): the planner
(a path on the costmap), the controller (Graceful, /cmd_vel to walker_node),
the behaviors (turn, wait), and the navigator (behavior_trees/navigate.xml),
brought up by one lifecycle manager. In rviz2, "2D Goal Pose" sends a goal.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SERVERS = [
    ("nav2_controller", "controller_server"),
    ("nav2_planner", "planner_server"),
    ("nav2_behaviors", "behavior_server"),
    ("nav2_bt_navigator", "bt_navigator"),
]


def generate_launch_description():
    share = get_package_share_directory("harp_ros")
    params = os.path.join(share, "config", "nav2_params.yaml")
    tree = os.path.join(share, "behavior_trees", "navigate.xml")
    nodes = [
        Node(package=package, executable=name, name=name, output="screen",
             parameters=[params, {"default_nav_to_pose_bt_xml": tree}]
             if name == "bt_navigator" else [params])
        for package, name in SERVERS
    ]
    nodes.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                      name="lifecycle_manager_navigation", output="screen", parameters=[params]))
    nodes.append(Node(package="rviz2", executable="rviz2", output="screen",
                      arguments=["-d", os.path.join(share, "rviz", "nav2.rviz")],
                      condition=IfCondition(LaunchConfiguration("rviz"))))
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="open rviz2"),
        *nodes,
    ])

"""Nav2 for the G1, inside our image (Foxy), started by entrypoint.sh.

It needs /odom, the odom -> base_link transform (harp_node) and a lidar on
/utlidar/cloud_livox_mid360. Only the Nav2 servers we use, each as its own
node: the planner (a path on the costmap), the controller (/cmd_vel to
harp_node), the recoveries (turn, wait) and the navigator (navigate.xml),
brought up by one lifecycle manager.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))
PARAMS = os.path.join(HERE, "nav2_params.yaml")
TREE = os.path.join(HERE, "navigate.xml")
SERVERS = [
    ("nav2_controller", "controller_server"),
    ("nav2_planner", "planner_server"),
    ("nav2_recoveries", "recoveries_server"),
    ("nav2_bt_navigator", "bt_navigator"),
]


def generate_launch_description():
    nodes = [
        Node(package=package, executable=name, name=name, output="both",
             parameters=[PARAMS, {"default_bt_xml_filename": TREE}]
             if name == "bt_navigator" else [PARAMS])
        for package, name in SERVERS
    ]
    nodes.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                      name="lifecycle_manager_navigation", output="both", parameters=[PARAMS]))
    return LaunchDescription(nodes)

"""Nav2 for the G1, inside our image (Foxy), started by entrypoint.sh.

It needs /odom, the odom -> base_link transform (harp_node) and a lidar on
/utlidar/cloud_livox_mid360. Only the Nav2 servers we use, each as its own
node: the planner (a path on the costmap), the controller (/cmd_vel_nav, which
harp/approach_stop.py passes on to harp_node's /cmd_vel unless something comes at
the robot), the recoveries (wait) and the navigator (navigate.xml), brought up by
one lifecycle manager. With the face visor on (harp/lidar_mount.py), the costmaps
leave out the lidar's clearing view ahead (nav2_params.yaml).
"""

import os
import sys
import tempfile

from launch import LaunchDescription
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from harp import lidar_mount  # noqa: E402

PARAMS = os.path.join(HERE, "nav2_params.yaml")
TREE = os.path.join(HERE, "navigate.xml")
DRIVES = {"controller_server", "recoveries_server"}   # publish cmd_vel
SERVERS = [
    ("nav2_controller", "controller_server"),
    ("nav2_planner", "planner_server"),
    ("nav2_recoveries", "recoveries_server"),
    ("nav2_bt_navigator", "bt_navigator"),
]


def params():
    """nav2_params.yaml, or a copy without lidar_front_clearing with the visor on."""
    if not lidar_mount.VISOR:
        return PARAMS
    with open(PARAMS) as f:
        text = f.read()
    sources = " lidar_clearing lidar_front_clearing camera_marking"
    assert text.count(sources) == 2, "nav2_params.yaml: lidar_front_clearing not in both costmaps"
    path = os.path.join(tempfile.mkdtemp(), "nav2_params.yaml")
    with open(path, "w") as f:
        f.write(text.replace(sources, " lidar_clearing camera_marking"))
    return path


def generate_launch_description():
    params_file = params()
    nodes = [
        Node(package=package, executable=name, name=name, output="both",
             parameters=[params_file, {"default_bt_xml_filename": TREE}]
             if name == "bt_navigator" else [params_file],
             remappings=[("cmd_vel", "cmd_vel_nav")] if name in DRIVES else [])
        for package, name in SERVERS
    ]
    nodes.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                      name="lifecycle_manager_navigation", output="both", parameters=[params_file]))
    return LaunchDescription(nodes)

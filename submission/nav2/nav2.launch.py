"""Nav2 for the G1, inside our image (Foxy), started by entrypoint.sh.

It needs /odom, the odom -> base_link transform (harp_node) and a lidar on
/utlidar/cloud_livox_mid360. Only the Nav2 servers we use, each as its own
node: the planner (a path on the costmap), the controller (/cmd_vel_nav, which
harp/approach_stop.py passes on to harp_node's /cmd_vel unless something comes at
the robot), the recoveries (wait) and the navigator (navigate.xml), brought up by
one lifecycle manager.

With predictive avoidance (HARP_AVOIDANCE_FILTER, from entrypoint.sh), that filter owns
moving objects, and Nav2 runs on a copy of nav2_params.yaml (parameters_for) where:
  - both costmaps mark obstacles from the filter's STATIC_TOPIC, the lidar without the
    returns of objects seen moving, so a hallway full of walkers is not a wall to the
    planner (it failed to find any path and the route was given up); clearing still
    uses the raw lidar;
  - the controller keeps sending the path's command when something blocks its
    trajectory (FollowPath.follow_when_blocked) instead of stopping and, after
    failure_tolerance, aborting the path.
A rewritten file rather than launch overrides: Foxy gives the file's own node entries
priority over a launch override (written for every node, /**), whatever the order.
"""

import os
import tempfile

import yaml

from launch import LaunchDescription
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))
PARAMS = os.path.join(HERE, "nav2_params.yaml")
TREE = os.path.join(HERE, "navigate.xml")
DRIVES = {"controller_server", "recoveries_server"}   # publish cmd_vel
OWNS_MOVING = {"predictive_avoidance"}   # filters that check every command against moving objects
STATIC_TOPIC = "/harp/static_cloud"      # harp/predictive_avoidance.py's STATIC_TOPIC
SERVERS = [
    ("nav2_controller", "controller_server"),
    ("nav2_planner", "planner_server"),
    ("nav2_recoveries", "recoveries_server"),
    ("nav2_bt_navigator", "bt_navigator"),
]


def predictive_parameters():
    """nav2_params.yaml, as a dict, changed for a filter that owns moving objects."""
    with open(PARAMS) as source:
        params = yaml.safe_load(source)
    params["controller_server"]["ros__parameters"]["FollowPath"]["follow_when_blocked"] = True
    for costmap in ("local_costmap", "global_costmap"):
        layer = params[costmap][costmap]["ros__parameters"]["stvl_layer"]
        layer["lidar_marking"]["topic"] = STATIC_TOPIC
    return params


def parameters_for(avoidance_filter=None):
    """The parameter file Nav2 runs with for this filter."""
    if avoidance_filter not in OWNS_MOVING:
        return PARAMS
    path = os.path.join(tempfile.gettempdir(), "harp_nav2_params_predictive.yaml")
    with open(path, "w") as out:
        yaml.safe_dump(predictive_parameters(), out, sort_keys=False)
    return path


def generate_launch_description():
    params = parameters_for(os.environ.get("HARP_AVOIDANCE_FILTER"))
    nodes = [
        Node(package=package, executable=name, name=name, output="both",
             parameters=[params, {"default_bt_xml_filename": TREE}]
             if name == "bt_navigator" else [params],
             remappings=[("cmd_vel", "cmd_vel_nav")] if name in DRIVES else [])
        for package, name in SERVERS
    ]
    nodes.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                      name="lifecycle_manager_navigation", output="both", parameters=[params]))
    return LaunchDescription(nodes)

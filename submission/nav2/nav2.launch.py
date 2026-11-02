"""Nav2 for the G1, inside our image (Foxy), started by entrypoint.sh.

It needs /odom, the odom -> base_link transform (harp_node) and a lidar on
/utlidar/cloud_livox_mid360. Only the Nav2 servers we use, each as its own
node: the planner (a path on the costmap), the controller (/cmd_vel_nav, which
harp/approach_stop.py passes on to harp_node's /cmd_vel unless something comes at
the robot), the recoveries (wait) and the navigator (navigate.xml), brought up by
one lifecycle manager.

Nav2 runs on nav2_params.yaml, or on a rewritten copy (parameters_for) when:
  - the face visor is on (harp/lidar_mount.py): the costmaps leave out the lidar's
    clearing view ahead (lidar_front_clearing);
  - predictive avoidance (HARP_AVOIDANCE_FILTER, from entrypoint.sh) owns moving objects:
      - both costmaps mark obstacles from the filter's STATIC_TOPIC, the lidar without the
        returns of objects seen moving, so a hallway full of walkers is not a wall to the
        planner (it failed to find any path and the route was given up), and the head
        camera only clears (its marking would put the walkers back); clearing still uses
        the raw lidar;
      - the controller keeps sending the path's command when something blocks its
        trajectory (FollowPath.follow_when_blocked) instead of stopping and, after
        failure_tolerance, aborting the path.
A rewritten file rather than launch overrides: Foxy gives the file's own node entries
priority over a launch override (written for every node, /**), whatever the order.
"""

import os
import sys
import tempfile

import yaml

from launch import LaunchDescription
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from harp import lidar_mount  # noqa: E402

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


def rewritten_parameters(visor, predictive):
    """nav2_params.yaml, as a dict, changed for the visor and for a filter that owns
    moving objects."""
    with open(PARAMS) as source:
        params = yaml.safe_load(source)
    if predictive:
        params["controller_server"]["ros__parameters"]["FollowPath"]["follow_when_blocked"] = True
    for costmap in ("local_costmap", "global_costmap"):
        layer = params[costmap][costmap]["ros__parameters"]["stvl_layer"]
        sources = layer["observation_sources"].split()
        assert "lidar_front_clearing" in sources and "camera_marking" in sources, \
            f"nav2_params.yaml: {costmap} sources changed"
        if visor:
            sources.remove("lidar_front_clearing")
        if predictive:
            layer["lidar_marking"]["topic"] = STATIC_TOPIC
            sources.remove("camera_marking")
        layer["observation_sources"] = " ".join(sources)
    return params


def parameters_for(avoidance_filter=None, visor=None):
    """The parameter file Nav2 runs with for this filter and visor setting."""
    visor = lidar_mount.VISOR if visor is None else visor
    predictive = avoidance_filter in OWNS_MOVING
    if not (visor or predictive):
        return PARAMS
    path = os.path.join(tempfile.mkdtemp(), "nav2_params.yaml")
    with open(path, "w") as out:
        yaml.safe_dump(rewritten_parameters(visor, predictive), out, sort_keys=False)
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

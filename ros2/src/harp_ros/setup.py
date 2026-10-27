from glob import glob

from setuptools import find_packages, setup

package_name = "harp_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/behavior_trees", glob("behavior_trees/*.xml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="HARP Human Interaction team",
    maintainer_email="todo@todo.todo",
    description="ROS 2 for the G1: walker_node (/cmd_vel in, /odom and /tf out), lidar_relay (the robot's lidar into our ROS 2 domain) and Nav2 (nav2.launch.py).",
    license="TODO",
    entry_points={
        "console_scripts": [
            "walker_node = harp_ros.walker_node:main",
            "lidar_relay = harp_ros.lidar_relay:main",
        ],
    },
)

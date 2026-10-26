from setuptools import find_packages, setup

package_name = "harp_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="HARP Human Interaction team",
    maintainer_email="todo@todo.todo",
    description="ROS 2 nodes for the G1: walker_node (/cmd_vel in, /odom and /tf out).",
    license="TODO",
    entry_points={
        "console_scripts": [
            "walker_node = harp_ros.walker_node:main",
        ],
    },
)

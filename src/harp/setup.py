from glob import glob

from setuptools import setup

package_name = "harp"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HARP Human Interaction team",
    maintainer_email="",
    description="Social-interaction nodes for the Unitree G1 EDU.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "robot_bridge = harp.robot_bridge:main",
            "perception = harp.perception:main",
            "voice = harp.voice:main",
            "behavior = harp.behavior:main",
        ],
    },
)

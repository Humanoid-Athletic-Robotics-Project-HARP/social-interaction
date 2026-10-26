from glob import glob

from setuptools import find_packages, setup

package_name = "harp_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/rviz", ["rviz/lidar.rviz"]),
        ("share/" + package_name + "/worlds", glob("worlds/*.xml") + glob("worlds/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="HARP Human Interaction team",
    maintainer_email="todo@todo.todo",
    description="MuJoCo sim of the 29-joint G1 that speaks the robot's low-level DDS topics.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "run_sim = harp_sim.run_sim:main",
        ],
    },
)

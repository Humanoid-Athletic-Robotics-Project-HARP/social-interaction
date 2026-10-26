from setuptools import find_packages, setup

package_name = "harp_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    # The GR00T policies and their license ship inside the package.
    package_data={package_name: ["gr00t/policy/*", "gr00t/LICENSE-GR00T-WholeBodyControl.txt"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="HARP Human Interaction team",
    maintainer_email="todo@todo.todo",
    description="G1 walking without ROS: GR00T policies, leg odometry, smooth control, move_distance.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "walk = harp_control.gr00t.walk:main",
            "move_distance = harp_control.move_distance:main",
        ],
    },
)

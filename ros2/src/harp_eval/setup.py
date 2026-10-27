from glob import glob

from setuptools import find_packages, setup

package_name = "harp_eval"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="HARP Human Interaction team",
    maintainer_email="todo@todo.todo",
    description="Course tests and run logging for the G1: course, course_goal, run_recorder, runs, and Nav2 replay (replay.launch.py).",
    license="TODO",
    entry_points={
        "console_scripts": [
            "run_recorder = harp_eval.run_recorder:main",
            "course = harp_eval.course:main",
            "course_goal = harp_eval.course_goal:main",
            "runs = harp_eval.runs:main",
        ],
    },
)

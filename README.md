# HARP Human Interaction

Social interaction on the Unitree G1 EDU. This is a skeleton: every node is a
stub with fake data, and nothing is sent to the robot yet.

## Layout

```
src/
├── harp_interfaces/   message definitions (Utterance, Person, PersonArray)
└── harp/              all nodes, one Python package
    ├── harp/
    │   ├── robot_bridge.py  the ONLY node that talks to the robot (dry run by default)
    │   ├── perception.py    lidar/camera -> people          (fake for now)
    │   ├── voice.py         mic -> speech-to-text           (fake for now)
    │   └── behavior.py      decides what to say and do      (echo + face person)
    ├── launch/harp.launch.py
    └── config/sim.yaml, real.yaml
scripts/deploy.sh      rsync the repo to the Jetson
dependencies.repos     Unitree code to fetch later (not needed yet)
context/               project notes and memory
```

## Topics

| Topic | Type | From → To |
|---|---|---|
| `/harp/heard` | `harp_interfaces/Utterance` | voice → behavior |
| `/harp/people` | `harp_interfaces/PersonArray` | perception → behavior |
| `/harp/say` | `std_msgs/String` | behavior → robot_bridge |
| `/harp/cmd_vel` | `geometry_msgs/Twist` | behavior → robot_bridge |

## Build and run

Needs ROS 2 (Foxy on the Jetson, or Humble). From the repo root:

```bash
colcon build
source install/setup.bash
ros2 launch harp harp.launch.py              # sim config
ros2 launch harp harp.launch.py config:=real # on the robot
```

You should see fake speech echoed back ("You said: ...") and dry-run turn
commands as the fake person moves.

To copy the repo to the robot over Ethernet: `scripts/deploy.sh`.

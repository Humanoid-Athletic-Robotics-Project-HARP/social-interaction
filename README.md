# HARP Human Interaction

Current goal: make the Unitree G1 walk a specific distance, using ROS 2 and
only Unitree's official code.

Everything below runs **on the robot's Jetson** (`ssh unitree@192.168.123.164`).

## 1. One-time setup: Unitree's official ROS 2 package

Check that ROS 2 is installed (`ls /opt/ros`), then follow the
[unitree_ros2 README](https://github.com/unitreerobotics/unitree_ros2) to clone
it to `~/unitree_ros2` and build `cyclonedds_ws`. This gives you the robot's
message types (`unitree_api`, `unitree_go`, `unitree_hg`).

In `~/unitree_ros2/setup.sh`, change the network interface (`enp3s0`) to the
Jetson's interface on the `192.168.123.x` network (find it with `ip a`, usually
`eth0`). Then, in every new terminal:

```bash
source ~/unitree_ros2/setup.sh
```

Check you can see the robot's odometry:

```bash
ros2 topic echo /lf/odommodestate
```

## 2. Optional: move with Unitree's own example (no code from us)

Build the official examples (`cd ~/unitree_ros2/example && colcon build`,
then `source install/setup.bash`) and run:

```bash
ros2 run unitree_ros2_example g1_loco_client_example --set_velocity="0.2 0 0 5"
```

That walks at 0.2 m/s for 5 s, roughly 1 m. It doesn't measure anything, so the
real distance will be off.

## 3. Walk an exact distance: `move_distance.py`

Our one script. It sends the same walking command as Unitree's example, but
watches the robot's odometry and stops when the target distance is reached,
then prints how far the robot actually went.

```bash
python3 move_distance.py --ros-args -p distance:=0.5 -p speed:=0.2
```

- `distance`: meters, negative walks backward. Default 1.0.
- `speed`: m/s, capped at 0.4. Default 0.2.

## Safety

- Stand the robot up in its normal walking mode with the remote first. The
  script never changes the robot's mode.
- Clear at least 2 m beyond the target distance, and keep someone holding the
  remote the whole time.
- Start with 0.5 m.
- Each command only lasts 0.5 s, so if the script crashes or the network drops,
  the robot stops on its own. Ctrl-C also sends a stop.

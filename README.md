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

## 3. Walk a distance or turn an angle: `move_distance.py`

Our one script. It sends the same velocity command as Unitree's example, but
watches the robot's odometry and stops when the target is reached, then prints
how far the robot actually went or turned. Each run does one move: set exactly
one of `distance` or `angle`.

```bash
python3 move_distance.py --ros-args -p distance:=0.5 -p speed:=0.2
python3 move_distance.py --ros-args -p angle:=90.0 -p turn_speed:=20.0
```

- `distance`: meters, negative walks backward. Default 0.
- `speed`: positive m/s, capped at 0.4. Default 0.2.
- `angle`: degrees, positive turns left (counterclockwise). Default 0.
- `turn_speed`: positive deg/s, capped at 30. Default 20.

All values must be finite and written as decimals (`90.0`, not `90`), or ROS
rejects the parameter type.

The script aborts if valid odometry stops for 0.5 s, its velocity command is
rejected, the move takes much longer than expected, or a turn goes more than
10° the wrong way. Errors attempt a stop and return a nonzero exit code.
Distance and heading are estimated from odometry (heading is
`imu_state.rpy[2]`); accuracy and timeout settings need robot tests.

Before the first turn, check that heading is reported in radians and which way
it changes: turn the robot left with the remote while running
`ros2 topic echo /lf/odommodestate --field imu_state.rpy`. The third value
should increase.

## Safety

- Stand the robot up in its normal walking mode with the remote first. The
  script never changes the robot's mode.
- Clear at least 2 m beyond the target distance, and keep someone holding the
  remote the whole time.
- Start with 0.5 m, and 45° for turns.
- Commands request 0.5 s validity; verify actual expiry behavior on the robot.
- Reaching the target, an error, or Ctrl-C sends the stop command 5 times,
  0.1 s apart, in case some are lost. If the network is down, none get through.

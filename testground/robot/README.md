# Robot status

`status_server.py` runs on PC2 and serves the G1's health as JSON on port 28735
(`/status`) for the dashboard's Robot status view (`viewer/`):

- **Battery charge**, from `rt/lf/bmsstate` on Unitree's DDS.
- **Each motor's casing and winding temperature**, from `rt/lowstate`.
- **HARP's harness**: armed, start-up phase, handover, stop reason and the stock FSM,
  from its `/safety/hardware_status` on its ROS 2 network (domain 97).

It only listens: it publishes nothing, switches no service and runs at nice 10. The
limits it reports (85 °C casing, 120 °C winding, 10% battery) are where the harness
stops the robot.

```bash
source /opt/ros/foxy/setup.bash && export LD_LIBRARY_PATH=$HOME/cyclonedds_ws/install/cyclonedds/lib:$LD_LIBRARY_PATH && python3 status_server.py
```

Without ROS 2 sourced, or with `--no-harness`, it serves the battery and motors only.
Then open the dashboard's Robot status tile.

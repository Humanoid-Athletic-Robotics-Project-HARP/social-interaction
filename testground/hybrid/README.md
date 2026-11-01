# Hybrid view

The head lidar and the head camera's depth in one live 3D view, so the camera fills
the lidar's blind cone in front of the robot. How the two are put together, and what
it showed: `context/g1-sensor-fusion.md`.

`hybrid_viewer.py` runs on PC2 and serves a page on port 8082: lidar points in blue,
camera depth in orange (dim on the floor, bright on things sticking up), and how much
of the lidar's blind cone the camera sees.

Tabs along the top switch to the lidar's and the camera's own views
(`lidar/lidar_viewer.py`, `camera/camera_viewer.py`), served by the same program
from the same sensor streams: one program to start for all three. Colour is asked
of Unitree's videohub only once the Camera tab is opened.

It reads both sensors itself; whichever another program already has (`lidar_viewer.py`,
`record.py`, `camera_viewer.py`, someone else's driver) says "Not connected.", and
`--no-lidar` / `--no-depth` leave them to others from the start. On PC2, in `~/Workspace/testground_social_interaction/hybrid`:

```bash
python3 hybrid_viewer.py
```

To touch no hardware at all, so it can run while someone else has the sensors, it
listens to ROS 2 topics instead: a lidar `PointCloud2` in the lidar's own frame and a
depth `Image`, from whatever node owns the devices (`--lidar-topic`, `--depth-topic`;
colour still comes from videohub):

```bash
source /opt/ros/foxy/setup.bash && export ROS_DOMAIN_ID=42 LD_LIBRARY_PATH=$HOME/cyclonedds_ws/install/cyclonedds/lib:$LD_LIBRARY_PATH && python3 hybrid_viewer.py --source ros
```

By default the lidar comes from Unitree's `lidar_driver` service (on Unitree's DDS) and
depth from `../camera/depth_node.py` (on ROS 2). The `LD_LIBRARY_PATH` keeps Unitree's
SDK on its own CycloneDDS, not Foxy's.

with `ROS_DOMAIN_ID` set to the publishers' domain (it refuses 0, Unitree's: see
`context/robot-pc2.md`). Until something publishes, both say "Not connected."

Then open `http://ADDRESS:8082` (PC2's Wi-Fi address, in the main checkout's `.env`).
`--camera-yaw DEG` turns the camera's points if they don't line up with the lidar's
where both see.

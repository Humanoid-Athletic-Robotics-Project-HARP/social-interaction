# Hybrid view

The head lidar and the head camera's depth in one live 3D view, so the camera fills
the lidar's blind cone in front of the robot. How the two are put together, and what
it showed: `context/g1-sensor-fusion.md`.

`hybrid_viewer.py` runs on PC2 and serves a page on port 8082: lidar points in blue,
camera depth in orange (dim on the floor, bright on things sticking up), and how much
of the lidar's blind cone the camera sees.

It reads both sensors itself, so first stop `lidar_viewer.py`, `record.py` and
`camera_viewer.py`. On PC2, in `~/Workspace/testground_social_interaction/hybrid`:

```bash
python3 hybrid_viewer.py
```

Then open `http://ADDRESS:8082` (PC2's Wi-Fi address, in the main checkout's `.env`).
`--camera-yaw DEG` turns the camera's points if they don't line up with the lidar's
where both see.

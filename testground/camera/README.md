# Head camera

The G1's head camera, an Intel RealSense D435i, live in a browser. What it is and what
it can see: `context/g1-head-camera.md`.

`camera_viewer.py` runs on PC2 and serves a page on port 8081:

- **Colour**, from Unitree's videohub service (it asks for JPEGs; videohub owns the
  colour device).
- **Depth**, read straight from `/dev/video0`, coloured by distance; click a point
  for its distance, how far ahead and to the side it is, and its height.
- **In front of the robot, seen from above**: the depth as a top-down map (the robot
  at the bottom, facing up; rings every metre), levelled on a floor plane fitted
  every second: grey is floor, coloured dots are things sticking up, by height. The
  fit also gives the camera's real height and pitch.

On PC2, in `~/Workspace/testground_social_interaction/camera`:

```bash
python3 camera_viewer.py
```

Then open `http://ADDRESS:8081` (PC2's Wi-Fi address, in the main checkout's `.env`)
from a computer on the same Wi-Fi. Ctrl-C stops it and releases the camera.
`--size 1280x720 --fps 30` for full-resolution depth; `--no-colour` for depth only.
Only one program can stream the depth camera: if another has it, or with `--no-depth`,
the depth panels say "Not connected." and colour, which anyone can share, keeps working.
It runs at low priority beside HARP's harness.

---
name: g1-face-visor
description: Read when the lidar's view ahead comes up, before putting the G1's face visor back on or taking it off, or before changing the lidar's mount height or the sim's blind cone
metadata:
  type: project
  updated: 2026-11-06T23:41:09Z
  covers: [submission/harp/lidar_mount.py, testground/lidar/visor_model.py, testground/lidar/self_filter_check.py]
---

The face visor blinds the head lidar ahead; with it off, the lidar sees ahead.
Measured 2026-11-02 (`2026-11-02/lidar/visor-off-standing-60s`, [[robot-reference-data]]):
the visor was off for that session; check the robot before assuming either way.

**Visor on** ([[g1-lidar-measured]]): nothing real within about ±60° of ahead at any
elevation, over raw UDP and Unitree's DDS alike (`dds-trial-20s`).

**Visor off** (torso frame): it sees ±38–40° either side of ahead from the top of its
fan down to about −23°, narrowing to ±30° at −35° and about ±10° at −47° (the chin);
the face frame's sides stay blind out to about ±56°, and the band holding the camera
blocks the topmost row. 34.5% of the front half is blind (visor on: 62.7%). The returns
ahead are real: normal reflectivity, at 1.75–5.8 m in that room. With the torso's
about 6° lean, a person ahead is seen from about 1.1 m up at 2 m (1.0 m at 3 m) down,
and the floor from about 1.4 m ahead.

**The head:** behind the visor is a cavity in an oval face frame, the D435i in a band at
its top and the lidar's glass dome just below it (中邮证券 G1 teardown, pp. 30–31, with a
Unitree side drawing showing the visor in front of both sensors). No official removal
procedure found. Livox's Mid-360 manual: nothing, glass included, may sit in its view.

**In the sim:** `harp/lidar_mount.py` (`VISOR`, off by default; `VIEW`, `sees`) is
the head's shadow fitted to both maps, used by the sim lidar, the stand-in person
detector and Nav2's clearing views ([[sim-vs-real]]). Set `VISOR` to the robot's state.

**The model** (`testground/lidar/visor_model.py`, not yet run in its committed form;
the numbers are from the scratch version it came from): Unitree's `head_link.STL` (no
visor: its face is the open frame) plus a visor on the frame's front, rays cast in
MuJoCo on Unitree's G1 model posed with the recorded joints, from the lidar's optical
centre. It predicted both states: visor on, 91% of cells agree with the measurement;
visor off, 89% (32.8% blind predicted, 34.5% measured). Not modelled: blind strips at
about ±90–105° at every elevation, in both states (harness straps or the head's sides).

**Lidar height:** Unitree's URDF origin (`harp/lidar_mount.py`, z 0.4284) is 21 mm above
the optical centre that its own mesh's window dome gives (Livox: 13 mm above the dome's
bottom, z 0.408; its manual puts the IMU 44 mm from the point origin toward the mounting
face). Three measurements favour the optical centre:
- the blind maps: from the URDF origin the model blocks about 5 times as many cells that
  are really open;
- the shoulders (`testground/lidar/self_filter_check.py`, both captures with joints,
  2026-11-01 and 11-02, different arm poses): from the URDF origin real shoulder returns
  are 4.3–4.6 cm nearer than the model and half the rays miss it; from 21 mm lower the
  median gap is −0.8 to −1.1 cm, 66–77% within 2 cm, and 88–89% meet it (best 21–30 mm);
- the floor, weakly: a plane fit puts it 1210 mm below the points' origin (2026-11-02),
  and the model with that capture's joints puts the soles 1222 mm below the URDF origin
  and 1201 mm below the optical centre.
`lidar_mount.POSITION` still uses the URDF origin; moving it changes the sim lidar, its
TF (Nav2's heights), the self-filter and `VIEW`'s fit, so compare course runs.

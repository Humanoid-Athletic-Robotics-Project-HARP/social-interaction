---
name: g1-face-visor
description: Read when the lidar's blind wedge ahead comes up, before taking the G1's face visor off, or before changing the lidar's mount height or the sim's blind cone
metadata:
  type: project
  updated: 2026-11-02T00:39:39Z
  covers: [submission/harp/lidar_mount.py, testground/lidar/visor_model.py]
---

The head lidar's blind wedge ahead ([[g1-lidar-measured]]) is the face visor. Modelled
2026-11-01, not yet tried on the robot.

**The head:** behind the visor is a cavity in an oval face frame, the D435i in a band at
its top and the lidar's glass dome just below it (中邮证券 G1 teardown, pp. 30–31, with a
Unitree side drawing showing the visor in front of both sensors). The visor comes off
in that teardown; no official removal procedure found. Livox's Mid-360 manual: nothing,
glass included, may sit in its view. Unitree's `head_link.STL` has no visor: its face is
the open frame.

**The model** (`testground/lidar/visor_model.py`; the numbers below are from the
scratch version it was written from, and it hasn't been run itself yet): a visor added
on the frame's front, rays cast in MuJoCo (the submission image's 3.2.3) on Unitree's
G1 model posed with the recorded joints (`standing-60s`), from the lidar's optical centre. With the visor: 62.5% of the front half blind
(measured 62.7%), 91% of cells agree. Not modelled: blind strips at about ±95–105° of
azimuth at every elevation (harness straps or the head's sides; unchecked).

**Removing it would open** (torso frame) ±38–40° of azimuth from the top of the fan
down to about −23°, ±30° at −35°, ±8° at −47° (the chin). The frame's sides keep about
40–64° each side blind; the camera blocks only the topmost rays. With the torso's
about 6° lean, a person ahead is seen from about 1.1 m up at 2 m (1.0 m at 3 m) down,
and the floor from about 1.4 m ahead.

**Lidar height:** Unitree's URDF origin (`harp/lidar_mount.py`, z 0.4284) is 21 mm above
the optical centre that its own mesh's window dome gives (Livox: 13 mm above the dome's
bottom, z 0.408). The measured map fits the mesh's; from the URDF origin the model
blocks about 5 times as many cells that are really open. Check against the measured
floor height before changing the mount.

**To confirm:** a 20 s capture with the visor off, then `blind_map.py` on it, compared
with `visor_model.py`'s visor-off map.

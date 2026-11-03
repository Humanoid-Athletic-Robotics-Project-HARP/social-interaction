---
name: approach-stop
description: Read before changing what stands the robot while something comes at it (harp/approach_stop.py, between Nav2 and our node), or what holds zero on lost input
metadata:
  type: project
  updated: 2026-11-03T17:33:36Z
  covers: [submission/harp/approach_stop.py, submission/tests/test_approach_stop.py]
---

**Rule** (the maintainer's call, 2026-10-30; ask before changing it): the robot stands while something comes at it, and Nav2 drives again once it has passed; it never dodges (people step around a robot that stands still; a swerve is hard to read and a quick sidestep is where a biped stumbles). It is the default; the opt-in, sim-only predictive filter ([[predictive-avoidance]]) does sidestep and evade, so making it the default needs the maintainer's say. Still obstacles, standing people included, stay Nav2's. Nav2's controller and recoveries publish `/cmd_vel_nav` (`nav2.launch.py`); the node passes it to `/cmd_vel` or sends zero.

**How:** it reads the raw clouds, not the costmap: the nearest point in a strip 0.7 m to each side, outside the arms, from the lidar (0.3–1.8 m up, to 6 m; with the visor off it sees the strip from about 1 m out) and from the latest head-camera frame (0.1–1.8 m up, about 0.3–2.7 m ahead on the robot, farther in the sim: [[sim-vs-real]]; legs only), by straight-line range, which turning doesn't change. "Coming" when 4 of the last 5 scan-to-scan steps (6 scans, about 0.5 s) saw it close 0.3 m/s faster than the robot walks (a pole swinging into the strip is one step). Once standing, it also stays while the nearest point moved over 0.3 m/s across those 6 scans (someone walking across the strip), and goes 1 s after neither holds. Its rule is tested in `submission/tests/`.

**Lost input:** it holds zero from startup until a usable cloud, a valid live lidar transform and finite odometry speed arrive, and while any is over 0.5 s old (empty, malformed, wrong-frame and all-invalid clouds don't count). A failed course-goal or route result latches zero until a new session. The startup hold runs in every HIM run; losing an input mid-run and the failed-goal latch have offline tests only. It's the only stop on lost lidar: Foxy's Nav2 doesn't act on a stale costmap ([[nav2-sim]], [[safety]]).

**Scoring:** `speed_near_people_mps`, the robot's fastest true speed (over 0.5 s) while a mover within 1.5 m comes closer; `oncoming` allows 0.1 m/s.

**Results, visor off** (2026-11-02, `runs/*_visor-off`, 0 contacts, 0 recoveries): oncoming PASS 29.6 s, stood 5.9 s from 3.56 m, 0.002 m/s near the person, closest 0.39 m; crossing PASS 25.0 s, stood 2.3 s at 1.11 m, closest 0.66 m. Visor on (2026-11-01, `runs/*_real-sensors-2`): the camera alone stood it in `oncoming` with the person 3.45 m away. It never stops on the courses without people. From the side it walks on at 0.25–0.27 m/s while someone comes within 1.5 m: the strip sees them only about 0.95 m to the side.

**Rejected:** watching beside the strip for someone coming in from the side: in slalom it stood 7 times for still poles (the nearest point on a wall or thick pole slides along it as the robot walks and sways), `runs/*_stop-sides*`. Doing it properly needs a moving-object tracker, like the predictive filter's (`harp/obstacle_tracker.py`). The "still moving" rule has the same weakness, but only acts once the robot already stands.

**Not covered:** something still that is too close (a collision-monitor stop is still open), and the robot: `entrypoint.sh` starts it only in a world's image in the sim.

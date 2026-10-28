---
name: waypoint-loop
description: Read when extending route state or connecting waypoint sequencing to navigation
metadata:
  type: reference
  updated: 2026-10-28T04:48:12Z
  covers: [ros2/src/harp_eval/harp_eval/waypoint_route.py, ros2/src/harp_eval/harp_eval/course.py, ros2/src/harp_eval/harp_eval/course_goal.py, ros2/src/harp_eval/launch/course.launch.py, ros2/src/harp_eval/config/rectangle.yaml, ros2/src/harp_eval/test/*.py]
---

`course WORLD --route NAME_OR_FILE` or repeated `--waypoint X Y [YAW_DEG]` runs a finite route through the existing simulator, lidar relay, walker and Nav2 launch. Bundled routes are `harp_eval/config/*.yaml`; `rectangle` is for `one_box`. `--laps N` overrides the YAML lap count; omitted laps default to one. The CLI validates and saves the effective input into `runs/<run>/route.yaml` before startup.

Route YAML requires `frame: odom` and nonempty `waypoints` of `[x_m, y_m, yaw_deg]` (yaw optional, default zero). Laps and repeats must be positive integers. All targets reuse the same odom origin across laps. `--goal`, `--route`, and `--waypoint` are mutually exclusive; `--laps` needs a route. `--timeout` bounds each waypoint, including server discovery/acceptance. `--repeat` starts a fresh simulation. File criteria remain unchanged when overriding laps; the bundled route has no fixed total-time budget.

`WaypointRoute.advance()` is called only after confirmed success. Goal rejection, failure, timeout, or Ctrl-C ends the course; cancellation acknowledgment and completion are bounded. Submission/cancellation use bounded asynchronous action calls because the Commander wrappers can block before a timeout check. Manual takeover confirms cancellation, waits for hand-back, then resends the same waypoint with the remaining execution budget. Separate `waypoint_paused`/`waypoint_resumed` events preserve progress; full wall time and route criteria include pauses, while waypoint timeouts exclude them. Walker stop/error ends a pause; `/walker/stop` damps the controller.

`goal_sent`/`goal_result` mark the entire route (start contains the final planned target); `waypoint_sent`/`waypoint_result` record individual attempts. The summary uses the whole route window and adds lap/waypoint counts. Route checks use only optional route `criteria`; absent criteria give an outcome with no PASS/FAIL verdict. The copied route is retained for later summaries.

After the normal build, run offline checks from `ros2/`:

```bash
pixi run --as-is python -m unittest discover -s src/harp_eval/test -v
```

**Recorded simulation:** `2026-10-28_04-44-05Z_sim_open_floor_review-manual-resume` (pinned, `ab33d11`): two inline targets, 3 laps, 6/6 visits, 67.7 s, 4.60 m, zero contacts/recoveries, 0.26 m end error. L1 paused waypoint 1; R1+A resumed the same target after 8.2 s, then the route completed. Earlier obstacle-route evidence is pinned in `2026-10-28_00-49-07Z_sim_one_box_review-custom-route`: 6/6 visits over 3 laps, zero contacts. Physical behavior remains untested.

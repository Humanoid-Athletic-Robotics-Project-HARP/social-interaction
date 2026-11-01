#!/usr/bin/env python3
"""Watch the planar avoidance controller with live MuJoCo raycast observations.

On macOS launch using mjpython. Close the viewer or press Ctrl-C to exit.
"""
import argparse
import json
import time

import mujoco
import mujoco.viewer
import numpy as np

from test_predictive_sim import run, scenarios


class LiveView:
    def __init__(self, goal):
        self.goal = goal
        self.viewer = None
        self.start = None
        self.mode = None
        self.trail = []

    def geom(self, kind, size, pos, color):
        scene = self.viewer.user_scn
        if scene.ngeom >= scene.maxgeom:
            return None
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, kind, np.asarray(size, dtype=float),
                           np.asarray(pos, dtype=float), np.eye(3).ravel(),
                           np.asarray(color, dtype=np.float32))
        scene.ngeom += 1
        return geom

    def line(self, a, b, color, width=2):
        geom = self.geom(mujoco.mjtGeom.mjGEOM_LINE, [0, 0, 0], [0, 0, 0], color)
        if geom is not None:
            mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, width,
                                np.asarray(a, dtype=float), np.asarray(b, dtype=float))

    def __call__(self, world, pose, stamp, points, tracks, result):
        if self.viewer is None:
            world.model.geom_rgba[0] = [.15, .45, .95, 1]
            world.model.geom_rgba[1:] = [1, .45, .1, 1]
            self.viewer = mujoco.viewer.launch_passive(world.model, world.data)
            self.viewer.cam.lookat[:] = [1, 0, 0]
            self.viewer.cam.distance = 8
            self.viewer.cam.elevation = -65
            self.viewer.cam.azimuth = 90
            self.start = time.monotonic()
        if not self.viewer.is_running():
            raise KeyboardInterrupt
        time.sleep(max(0, self.start + stamp - time.monotonic()))
        self.trail.append([pose[0], pose[1], .04])
        colors = {'follow': [.15, .45, .95, 1], 'evade': [.7, .2, .95, 1],
                  'yield': [1, .85, .1, 1], 'blocked': [1, .1, .1, 1]}
        world.model.geom_rgba[0] = colors[result.mode]
        with self.viewer.lock():
            self.viewer.user_scn.ngeom = 0
            for value in range(-5, 8):
                self.line([value, -5, 0], [value, 5, 0], [.3, .3, .3, 1], 1)
                self.line([-5, value, 0], [7, value, 0], [.3, .3, .3, 1], 1)
            self.geom(mujoco.mjtGeom.mjGEOM_SPHERE, [.12, 0, 0],
                      [self.goal[0], self.goal[1], .05], [.1, 1, .2, 1])
            # These are the actual ray hits given to the tracker, not object truth.
            for point in points[::max(1, len(points)//150)]:
                self.line([pose[0], pose[1], .6], [*point, .6], [.2, .8, .3, .25], 1)
                self.geom(mujoco.mjtGeom.mjGEOM_SPHERE, [.025, 0, 0],
                          [*point, .62], [.2, 1, .3, 1])
            for track in tracks:
                self.line([track.x, track.y, 1.3],
                          [track.x+track.vx, track.y+track.vy, 1.3], [1, 1, .2, 1], 4)
            recent = self.trail[-200:]
            for a, b in zip(recent, recent[1:]):
                self.line(a, b, [.2, .8, 1, 1], 3)
        self.viewer.sync()
        if result.mode != self.mode:
            print(f"t={stamp:.1f}s  mode={result.mode}  alert={result.alert}", flush=True)
            self.mode = result.mode

    def close(self):
        if self.viewer is not None:
            self.viewer.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=list(scenarios()), default='head_on')
    args = parser.parse_args()
    spec = scenarios()[args.scenario]
    view = LiveView(spec['goal'])
    print('Planar avoidance: orange=objects, green=LiDAR hits/goal, yellow=tracked velocity.\n'
          'Robot: blue=follow, purple=evade, yellow=yield, red=blocked/alert.\n'
          'Close the window to exit. Walking G1/GR00T is not part of this view.', flush=True)
    try:
        report, _ = run(args.scenario, spec, observer=view)
        print(json.dumps(report, indent=2), flush=True)
        print('Scenario complete. Close the window to exit.', flush=True)
        while view.viewer.is_running():
            view.viewer.sync()
            time.sleep(.05)
    except KeyboardInterrupt:
        pass
    finally:
        view.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

"""Native simulator entrypoint; uses existing G1 dynamics and LiDAR unchanged."""
import os
import sys
import numpy as np
from harp_sim import run_sim, lidar
from harp.crowd_motion import CrowdMotion


class SeededLidar(lidar.SimLidar):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.rng = np.random.default_rng(int(os.environ.get('HARP_CROWD_SEED','1'))+123)


if __name__ == '__main__':
    # Preserve this entrypoint when macOS switches to MuJoCo's GUI interpreter.
    # The historical launcher otherwise restarts harp_sim.run_sim directly,
    # silently dropping the seeded LiDAR and continuous-crowd overrides.
    if (sys.platform == 'darwin' and '--headless' not in sys.argv
            and not os.environ.get(run_sim.UNDER_MJPYTHON)):
        os.environ[run_sim.UNDER_MJPYTHON] = '1'
        os.execvp('mjpython', ['mjpython', '-m', 'native_crowd_world', *sys.argv[1:]])
    lidar.SimLidar = SeededLidar
    if os.environ.get('HARP_CROWD_MODE') != 'legacy':
        run_sim.Movers = CrowdMotion
    raise SystemExit(run_sim.main())

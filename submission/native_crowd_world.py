"""Native simulator entrypoint; uses existing G1 dynamics and LiDAR unchanged."""
import os
import numpy as np
from harp_sim import run_sim, lidar
from harp.crowd_motion import CrowdMotion


class SeededLidar(lidar.SimLidar):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.rng = np.random.default_rng(int(os.environ.get('HARP_CROWD_SEED','1'))+123)


if __name__ == '__main__':
    lidar.SimLidar = SeededLidar
    if os.environ.get('HARP_CROWD_MODE') != 'legacy':
        run_sim.Movers = CrowdMotion
    raise SystemExit(run_sim.main())

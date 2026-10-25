"""Simulated G1 with a stand-in for Unitree's `sport` service, so code that
walks the real robot through its built-in controller (LocoClient,
move_distance.py) can run against MuJoCo instead.

The real sport service is Unitree's closed walking controller. This stand-in
does not walk: it slides the whole robot, legs fixed, at the commanded
velocity. It checks script logic (requests, odometry, stopping), not how the
real robot moves.

Answers on rt/api/sport/request, like the robot:
  - 7105 SetVelocity {"velocity":[vx,vy,omega],"duration":s}, body frame.
    The velocity is held for `duration` seconds, then drops to zero.
  - 7101 SetFsmId {"data":id}: stored only, velocity works in any mode.
  - 7001 GetFsmId.
  Anything else gets error 3203 (not implemented).
Publishes rt/lf/odommodestate (unitree_go SportModeState) at 20 Hz: position,
velocity (world frame), imu_state.rpy and quaternion (w, x, y, z), yaw_speed.
Yaw is counterclockwise-positive; whether the real robot agrees is untested.

Run from the sim venv (README, Simulation section), DDS domain 1 by default:
  mjpython sim/fake_sport.py               # macOS, with viewer
  python3 sim/fake_sport.py --headless
"""

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.rpc.internal import RPC_ERR_SERVER_API_PARAMETER
from unitree_sdk2py.rpc.server import Server

SET_VELOCITY_API_ID = 7105
SET_FSM_ID_API_ID = 7101
GET_FSM_ID_API_ID = 7001
START_FSM_ID = 500        # LocoClient.Start(); the robot is assumed to be walking already
STEP_DT = 0.01            # s, pose integration step
ODOM_PERIOD = 0.05        # s, 20 Hz like the robot's lf/odommodestate
VIEWER_PERIOD = 0.02      # s

DEFAULT_SCENE = (Path(__file__).resolve().parents[2]
                 / "unitree_mujoco/unitree_robots/g1/scene_29dof.xml")


class FakeSport(Server):
    """Holds the latest velocity command and the robot's planar pose."""

    def __init__(self):
        super().__init__("sport")
        self.lock = threading.Lock()
        self.command = (0.0, 0.0, 0.0)   # vx, vy (body frame, m/s), omega (rad/s)
        self.command_until = 0.0         # time.monotonic() when the command expires
        self.fsm_id = START_FSM_ID
        self.x = self.y = self.yaw = 0.0

    def Init(self):
        self._SetApiVersion("1.0.0.0")  # LocoClient's LOCO_API_VERSION
        self._RegistHandler(SET_VELOCITY_API_ID, self.set_velocity, False)
        self._RegistHandler(SET_FSM_ID_API_ID, self.set_fsm_id, False)
        self._RegistHandler(GET_FSM_ID_API_ID, self.get_fsm_id, False)

    def set_velocity(self, parameter):
        try:
            p = json.loads(parameter)
            vx, vy, omega = (float(v) for v in p["velocity"])
            duration = float(p.get("duration", 1.0))
        except (ValueError, KeyError, TypeError):
            return RPC_ERR_SERVER_API_PARAMETER, ""
        if not all(math.isfinite(v) for v in (vx, vy, omega, duration)) or duration < 0:
            return RPC_ERR_SERVER_API_PARAMETER, ""
        with self.lock:
            self.command = (vx, vy, omega)
            self.command_until = time.monotonic() + duration
        return 0, ""

    def set_fsm_id(self, parameter):
        try:
            fsm_id = int(json.loads(parameter)["data"])
        except (ValueError, KeyError, TypeError):
            return RPC_ERR_SERVER_API_PARAMETER, ""
        with self.lock:
            self.fsm_id = fsm_id
        return 0, ""

    def get_fsm_id(self, parameter):
        with self.lock:
            return 0, json.dumps({"data": self.fsm_id})

    def step(self, dt):
        """Advance the pose by dt; return the world-frame velocity (vx, vy, omega)."""
        with self.lock:
            if time.monotonic() >= self.command_until:
                self.command = (0.0, 0.0, 0.0)
            vx, vy, omega = self.command
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            wvx, wvy = c * vx - s * vy, s * vx + c * vy
            self.x += wvx * dt
            self.y += wvy * dt
            self.yaw = (self.yaw + omega * dt + math.pi) % (2.0 * math.pi) - math.pi
            return wvx, wvy, omega


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--domain", type=int, default=1, help="DDS domain id (robot uses 0)")
    parser.add_argument("--interface", default="lo0" if sys.platform == "darwin" else "lo")
    parser.add_argument("--scene", default=str(DEFAULT_SCENE))
    parser.add_argument("--headless", action="store_true", help="no viewer window")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(args.scene)
    data = mujoco.MjData(model)
    free = next(j for j in range(model.njnt) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE)
    base = model.jnt_qposadr[free]       # qpos[base:base+7] = x, y, z, qw, qx, qy, qz
    height = float(data.qpos[base + 2])  # keep the model's standing height

    ChannelFactoryInitialize(args.domain, args.interface)
    sport = FakeSport()
    sport.Init()
    sport.Start()
    odom_pub = ChannelPublisher("rt/lf/odommodestate", SportModeState_)
    odom_pub.Init()
    odom = unitree_go_msg_dds__SportModeState_()

    viewer = None
    if not args.headless:
        # macOS needs mjpython for this
        viewer = mujoco.viewer.launch_passive(model, data)
    print(f"Fake sport service on domain {args.domain}, {args.interface}. Ctrl-C to quit.")

    next_step = time.monotonic()
    last_odom = last_view = 0.0
    try:
        while viewer is None or viewer.is_running():
            wvx, wvy, omega = sport.step(STEP_DT)
            half = sport.yaw / 2.0
            quat = (math.cos(half), 0.0, 0.0, math.sin(half))
            now = time.monotonic()

            if now - last_odom >= ODOM_PERIOD:
                odom.position = [sport.x, sport.y, height]
                odom.velocity = [wvx, wvy, 0.0]
                odom.yaw_speed = omega
                odom.imu_state.rpy = [0.0, 0.0, sport.yaw]
                odom.imu_state.quaternion = list(quat)
                odom.imu_state.gyroscope = [0.0, 0.0, omega]
                odom_pub.Write(odom)
                last_odom = now

            if viewer is not None and now - last_view >= VIEWER_PERIOD:
                with viewer.lock():
                    data.qpos[base:base + 3] = (sport.x, sport.y, height)
                    data.qpos[base + 3:base + 7] = quat
                    mujoco.mj_forward(model, data)
                viewer.sync()
                last_view = now

            next_step += STEP_DT
            time.sleep(max(0.0, next_step - time.monotonic()))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

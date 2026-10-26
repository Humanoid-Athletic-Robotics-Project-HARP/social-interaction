"""MuJoCo sim of the 29-joint G1 that speaks the robot's low-level DDS topics,
for running our own controllers (harp_control) before the real robot.

Like unitree_mujoco's simulate_python, and using its G1 model, but:
  - The PD torque (tau + kp*(q_cmd - q) + kd*(dq_cmd - dq)) is recomputed at
    every physics step from the latest rt/lowcmd, as the robot's motor drivers
    do. unitree_mujoco computes it once per command and holds it, which made
    the GR00T policy blow up the sim on landing.
  - Steps are paced against the wall clock, so it runs at real time.
  - imu_state.rpy and tick are filled in.
  - It needs none of unitree_mujoco's three local fixes (no RecurrentThread,
    no glfw import).

Publishes rt/lowstate (unitree_hg) every step; reads rt/lowcmd. Until a
command arrives the joints are limp. The robot starts hanging from a virtual
elastic band on the torso. In the viewer: 7 shortens the band (raises the
robot), 8 lengthens it (lowers it), 9 releases or re-attaches it.

--world NAME adds a course of obstacles from worlds/NAME.xml (or any MuJoCo
file with geoms in its worldbody); --box X Y puts a fixed box on the floor,
X m ahead and Y m left of where the robot starts; --lidar adds the head lidar
(lidar.py) as a ROS 2 point cloud. Whenever the robot touches an obstacle,
the sim prints it, so a run that prints no contact never hit anything.
--record FILE saves the robot's true pose (qpos) at 20 Hz to FILE.npz on
quitting, for measuring paths and clearances afterwards.

Run from the ROS 2 environment (DDS domain 1 by default):
  ros2 run harp_sim run_sim
  ros2 run harp_sim run_sim --headless --auto-release 4
  ros2 run harp_sim run_sim --lidar --box 2 0 --box 3 -1 0.3 1.0 0.8
  ros2 run harp_sim run_sim --lidar --world obstacle_course
On macOS the viewer needs mjpython, so with the viewer it restarts itself
under mjpython.
"""

import argparse
import math
import os
import sys
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np
from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

# Unitree's G1 model (unitree_mujoco, cloned next to this repo). The ROS 2
# environment (ros2/pixi.toml) sets HARP_G1_SCENE to it.
DEFAULT_SCENE = os.environ.get("HARP_G1_SCENE")
UNDER_MJPYTHON = "HARP_SIM_UNDER_MJPYTHON"   # set once we have restarted under mjpython
SIM_DT = 0.005            # s, as in NVIDIA's GR00T sim and unitree_mujoco
VIEWER_PERIOD = 0.02      # s
BAND_POINT = np.array([0.0, 0.0, 3.0])   # band anchor above the robot, as in unitree_mujoco
BAND_STIFFNESS, BAND_DAMPING = 200.0, 100.0
BAND_STEP = 0.1           # m per key press
STANDING_PELVIS_Z = 0.80  # auto-release: lower the band until the pelvis is this low,
RELEASE_DELAY = 3.0       # then wait this long (time to start the policy) and release
MODE_MACHINE = 5          # what a 29-joint G1 rev 1.0 reports in lowstate
RECORD_PERIOD = 0.05      # s between --record samples
MAX_LAG = 0.1             # s behind real time before giving up on catching up
BOX_SIZE = (0.5, 0.5, 0.5)   # m, width (x), depth (y), height, when --box gives only X Y
BOX_RGBA = (0.8, 0.5, 0.2, 1.0)


class Band:
    """Virtual elastic band holding the torso up, like unitree_mujoco's."""

    def __init__(self):
        self.length = 0.0
        self.enabled = True

    def force(self, pos, vel):
        delta = BAND_POINT - pos
        distance = np.linalg.norm(delta)
        direction = delta / distance
        speed = np.dot(vel, direction)
        return (BAND_STIFFNESS * (distance - self.length) - BAND_DAMPING * speed) * direction

    def on_key(self, key):
        if key == ord("7"):
            self.length -= BAND_STEP
        elif key == ord("8"):
            self.length += BAND_STEP
        elif key == ord("9"):
            self.enabled = not self.enabled
            print("Band attached." if self.enabled else
                  "Band released: the robot is on its own (9 re-attaches).")


class BoxArg(argparse.Action):
    """--box X Y [W D H]: appends (x, y, w, d, h) to the boxes."""

    def __call__(self, parser, namespace, values, option_string=None):
        if len(values) not in (2, 5):
            parser.error(f"{option_string} takes X Y or X Y W D H, got {len(values)} numbers")
        try:
            numbers = [float(v) for v in values]
        except ValueError:
            parser.error(f"{option_string} takes numbers, got {' '.join(values)}")
        if not all(math.isfinite(n) for n in numbers) or any(n <= 0 for n in numbers[2:]):
            parser.error(f"{option_string} takes finite numbers and positive sizes")
        boxes = getattr(namespace, self.dest) or []
        boxes.append(tuple(numbers) if len(numbers) == 5 else (*numbers, *BOX_SIZE))
        setattr(namespace, self.dest, boxes)


def world_path(name):
    """A world file: a path, or the name of one of harp_sim's worlds/."""
    if os.path.exists(name):
        return name
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory("harp_sim"), "worlds", name + ".xml")
    if not os.path.exists(path):
        raise SystemExit(f"No world {name}: not a file, and not in {os.path.dirname(path)}")
    return path


def load_model(scene, boxes, world=None):
    """Unitree's G1 scene plus a course (world file) and fixed boxes standing on the floor."""
    spec = mujoco.MjSpec.from_file(scene)
    if world:
        # Its worldbody's geoms and sites join ours, named course/...
        spec.attach(mujoco.MjSpec.from_file(world_path(world)), prefix="course/",
                    frame=spec.worldbody.add_frame())
    for i, (x, y, w, d, h) in enumerate(boxes):
        spec.worldbody.add_geom(name=f"box{i}", type=mujoco.mjtGeom.mjGEOM_BOX,
                                size=[w / 2, d / 2, h / 2], pos=[x, y, h / 2], rgba=BOX_RGBA)
    return spec.compile()


class ContactReport:
    """Prints each time a robot body starts touching an obstacle (anything
    fixed in the world except the floor). Contacts flicker on and off while
    touching, so one with a gap under GAP seconds counts as the same touch."""

    GAP = 1.0   # s

    def __init__(self, model):
        self.model = model
        self.obstacles = {g for g in range(model.ngeom)
                          if model.geom_bodyid[g] == 0 and model.geom(g).name != "floor"}
        self.last_seen = {}   # (robot body, obstacle geom) -> sim time
        self.count = 0

    def check(self, data):
        for c in data.contact[:data.ncon]:
            for robot, obstacle in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                body = self.model.geom_bodyid[robot]
                if obstacle not in self.obstacles or body == 0:
                    continue
                if data.time - self.last_seen.get((body, obstacle), -math.inf) > self.GAP:
                    self.count += 1
                    print(f"Contact {self.count} at t={data.time:.1f} s: {self.model.body(body).name} "
                          f"touched {self.model.geom(obstacle).name or f'geom {obstacle}'}", flush=True)
                self.last_seen[(body, obstacle)] = data.time


def sensor_slice(model, name):
    i = model.sensor(name).id
    return slice(model.sensor_adr[i], model.sensor_adr[i] + model.sensor_dim[i])


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--domain", type=int, default=1, help="DDS domain id (robot uses 0)")
    parser.add_argument("--interface", default="lo0" if sys.platform == "darwin" else "lo")
    parser.add_argument("--scene", default=DEFAULT_SCENE,
                        help="G1 MuJoCo model (default: $HARP_G1_SCENE)")
    parser.add_argument("--headless", action="store_true", help="no viewer window")
    parser.add_argument("--auto-release", type=float, metavar="SECONDS",
                        help="after this long, lower the robot onto its feet and release the band")
    parser.add_argument("--duration", type=float, help="quit after this many seconds")
    parser.add_argument("--log", action="store_true", help="print the pelvis pose every second")
    parser.add_argument("--record", metavar="FILE",
                        help="save the true qpos at 20 Hz to FILE (.npz) on quitting")
    parser.add_argument("--world", metavar="NAME",
                        help="a course of obstacles: worlds/NAME.xml in harp_sim, or a file path")
    parser.add_argument("--box", dest="boxes", action=BoxArg, nargs="+", metavar="NUM", default=[],
                        help="X Y [W D H]: a fixed box X m ahead and Y m left of the start, "
                             f"W x D x H m (default {' x '.join(map(str, BOX_SIZE))}); repeatable")
    parser.add_argument("--lidar", action="store_true",
                        help="simulate the head lidar and publish it as a ROS 2 point cloud")
    args = parser.parse_args()
    if not args.scene:
        parser.error("no G1 model: pass --scene, or set HARP_G1_SCENE (the ROS 2 environment does)")
    if sys.platform == "darwin" and not args.headless and not os.environ.get(UNDER_MJPYTHON):
        # MuJoCo's viewer only runs under mjpython on macOS; ros2 run starts plain python.
        os.environ[UNDER_MJPYTHON] = "1"
        os.execvp("mjpython", ["mjpython", "-m", "harp_sim.run_sim", *sys.argv[1:]])

    model = load_model(args.scene, args.boxes, args.world)
    contacts = ContactReport(model)
    model.opt.timestep = SIM_DT
    data = mujoco.MjData(model)
    nu = model.nu
    joints = [model.actuator_trnid[i, 0] for i in range(nu)]
    qadr = np.array([model.jnt_qposadr[j] for j in joints])
    vadr = np.array([model.jnt_dofadr[j] for j in joints])
    torque_low, torque_high = model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1]
    imu_quat = sensor_slice(model, "imu_quat")
    imu_gyro = sensor_slice(model, "imu_gyro")
    imu_acc = sensor_slice(model, "imu_acc")
    torso = model.body("torso_link").id

    lidar = None
    if args.lidar:
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from harp_sim.lidar import PERIOD as LIDAR_PERIOD, TOPIC as LIDAR_TOPIC, SimLidar
        # ROS 2 on its own domain (ROS_DOMAIN_ID, 2), the robot's topics on args.domain.
        # No signal handlers: Ctrl-C stays a KeyboardInterrupt for the loop below.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        lidar = SimLidar(model)
        next_scan = 0.0

    ChannelFactoryInitialize(args.domain, args.interface)
    state_pub = ChannelPublisher("rt/lowstate", LowState_)
    state_pub.Init()
    latest_cmd = [None]
    cmd_sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
    cmd_sub.Init(lambda msg: latest_cmd.__setitem__(0, msg), 10)
    state = unitree_hg_msg_dds__LowState_()

    band = Band()
    viewer = None
    if not args.headless:
        viewer = mujoco.viewer.launch_passive(model, data, key_callback=band.on_key)

        def draw():
            # Separate thread, so drawing never delays physics or rt/lowstate.
            while viewer.is_running():
                viewer.sync()
                time.sleep(VIEWER_PERIOD)
        threading.Thread(target=draw, daemon=True).start()
    print(f"G1 sim on DDS domain {args.domain}, {args.interface}. Ctrl-C here quits.")
    if viewer is not None:
        print("MuJoCo window keys: 8 lowers the robot, 7 raises it, 9 releases or "
              "re-attaches the band. Controller keys go to the controller's terminal.")
    if lidar is not None:
        print(f"Lidar: sensor_msgs/PointCloud2 on {LIDAR_TOPIC} (ROS 2), 10 Hz.")
    for i in range(model.nsite):
        if model.site(i).name.startswith("course/goal"):
            x, y = model.site_pos[i][:2]
            print(f"Course {model.site(i).name[len('course/'):]}: {x:+.1f} m ahead, {y:+.1f} m left of the start")

    record_t, record_qpos, next_record = [], [], 0.0
    start = time.monotonic()
    next_step = start
    release_at = None
    try:
        while viewer is None or viewer.is_running():
            now = time.monotonic()
            if args.duration and now - start > args.duration:
                break

            if args.auto_release is not None and band.enabled and data.time > args.auto_release:
                if data.qpos[2] > STANDING_PELVIS_Z:
                    band.length += 0.5 * SIM_DT   # lower at 0.1 m/s
                elif release_at is None:
                    release_at = data.time + RELEASE_DELAY
                    print(f"Feet down at t={data.time:.1f} s; releasing the band in {RELEASE_DELAY:.0f} s")
                elif data.time > release_at:
                    band.enabled = False
                    print(f"Band released at t={data.time:.1f} s: the robot is on its own.")
            if band.enabled:
                data.xfrc_applied[torso, :3] = band.force(data.qpos[:3], data.qvel[:3])
            else:
                data.xfrc_applied[torso, :3] = 0.0

            q, dq = data.qpos[qadr], data.qvel[vadr]
            cmd = latest_cmd[0]
            if cmd is None:
                data.ctrl[:] = 0.0
            else:
                m = cmd.motor_cmd
                q_cmd = np.array([m[i].q for i in range(nu)])
                dq_cmd = np.array([m[i].dq for i in range(nu)])
                kp = np.array([m[i].kp for i in range(nu)])
                kd = np.array([m[i].kd for i in range(nu)])
                tau = np.array([m[i].tau for i in range(nu)])
                torque = tau + kp * (q_cmd - q) + kd * (dq_cmd - dq)
                data.ctrl[:] = np.clip(torque, torque_low, torque_high)

            if viewer is not None:
                with viewer.lock():
                    mujoco.mj_step(model, data)
            else:
                mujoco.mj_step(model, data)

            for i in range(nu):
                ms = state.motor_state[i]
                ms.q, ms.dq, ms.tau_est = float(data.qpos[qadr[i]]), float(data.qvel[vadr[i]]), float(data.ctrl[i])
            w, x, y, z = data.sensordata[imu_quat]
            state.imu_state.quaternion = [float(w), float(x), float(y), float(z)]
            state.imu_state.gyroscope = data.sensordata[imu_gyro].tolist()
            state.imu_state.accelerometer = data.sensordata[imu_acc].tolist()
            state.imu_state.rpy = [
                math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
                math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
                math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
            ]
            state.mode_machine = MODE_MACHINE
            state.tick = (state.tick + round(SIM_DT * 1000)) % 2**32  # ms, like the robot
            state_pub.Write(state)
            contacts.check(data)
            if lidar is not None and data.time >= next_scan:
                lidar.scan(data.qpos)
                next_scan += LIDAR_PERIOD

            if args.record and data.time >= next_record:
                record_t.append(data.time)
                record_qpos.append(data.qpos.copy())
                next_record += RECORD_PERIOD

            if args.log and data.time % 1.0 < SIM_DT:
                pw, px, py, pz = data.qpos[3:7]
                yaw = math.degrees(math.atan2(2 * (pw * pz + px * py), 1 - 2 * (py * py + pz * pz)))
                print(f"t={data.time:5.1f}  pelvis x={data.qpos[0]:+.2f} y={data.qpos[1]:+.2f} "
                      f"z={data.qpos[2]:.2f} yaw={yaw:+6.1f}  wall={now - start:5.1f}", flush=True)

            next_step += SIM_DT
            lag = time.monotonic() - next_step
            if lag > MAX_LAG:
                print(f"Warning: the sim fell {lag * 1000:.0f} ms behind real time at "
                      f"t={data.time:.1f} s. A controller in sim mode stops if state is over 0.5 s late.")
                next_step = time.monotonic()   # don't try to catch up in a burst
            else:
                time.sleep(max(0.0, -lag))
    except KeyboardInterrupt:
        pass
    if args.record:
        np.savez(args.record, t=np.array(record_t), qpos=np.array(record_qpos),
                 world=args.world or "", boxes=np.array(args.boxes, dtype=float).reshape(-1, 5))
        print(f"Saved {len(record_t)} poses to {args.record}")
    if lidar is not None:
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

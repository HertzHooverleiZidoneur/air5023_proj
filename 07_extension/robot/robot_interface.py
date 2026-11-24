"""Synchronous adapter around the pinned official Stretch server.

All physics, actuator mappings and camera rendering come from stretch_mujoco.
The adapter owns scheduling so observations and task checks share one MjData.
Owner: TEAM-07; consumers: TEAM-06 observations, TEAM-01 base, TEAM-02 arm.
See docs/development/README.md before changing shared contracts.
"""
from dataclasses import asdict
import signal
import threading
import time
from types import SimpleNamespace

import mujoco
import numpy as np
from stretch_mujoco.datamodels.status_command import CommandBaseVelocity, CommandMove
from stretch_mujoco.enums.stretch_cameras import StretchCameras
from stretch_mujoco.mujoco_server import MujocoServer, MujocoServerProxies
from stretch_mujoco.mujoco_server_camera_manager import MujocoServerCameraManagerSync


JOINTS = ("lift", "arm", "wrist_yaw", "wrist_pitch", "wrist_roll", "head_pan", "head_tilt")


class RobotInterface:
    def __init__(self, scene=None, *, model=None, gui=False, realtime=False,
                 control_hz=None, camera_hz=None, cameras=True, sample_cameras_on_step=True):
        from configs import load
        config = load("robot")
        control_hz = config["control_hz"] if control_hz is None else control_hz
        camera_hz = config["camera_hz"] if camera_hz is None else camera_hz
        self.base_limits = np.asarray(config["base_velocity_limits"], dtype=float)
        if control_hz <= 0 or camera_hz <= 0 or camera_hz > control_hz:
            raise ValueError("Require 0 < camera_hz <= control_hz")
        self.control_hz, self.camera_hz = control_hz, camera_hz
        self.realtime = realtime
        self.viewer = None
        self.camera_manager = None
        self._sample_cameras_on_step = sample_cameras_on_step
        self._closed = False
        self._signals = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
        # Stop the server's optional asynchronous sensor reader before simulation.
        # Use ordinary dictionaries for the official single-process proxy API.
        self.event = threading.Event()
        self.event.set()
        self.proxies = MujocoServerProxies.default(SimpleNamespace(dict=dict))
        self.server = MujocoServer(str(scene) if scene else None, model, self.event,
                                   self.proxies, None, None)
        self.server.sensor_manager.sensors_thread.join(timeout=2)
        if self.server.sensor_manager.sensors_thread.is_alive():
            raise RuntimeError("Could not stop asynchronous sensor reader")
        self.event.clear()
        for sig, handler in self._signals.items():
            signal.signal(sig, handler)
        self.model, self.data = self.server.mjmodel, self.server.mjdata
        if not config.get("compute_mujoco_sensors", False):
            # The baseline uses RGB and qpos/actuator state, not the 360 lidar rays.
            self.model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_SENSOR)
        self.model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_AUTORESET)
        ratio = 1 / (control_hz * self.model.opt.timestep)
        if abs(ratio - round(ratio)) > 1e-6:
            raise ValueError("control period must be a multiple of the physics timestep")
        self.substeps = round(ratio)
        self.command = self.proxies.get_command()
        mujoco.mj_forward(self.model, self.data)
        self.server.pull_status()
        if cameras:
            self.camera_manager = MujocoServerCameraManagerSync(camera_hz, [
                StretchCameras.cam_d435i_rgb, StretchCameras.cam_d405_rgb,
            ], self.server)
        self._camera_time = -np.inf
        if gui:
            from mujoco import viewer as mj_viewer
            self.viewer = mj_viewer.launch_passive(self.model, self.data)
            self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
        self._wall_start = time.monotonic()

    def get_state(self):
        self.server.pull_status()
        status = asdict(self.proxies.get_status())
        return {"time": float(self.data.time),
                "base_pose": self.server.base_controller.get_base_pose().copy(),
                "joints": {name: float(status[name]["pos"]) for name in JOINTS + ("gripper",)},
                "joint_velocities": {name: float(status[name]["vel"]) for name in JOINTS},
                "base_velocity": np.array([status["base"]["x_vel"], status["base"]["theta_vel"]])}

    def _sample_cameras(self):
        if self.camera_manager and self.data.time - self._camera_time >= 1 / self.camera_hz - 1e-9:
            self.camera_manager._pull_camera_data()
            self._camera_time = float(self.data.time)

    def get_camera_image(self, camera="head"):
        if not self.camera_manager:
            raise RuntimeError("Cameras are disabled")
        self._sample_cameras()
        enum = {"head": StretchCameras.cam_d435i_rgb, "wrist": StretchCameras.cam_d405_rgb}[camera]
        return self.proxies.get_cameras().get_camera_data(
            enum, auto_rotate=False, auto_correct_rgb=False).copy()

    def get_observation(self, instruction=""):
        from perception.observation import Observation
        return Observation(instruction=instruction, state=self.get_state(),
                           images={name: self.get_camera_image(name) for name in ("head", "wrist")},
                           image_time=self._camera_time)

    def get_object_pose(self, body_name):
        """Privileged world pose for baseline goals/evaluation, excluded from Observation."""
        body = self.data.body(body_name)
        return {"position": body.xpos.copy(), "quaternion": body.xquat.copy()}

    def move_base(self, v, w):
        if not np.isfinite([v, w]).all() or np.any(np.abs([v, w]) > self.base_limits):
            raise ValueError(f"base velocity exceeds limits {self.base_limits}")
        self.command.set_base_velocity(CommandBaseVelocity(float(v), float(w), True))

    def set_joint_targets(self, **targets):
        for name, value in targets.items():
            if name not in JOINTS:
                raise ValueError(f"Unknown position actuator {name}")
            lo, hi = self.model.actuator(name).ctrlrange
            if not np.isfinite(value) or not lo <= value <= hi:
                raise ValueError(f"{name} must be in [{lo}, {hi}]")
        for name, value in targets.items():
            self.command.set_move_to(CommandMove(name, True, float(value)))

    def set_gripper(self, open=True, *, position=None):
        # Public normalized aperture. Official command uses its calibrated range.
        aperture = float(open) if position is None else float(position)
        if not np.isfinite(aperture) or not 0 <= aperture <= 1:
            raise ValueError("gripper aperture must be in [0,1]")
        self.command.set_move_to(CommandMove("gripper", True, -0.376 + aperture * 0.936))

    def move_joints(self, targets, duration=2.0):
        if not np.isfinite(duration) or duration <= 0:
            raise ValueError("duration must be positive")
        start = self.get_state()["joints"]
        # Validate before moving any joint.
        self.set_joint_targets(**targets)
        steps = max(1, int(np.ceil(duration * self.control_hz)))
        for i in range(1, steps + 1):
            t = i / steps
            blend = 3 * t * t - 2 * t * t * t
            self.set_joint_targets(**{k: start[k] + blend * (v - start[k]) for k, v in targets.items()})
            self.step()

    def move_to_pose(self, target_pose, duration=2.0):
        from manipulation.arm_controller import solve_ik
        targets = solve_ik(self, target_pose)
        self.move_joints(targets, duration)
        return targets

    def step(self, steps=1):
        for _ in range(steps):
            if self._closed or (self.viewer and not self.viewer.is_running()):
                raise InterruptedError("Simulation viewer closed")
            for _ in range(self.substeps):
                self.server.push_command(self.command)
                mujoco.mj_step(self.model, self.data)
            if not np.isfinite(self.data.qpos).all():
                raise RuntimeError("Non-finite simulation state")
            self.server.pull_status()
            if self._sample_cameras_on_step:
                self._sample_cameras()
            if self.viewer:
                self.viewer.sync()
            if self.realtime:
                remaining = self._wall_start + self.data.time - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)

    def hold(self, seconds):
        self.step(max(1, int(np.ceil(seconds * self.control_hz))))

    def close(self):
        if not self._closed:
            self.move_base(0, 0)
            self.server.push_command(self.command)
            self.event.set()
            if self.camera_manager:
                self.camera_manager.close()
            if self.viewer:
                self.viewer.close()
            self._closed = True

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

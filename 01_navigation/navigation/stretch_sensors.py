"""Synchronous planar sensor sampling for the pinned Stretch MuJoCo model."""

import mujoco
import numpy as np

from .mini_slam.core import LidarModel, SensorSample


class StretchSensors:
    """Read wheel encoders, the base gyro, and LiDAR at the navigation rate."""

    wheel_radius = 0.0508
    track = 0.3153  # calibrated separation used by the official wheel controller
    blind_radius = 0.05

    def __init__(self, model, data):
        self.model = model
        self.data = data
        self._wheels = tuple(int(model.joint(name).qposadr[0])
                             for name in ("joint_left_wheel", "joint_right_wheel"))
        self._base = model.body("base_link").id
        self._laser = model.body("laser").id
        gyro = model.sensor("base_gyro")
        if gyro.type[0] != mujoco.mjtSensor.mjSENS_GYRO:
            raise ValueError("Stretch base_gyro must be a gyro sensor")
        self._gyro = slice(int(gyro.adr[0]), int(gyro.adr[0]) + 3)
        gyro_site = int(gyro.objid[0])

        # Read the fixed mounts in the model's reference configuration.
        reference = mujoco.MjData(model)
        mujoco.mj_kinematics(model, reference)
        base_rot = reference.xmat[self._base].reshape(3, 3)
        origin = base_rot.T @ (reference.xpos[self._laser] - reference.xpos[self._base])
        self._gyro_axis = base_rot[:, 2] @ reference.site_xmat[gyro_site].reshape(3, 3)
        lidars = [model.sensor(i) for i, kind in enumerate(model.sensor_type)
                  if kind == mujoco.mjtSensor.mjSENS_RANGEFINDER
                  and model.sensor(i).name.startswith("base_lidar")]
        sites = [int(sensor.objid[0]) for sensor in lidars]
        if len(sites) != 360 or not np.allclose(reference.site_xpos[sites],
                                                reference.xpos[self._laser], atol=1e-6):
            raise ValueError("Stretch LiDAR must have 360 beams sharing the laser origin")
        directions = reference.site_xmat[sites].reshape(-1, 3, 3)[:, :, 2] @ base_rot
        if not np.allclose(directions[:, 2], 0.0, atol=1e-6):
            raise ValueError("Stretch LiDAR beams must lie in the base plane")
        self._directions = directions
        self.lidar = LidarModel(
            angles=tuple(np.arctan2(directions[:, 1], directions[:, 0])),
            blind_radius=self.blind_radius,
            max_range=min(float(sensor.cutoff[0]) for sensor in lidars),
            origin=(float(origin[0]), float(origin[1])),
        )
        robot_bodies = {self._base}
        for body in range(self._base + 1, model.nbody):
            if model.body_parentid[body] in robot_bodies:
                robot_bodies.add(body)
        self._robot_geoms = np.isin(model.geom_bodyid, list(robot_bodies))
        self._groups = np.ones(6, dtype=np.uint8)
        self._groups[2] = 0  # Stretch visual-only meshes
        self._geoms = np.empty(len(sites), dtype=np.int32)
        self._distances = np.empty(len(sites), dtype=np.float64)

    def sample(self):
        model, data = self.model, self.data
        # mj_step advances qpos/qvel after computing derived transforms. Refresh
        # only the needed stages so wheel, gyro and rays share the sampled time.
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        mujoco.mj_comVel(model, data)
        flags = model.opt.disableflags
        try:
            model.opt.disableflags = flags & ~int(mujoco.mjtDisableBit.mjDSBL_SENSOR)
            mujoco.mj_sensorVel(model, data)
        finally:
            model.opt.disableflags = flags
        gyro_z = float(self._gyro_axis @ data.sensordata[self._gyro])

        world_rays = self._directions @ data.xmat[self._base].reshape(3, 3).T
        # Batch-cast the official beam geometry with hit IDs so we can keep
        # chassis/arm occlusions unknown. Position sensors stay off in mj_step.
        mujoco.mj_multiRay(model, data, data.xpos[self._laser], world_rays.ravel(),
                           self._groups, 1, self._laser, self._geoms, self._distances,
                           len(self._distances), self.lidar.max_range)
        valid = ((self._geoms >= 0) & (self._distances >= self.lidar.blind_radius)
                 & (self._distances <= self.lidar.max_range)
                 & ~self._robot_geoms[self._geoms])
        ranges = tuple(float(distance) if hit else None
                       for distance, hit in zip(self._distances, valid))
        return SensorSample(tuple(float(data.qpos[index]) for index in self._wheels),
                            gyro_z, ranges, float(data.time))

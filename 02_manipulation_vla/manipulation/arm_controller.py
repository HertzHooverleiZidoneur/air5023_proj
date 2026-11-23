"""TEAM-02: bounded IK using MuJoCo FK and SciPy's least-squares solver.

Stretch's pinned simulator exposes actuators but no Cartesian arm IK API.
Solve only its five independent arm coordinates, preserving base and objects.
"""
import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ARM_JOINTS = ("lift", "arm", "wrist_yaw", "wrist_pitch", "wrist_roll")


def solve_ik(robot, target_pose):
    """target_pose = {position: xyz [m], quaternion: optional wxyz}, world frame."""
    xyz = np.asarray(target_pose["position"], dtype=float)
    if xyz.shape != (3,) or not np.isfinite(xyz).all():
        raise ValueError("position must be a finite xyz vector")
    quaternion = target_pose.get("quaternion")
    target_rotation = None
    if quaternion is not None:
        q = np.asarray(quaternion, dtype=float)
        if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
            raise ValueError("quaternion must be finite nonzero wxyz")
        target_rotation = Rotation.from_quat(q[[1, 2, 3, 0]])
    model = robot.model
    data = mujoco.MjData(model)
    data.qpos[:] = robot.data.qpos
    start = np.array([robot.get_state()["joints"][name] for name in ARM_JOINTS])
    ranges = np.array([model.actuator(name).ctrlrange for name in ARM_JOINTS])

    def residual(q):
        for name, value in zip(ARM_JOINTS, q):
            if name == "arm":
                for j in range(4):
                    data.joint(f"joint_arm_l{j}").qpos[0] = value / 4
            else:
                data.joint("joint_" + name).qpos[0] = value
        mujoco.mj_kinematics(model, data)
        body = data.body("link_grasp_center")
        position_error = body.xpos - xyz
        if target_rotation is None:
            return np.r_[position_error, 0.002 * (q - start)]
        rotation = Rotation.from_matrix(body.xmat.reshape(3, 3))
        return np.r_[position_error, 0.15 * (target_rotation.inv() * rotation).as_rotvec()]

    result = least_squares(residual, np.clip(start, ranges[:, 0], ranges[:, 1]),
                           bounds=(ranges[:, 0], ranges[:, 1]), max_nfev=150,
                           ftol=1e-9, xtol=1e-9, gtol=1e-9)
    error = residual(result.x)
    if np.linalg.norm(error[:3]) > 0.01 or (target_rotation and np.linalg.norm(error[3:]) > 0.03):
        raise ValueError(f"Pose unreachable from this base pose; position error {np.linalg.norm(error[:3]):.3f} m")
    return dict(zip(ARM_JOINTS, result.x.tolist()))

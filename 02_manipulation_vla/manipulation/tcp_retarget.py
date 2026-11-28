"""Track one G1 TCP replay with Stretch position IK in the round-table scene."""

from dataclasses import dataclass
import json

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.signal import savgol_filter

from evaluation.round_table import RoundTableContacts, SETTLE_SECONDS, TABLETOP, summarize
from manipulation.arm_controller import ARM_JOINTS


@dataclass(frozen=True)
class TcpReplay:
    times: np.ndarray
    relative_xyz: np.ndarray
    close_time: float
    open_time: float

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as source:
            times, tcp, rotation, commands = (source[key] for key in ("time", "tcp", "rotation_xy", "close"))
        if (times.ndim != 1 or len(times) < 9 or tcp.shape != (len(times), 3)
                or commands.shape != times.shape or rotation.shape != (2, 2)):
            raise ValueError("Invalid TCP replay array shapes")
        if (not all(np.isfinite(array).all() for array in (times, tcp, rotation))
                or times[0] != 0 or not (np.diff(times) > 0).all()):
            raise ValueError("TCP replay requires finite data and increasing times starting at zero")
        if not np.allclose(rotation.T @ rotation, np.eye(2)) or not np.isclose(np.linalg.det(rotation), 1):
            raise ValueError("rotation_xy must be a proper planar rotation")
        if not np.isin(commands, (0, 1)).all() or commands[0] != 0 or commands[-1] != 0:
            raise ValueError("Expected binary gripper commands beginning and ending open")
        edges = np.flatnonzero(np.diff(commands)) + 1
        if len(edges) != 2 or not 0 < times[edges[0]] < times[edges[1]] < times[-1]:
            raise ValueError("Expected one close event followed by one open event before the replay ends")
        relative = savgol_filter(tcp, 9, 2, axis=0)
        relative[:, :2] = relative[:, :2] @ rotation.T
        relative -= relative[edges[0]].copy()
        return cls(times, relative, float(times[edges[0]]), float(times[edges[1]]))


def restore_dock(robot, dock_path):
    """Restore the fixed example's initial robot pose using model names."""
    dock = json.loads(dock_path.read_text())
    model, data = robot.model, robot.data
    base_joint = int(model.body("base_link").jntadr[0])
    if model.jnt_type[base_joint] != mujoco.mjtJoint.mjJNT_FREE:
        raise ValueError("The round-table dock requires a free base_joint")
    base_qpos = np.asarray(dock["base_xyz_quaternion_wxyz"], dtype=float)
    if (base_qpos.shape != (7,) or not np.isfinite(base_qpos).all()
            or not np.isclose(np.linalg.norm(base_qpos[3:]), 1.0)):
        raise ValueError("Invalid dock base pose")
    joint_positions = [(data.joint(name).qpos, float(value)) for name, value in dock["joint_positions"].items()]
    controls = [(model.actuator(name).id, float(value)) for name, value in dock["actuator_ctrl"].items()]
    if any(qpos.shape != (1,) or not np.isfinite(value) for qpos, value in joint_positions):
        raise ValueError("Expected finite single-DOF dock joint positions")
    if any(not np.isfinite(value) for _, value in controls):
        raise ValueError("Expected finite dock actuator targets")
    start = model.jnt_qposadr[base_joint]
    data.qpos[start:start + 7] = base_qpos
    for qpos, value in joint_positions:
        qpos[0] = value
    for actuator, value in controls:
        data.ctrl[actuator] = value
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)


class _PositionTracker:
    def __init__(self, robot):
        self.robot = robot
        model = robot.model
        self.kinematics = mujoco.MjData(model)
        self.bounds = np.array([model.actuator(name).ctrlrange for name in ARM_JOINTS]).T
        self.positions = {name: self.kinematics.joint(f"joint_{name}").qpos
                          for name in ARM_JOINTS if name != "arm"}
        self.extension = [self.kinematics.joint(f"joint_arm_l{i}").qpos for i in range(4)]
        self.tcp = self.kinematics.body("link_grasp_center")
        joints = robot.get_state()["joints"]
        self.last = np.array([joints[name] for name in ARM_JOINTS])
        self.max_error = 0.0

    def command(self, target):
        self.kinematics.qpos[:] = self.robot.data.qpos

        def residual(q):
            for name, value in zip(ARM_JOINTS, q):
                if name == "arm":
                    for section in self.extension:
                        section[0] = value / 4
                else:
                    self.positions[name][0] = value
            mujoco.mj_kinematics(self.robot.model, self.kinematics)
            # Preserve continuity and prefer neutral wrist pitch and roll.
            return np.r_[self.tcp.xpos - target, 0.003 * q[3:5], 0.001 * (q - self.last)]

        lo, hi = self.bounds
        result = least_squares(residual, np.clip(self.last, lo, hi), bounds=(lo, hi),
                               ftol=2e-6, xtol=2e-6, gtol=2e-6, max_nfev=32)
        error = float(np.linalg.norm(residual(result.x)[:3]))
        if error > 0.013:
            raise ValueError(f"TCP goal {target.tolist()} is unreachable (error {error:.3f} m)")
        self.max_error = max(self.max_error, error)
        self.last = result.x
        self.robot.set_joint_targets(**dict(zip(ARM_JOINTS, self.last.tolist())))


def _ease(fraction):
    return 3 * fraction * fraction - 2 * fraction * fraction * fraction


def _interpolate(start, end):
    return lambda fraction: start + (end - start) * _ease(fraction)


def run_retarget(robot, replay, *, support_projection=True, release_adapt=True, on_sample=None):
    """Operate on an existing simulator at the example's round-table dock."""
    robot.move_base(0, 0)
    robot.set_gripper(open=True)
    joints = robot.get_state()["joints"]
    robot.move_joints({name: value for name, value in joints.items() if name != "gripper"}, 0.25)
    robot.hold(0.75)
    initial_bottle = robot.get_object_pose("g1_bottle")["position"]
    contact = RoundTableContacts(robot)
    tracker = _PositionTracker(robot)
    tcp = robot.data.body("link_grasp_center")
    trace = []
    aperture = 1.0

    def phase(name, duration, target=None, *, closing=None):
        nonlocal aperture
        if closing is not None:
            aperture = float(not closing)
            robot.set_gripper(open=not closing)
        steps = max(1, round(duration * robot.control_hz))
        for i in range(1, steps + 1):
            goal = target(i / steps) if target is not None else None
            if goal is not None:
                tracker.command(goal)
            robot.step()
            sample = contact.sample()
            state = robot.get_state()
            sample.update(phase=name, goal=None if goal is None else goal.tolist(), tcp=tcp.xpos.tolist(),
                          gripper_command=aperture, target_q=tracker.last.tolist(),
                          actual_q=[state["joints"][name] for name in ARM_JOINTS],
                          base_pose=state["base_pose"].tolist())
            trace.append(sample)
            if on_sample is not None:
                on_sample(sample)

    # Align the source close point to the bottle at startup using simulator truth.
    grab = initial_bottle + [0, -0.029, 0.098]
    release_z = np.interp(replay.open_time, replay.times, replay.relative_xyz[:, 2])

    def goal_at(source_time):
        delta = np.array([np.interp(source_time, replay.times, dim) for dim in replay.relative_xyz.T])
        if support_projection:
            # Ease in a vertical correction during the descent to the tabletop.
            weight = _ease(np.clip((source_time - 9.55) / 0.85, 0, 1))
            delta[2] -= weight * release_z
        return grab + delta

    start = tcp.xpos.copy()
    first_goal = goal_at(0.0)
    above = start.copy()
    above[2] = max(first_goal[2], start[2])
    prealign = first_goal + [0, 0.105, 0.008]
    phase("reach_height", 1.4, _interpolate(start, above), closing=False)
    phase("reach_prealign", 1.6, _interpolate(above, prealign))
    phase("reach_source_start", 1.8, _interpolate(prealign, first_goal))
    phase("source_open", replay.close_time, lambda u: goal_at(u * replay.close_time), closing=False)
    clamped_duration = replay.open_time - replay.close_time
    phase("source_clamped", clamped_duration, lambda u: goal_at(replay.close_time + u * clamped_duration), closing=True)

    supported = any(TABLETOP in sample["contacts"] for sample in trace[-4:])
    release_clearance = float(trace[-1]["pos"][2] - initial_bottle[2])
    retreat_duration = replay.times[-1] - replay.open_time
    release = goal_at(replay.open_time)
    if release_adapt:
        if not supported:
            raise RuntimeError("No tabletop support before release; stopping with the gripper closed")
        phase("open_pause", 0.55, lambda _: release, closing=False)
        away = release + [0, 0.11, 0.012]
        phase("gripper_clearance", 1.0, _interpolate(release, away))

        def retreat(fraction):
            target = goal_at(replay.open_time + fraction * retreat_duration)
            target[1:] = np.maximum(target[1:], away[1:])
            return away + (target - away) * _ease(min(1.0, fraction / 0.18))

        phase("source_release_retreat", retreat_duration, retreat)
    else:
        phase("source_release_retreat", retreat_duration,
              lambda u: goal_at(replay.open_time + u * retreat_duration), closing=False)
    phase("settle", SETTLE_SECONDS)

    result = summarize(trace, initial_bottle, robot.control_hz, supported)
    result.update(control_hz=robot.control_hz, physics_hz=1 / robot.model.opt.timestep,
                  ik_position_max_m=tracker.max_error, source_command_edges_s=[replay.close_time, replay.open_time],
                  support_projection=support_projection, release_adapt=release_adapt,
                  release_height_above_initial_m=release_clearance,
                  controller="g1_tcp_position_retarget", target_alignment="simulator_ground_truth",
                  vla_loaded=False)
    return result

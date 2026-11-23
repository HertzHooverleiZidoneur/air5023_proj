"""TEAM-02: explicit Stretch action contract, validation and execution."""
from dataclasses import dataclass, field
import numpy as np
from robot.robot_interface import JOINTS


@dataclass(frozen=True)
class Action:
    # Absolute joint setpoints: lift/total arm extension in metres, others radians.
    joint_targets: dict[str, float] = field(default_factory=dict)
    base_velocity: tuple[float, float] = (0.0, 0.0)  # body v [m/s], yaw w [rad/s]
    gripper: float | None = None  # normalized aperture, 0 closed / 1 open
    embodiment: str = "stretch3"
    schema_version: str = "1.0"


def validate_action(robot, action):
    if not isinstance(action, Action) or action.embodiment != "stretch3" or action.schema_version != "1.0":
        raise ValueError("Require Action schema 1.0 for stretch3; model-specific vectors need an explicit adapter")
    velocity = np.asarray(action.base_velocity, dtype=float)
    if velocity.shape != (2,) or not np.isfinite(velocity).all() or np.any(abs(velocity) > robot.base_limits):
        raise ValueError("Invalid base velocity")
    for name, target in action.joint_targets.items():
        if name not in JOINTS:
            raise ValueError(f"Unknown joint {name}")
        lo, hi = robot.model.actuator(name).ctrlrange
        if not np.isfinite(target) or not lo <= target <= hi:
            raise ValueError(f"Out-of-range joint target {name}")
    if action.gripper is not None and (not np.isfinite(action.gripper) or not 0 <= action.gripper <= 1):
        raise ValueError("Invalid gripper aperture")


def apply_action(robot, action):
    validate_action(robot, action)
    robot.move_base(*action.base_velocity)
    robot.set_joint_targets(**action.joint_targets)
    if action.gripper is not None:
        robot.set_gripper(position=action.gripper)

"""TEAM-02: serializable metadata plus explicit NumPy image/state payloads.

Model preprocessing, normalization and action denormalization belong in a
checkpoint-specific adapter. This module does not assume DROID/GR00T semantics.
"""
import numpy as np
from robot.robot_interface import JOINTS

STATE_ORDER = ("base_x", "base_y", "base_yaw") + JOINTS + ("gripper",)


def to_policy_input(observation):
    state = observation.state
    return {
        "schema_version": observation.schema_version,
        "embodiment": observation.embodiment,
        "instruction": observation.instruction,
        "state": np.array([*state["base_pose"], *(state["joints"][n] for n in JOINTS),
                           (state["joints"]["gripper"] + 0.376) / 0.936], dtype=np.float32),
        "state_order": STATE_ORDER,
        "state_time": state["time"],
        "image_time": observation.image_time,
        "images": {k: np.ascontiguousarray(v) for k, v in observation.images.items()},
        "image_format": "RGB uint8 HWC, native camera orientation",
        "frame": "base pose in MuJoCo world (Z up), base velocity in robot frame",
    }

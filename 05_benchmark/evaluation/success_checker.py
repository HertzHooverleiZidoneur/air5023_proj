"""TEAM-05: lift/hold metric using robosuite's contact group checker."""
from types import SimpleNamespace
import numpy as np
from robosuite.utils.sim_utils import check_contact
from .criteria import HoldCheck


def descendant_geoms(model, body_name):
    ancestor = model.body(body_name).id
    result = []
    for geom in range(model.ngeom):
        body = int(model.geom_bodyid[geom])
        while body and body != ancestor:
            body = int(model.body_parentid[body])
        if body == ancestor:
            result.append(geom)
    return result


class SuccessChecker:
    def __init__(self, robot, target_body="red_cup_main", lift_height=0.05, hold_seconds=1.0):
        self.robot, self.target_body = robot, target_body
        self.initial_height = float(robot.data.body(target_body).xpos[2])
        self.lift_height, self.hold_seconds = lift_height, hold_seconds
        hz = getattr(robot, "control_hz", None)
        self.check = HoldCheck(lift_height, hold_seconds, 1.5 / hz if hz else None)
        if not np.isfinite(self.initial_height):
            raise ValueError("Target initial height must be finite")
        self.last = {}
        model = robot.model
        # The official Stretch model has unnamed collision geoms. Supply stable
        # identifiers to robosuite's public contact checker through its sim API.
        names = [model.geom(i).name or f"geom:{i}" for i in range(model.ngeom)]
        self.sim = SimpleNamespace(data=robot.data, model=SimpleNamespace(geom_id2name=lambda i: names[i]))
        group = lambda body: [names[i] for i in descendant_geoms(model, body)]
        self.target = group(target_body)
        self.left = group("link_gripper_finger_left")
        self.right = group("link_gripper_finger_right")
        if not self.target or not self.left or not self.right:
            raise ValueError("Target and both fingers must have geometry")
        excluded = set(group("base_link") + self.target)
        self.environment = [name for name in names if name not in excluded]

    def update(self):
        data = self.robot.data
        left = check_contact(self.sim, self.left, self.target)
        right = check_contact(self.sim, self.right, self.target)
        supported = check_contact(self.sim, self.target, self.environment)
        height = float(data.body(self.target_body).xpos[2])
        self.last = self.check.update(
            time=float(data.time), lift=height - self.initial_height,
            left=left, right=right, supported=supported,
            finite=bool(np.isfinite(data.qpos).all() and np.isfinite(getattr(data, "qvel", [])).all()))
        return self.last

    @property
    def held_since(self):
        return self.check.held_since

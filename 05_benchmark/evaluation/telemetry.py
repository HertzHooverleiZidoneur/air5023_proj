"""Control-tick navigation metrics, independent of policy and simulator imports."""
import math


class Telemetry:
    def __init__(self):
        self.phase = "setup"
        self.events = []
        self.previous = None
        self.active_contacts = set()
        self.samples = self.navigation_samples = 0
        self.navigation_seconds = self.navigation_distance_metres = 0.0
        self.navigation_collision_events = self.navigation_contact_samples = 0
        self.navigation_contacts = set()
        self.navigation_final_pose = None
        self.navigation_finished = False
        self.started = self.ended = None
        self.operation_contacts = set()
        self.operation_active = set()
        self.operation_events = self.operation_samples = 0
        self.all_contacts = set()
        self.all_active = set()
        self.all_events = 0

    def set_phase(self, name, time):
        if not isinstance(name, str) or not name or not math.isfinite(time):
            raise ValueError("Phase needs a name and finite simulation time")
        if self.phase == "navigate" and name != "navigate":
            self.navigation_finished = True
        if name == "navigate":
            self.navigation_finished = False
        self.phase = name
        self.events.append({"phase": name, "simulation_time": time})

    def sample(self, time, pose, contacts=(), operation_contacts=None):
        pose = list(map(float, pose))
        if len(pose) != 3 or not all(math.isfinite(value) for value in [time, *pose]):
            raise ValueError("Non-finite time or invalid base pose")
        if self.previous and time < self.previous[0]:
            raise ValueError("Simulation time moved backwards")
        pairs = {tuple(sorted(pair)) for pair in contacts}
        self.all_events += len(pairs - self.all_active)
        self.all_active = pairs
        self.all_contacts.update(pairs)
        operation = {tuple(sorted(pair)) for pair in (contacts if operation_contacts is None else operation_contacts)}
        if self.phase not in {"setup", "initialize", "navigate", "plan_dock", "stopping"}:
            self.operation_events += len(operation - self.operation_active)
            self.operation_samples += bool(operation)
            self.operation_contacts.update(operation)
            self.operation_active = operation
        else:
            self.operation_active = set()
        if self.phase == "navigate":
            self.navigation_samples += 1
            if self.previous:
                previous_time, previous_pose = self.previous
                self.navigation_seconds += time - previous_time
                self.navigation_distance_metres += math.dist(pose[:2], previous_pose[:2])
            self.navigation_collision_events += len(pairs - self.active_contacts)
            self.navigation_contact_samples += bool(pairs)
            self.navigation_contacts.update(pairs)
            self.navigation_final_pose = pose
        self.active_contacts = pairs if self.phase == "navigate" else set()
        self.previous = (time, pose)
        self.samples += 1
        if self.started is None:
            self.started = time
        self.ended = time
        return {"simulation_time": time, "phase": self.phase, "base_pose": pose,
                "robot_environment_contacts": [list(pair) for pair in sorted(pairs)]}

    def result(self, goal, criteria):
        pos_error = yaw_error = None
        if goal is not None and self.navigation_final_pose is not None:
            pose = self.navigation_final_pose
            pos_error = math.dist(pose[:2], goal[:2])
            yaw_error = abs((pose[2] - goal[2] + math.pi) % (2 * math.pi) - math.pi)
        navigation_success = bool(self.navigation_finished and pos_error is not None and
                                  pos_error <= criteria.navigation_position_tolerance_metres and
                                  yaw_error <= criteria.navigation_heading_tolerance_radians)
        return {"sample_count": self.samples, "navigation_sample_count": self.navigation_samples,
                "simulation_seconds": None if self.started is None else self.ended - self.started,
                "navigation_seconds": self.navigation_seconds if self.navigation_samples else None,
                "navigation_distance_metres": self.navigation_distance_metres if self.navigation_samples else None,
                "navigation_position_error_metres": pos_error, "navigation_heading_error_radians": yaw_error,
                "navigation_success": navigation_success,
                "navigation_collision_events": self.navigation_collision_events,
                "navigation_contact_samples": self.navigation_contact_samples,
                "navigation_contact_pairs": [list(pair) for pair in sorted(self.navigation_contacts)],
                "operation_collision_events": self.operation_events,
                "operation_contact_samples": self.operation_samples,
                "operation_contact_pairs": [list(pair) for pair in sorted(self.operation_contacts)],
                "operation_collision_policy": "report_only; target contact excluded",
                "all_robot_environment_collision_events": self.all_events,
                "all_robot_environment_contact_pairs": [list(pair) for pair in sorted(self.all_contacts)],
                "collision_sampling": "control_ticks; substep-only contacts may be missed",
                "last_phase": self.phase, "phases": self.events}


def failure_reason(metrics):
    if metrics.get("task_success"):
        return None
    if not metrics.get("navigation_success"):
        return "navigation_failed"
    if not metrics.get("collision_free"):
        return "navigation_collision"
    grasp = metrics.get("grasp") or {}
    if not grasp.get("state_finite", True):
        return "non_finite_state"
    if grasp.get("environment_contact"):
        return "object_supported"
    if not (grasp.get("left_contact") and grasp.get("right_contact")):
        return "dropped_after_grasp" if metrics.get("ever_grasp_success") else "missing_finger_contact"
    if (grasp.get("lift_metres") or 0) < metrics["criteria"]["minimum_lift_metres"]:
        return "insufficient_lift"
    return "insufficient_hold"

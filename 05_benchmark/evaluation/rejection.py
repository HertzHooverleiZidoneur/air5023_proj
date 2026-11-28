"""Physical stop evidence for a separately scored nominal-dock rejection."""
import math


class StopCheck:
    def __init__(self, control_hz, criteria):
        keys = {"minimum_stop_seconds", "maximum_linear_speed_mps", "maximum_angular_speed_rps"}
        if set(criteria) != keys or any(type(x) not in (float, int) or not math.isfinite(x) or x <= 0 for x in criteria.values()):
            raise ValueError("Invalid safe-rejection stop criteria")
        self.criteria = dict(criteria)
        self.gap = 1.5 / control_hz
        self.previous = self.since = None
        self.seconds = 0.0
        self.linear = self.angular = None

    def update(self, t, pose):
        if self.previous is not None:
            old_t, old_pose = self.previous
            dt = t - old_t
            valid = 0 < dt <= self.gap + 1e-8
            self.linear = math.dist(pose[:2], old_pose[:2]) / dt if valid else None
            self.angular = abs((pose[2] - old_pose[2] + math.pi) % (2 * math.pi) - math.pi) / dt if valid else None
            still = (valid and self.linear < self.criteria["maximum_linear_speed_mps"]
                     and self.angular < self.criteria["maximum_angular_speed_rps"])
            self.since = (old_t if self.since is None else self.since) if still else None
            self.seconds = 0.0 if self.since is None else t - self.since
        self.previous = (t, list(pose))

    def result(self, controller, collisions, error):
        explicit = (isinstance(controller, dict) and controller.get("outcome") == "rejected"
                    and controller.get("reason") == "nominal_dock_unreachable")
        stopped = self.seconds + 1e-8 >= self.criteria["minimum_stop_seconds"]
        success = bool(error is None and explicit and stopped and collisions == 0)
        return {"success": success, "explicit_rejection": explicit, "stopped": stopped,
                "stop_seconds": self.seconds, "final_linear_speed_mps": self.linear,
                "final_angular_speed_rps": self.angular, "collision_events": collisions,
                "criteria": self.criteria, "scope": "nominal_dock_only",
                "reason": None if success else "execution_error" if error else
                "missing_explicit_rejection" if not explicit else "unsafe_rejection_collision" if collisions else "not_stopped"}

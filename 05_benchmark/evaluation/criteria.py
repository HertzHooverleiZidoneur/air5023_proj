"""Shared strict criteria and simulator-independent continuous hold checking."""
from dataclasses import asdict, dataclass
import math
from pathlib import Path


@dataclass(frozen=True)
class Criteria:
    minimum_lift_metres: float = 0.05
    minimum_hold_seconds: float = 1.0
    navigation_position_tolerance_metres: float = 0.05
    navigation_heading_tolerance_radians: float = math.pi / 36
    maximum_navigation_collision_events: int = 0

    def __post_init__(self):
        for name, value in asdict(self).items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{name} must be finite and numeric")
            if name == "maximum_navigation_collision_events":
                if not isinstance(value, int) or value < 0:
                    raise ValueError(f"{name} must be a nonnegative integer")
            elif value <= 0:
                raise ValueError(f"{name} must be positive")

    def to_dict(self):
        return {**asdict(self), "require_both_fingers": True, "require_no_environment_contact": True}


def load_criteria(path=None):
    import yaml

    path = Path(path) if path else Path(__file__).resolve().parents[2] / "configs/task.yaml"
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("Task config must be a mapping")
    for name in ("require_both_fingers", "require_no_environment_contact"):
        if config.get(name, True) is not True:
            raise ValueError(f"{name} must remain true for all benchmark policies")
    for name in ("minimum_lift_metres", "minimum_hold_seconds"):
        if name not in config:
            raise ValueError(f"Task config is missing {name}")
    return Criteria(**{name: config[name] for name in Criteria.__dataclass_fields__ if name in config})


class HoldCheck:
    """Success is reset after a drop, a backwards clock or an unobserved gap."""

    def __init__(self, lift_height=0.05, hold_seconds=1.0, max_sample_gap=None):
        Criteria(minimum_lift_metres=lift_height, minimum_hold_seconds=hold_seconds)
        if max_sample_gap is not None and (not math.isfinite(max_sample_gap) or max_sample_gap <= 0):
            raise ValueError("max_sample_gap must be positive and finite")
        self.lift_height, self.hold_seconds = lift_height, hold_seconds
        self.max_sample_gap = max_sample_gap
        self.held_since = self.previous_time = None

    def update(self, *, time, lift, left, right, supported, finite=True):
        valid_numbers = math.isfinite(time) and math.isfinite(lift)
        continuous = (self.previous_time is None or
                      (time >= self.previous_time and
                       (self.max_sample_gap is None or time - self.previous_time <= self.max_sample_gap + 1e-8)))
        valid = valid_numbers and finite and left and right and not supported and lift >= self.lift_height
        if not valid or not continuous:
            self.held_since = None
        if valid and self.held_since is None:
            self.held_since = time
        self.previous_time = time if math.isfinite(time) else None
        held = 0.0 if self.held_since is None else time - self.held_since
        return {"success": bool(valid and held + 1e-8 >= self.hold_seconds),
                "left_contact": bool(left), "right_contact": bool(right),
                "environment_contact": bool(supported), "state_finite": bool(finite and valid_numbers),
                "lift_metres": lift if math.isfinite(lift) else None,
                "held_seconds": held, "simulation_time": time if math.isfinite(time) else None}

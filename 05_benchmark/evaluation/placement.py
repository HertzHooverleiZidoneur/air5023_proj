"""Independent timestamp-based pick/place judge; no controller success input."""
from collections import deque
from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class PlacementCriteria:
    minimum_lift_metres: float = 0.05
    minimum_hold_seconds: float = 1.0
    minimum_stable_seconds: float = 1.5
    maximum_tilt_degrees: float = 6.0
    maximum_speed_mps: float = 0.025
    force_threshold_n: float = 0.01

    def to_dict(self):
        return asdict(self)


class PlacementCheck:
    def __init__(self, control_hz, criteria=None):
        if not math.isfinite(control_hz) or control_hz <= 0:
            raise ValueError("control_hz must be finite and positive")
        self.criteria = criteria or PlacementCriteria()
        self.gap = 1.5 / control_hz
        self.support_window = 4 / control_hz
        self.initial = None
        self.previous = self.held_since = self.stable_since = None
        self.longest_hold = self.stable_seconds = self.peak_lift = 0.0
        self.recent = deque()
        self.closed = False
        self.release_seen = self.support_before_release = False
        self.invalid = False
        self.last = None

    def start(self, position):
        if len(position) != 3 or not all(math.isfinite(x) for x in position):
            raise ValueError("Invalid initial target position")
        self.initial = list(position)

    def gripper_command(self, opening, time):
        if opening and self.closed:
            self.release_seen = True
            self.support_before_release = any(
                0 <= time - t <= self.support_window + 1e-8 and supported for t, supported in self.recent)
            self.stable_since = None
            self.stable_seconds = 0.0
        self.closed = not opening

    def update(self, sample, *, finite=True):
        if self.initial is None:
            return
        t, pos = sample["t"], sample["pos"]
        valid = finite and all(math.isfinite(x) for x in [t, *pos, sample["tilt_deg"], sample["speed"]])
        continuous = self.previous is None or 0 < t - self.previous <= self.gap + 1e-8
        if not valid or not continuous:
            self.held_since = self.stable_since = None
        if not valid or self.previous is not None and t <= self.previous:
            self.invalid = True
        self.previous = t if math.isfinite(t) else None
        if not valid:
            self.stable_seconds = 0.0
            return
        self.last = sample
        lift = pos[2] - self.initial[2]
        self.peak_lift = max(self.peak_lift, lift)
        held = sample["left"] and sample["right"] and not sample["support"] and lift >= self.criteria.minimum_lift_metres
        self.held_since = (t if self.held_since is None else self.held_since) if held else None
        if self.held_since is not None:
            self.longest_hold = max(self.longest_hold, t - self.held_since)
        supported = sample["support"] and "g1_round_table_support" in sample["contacts"]
        self.recent.append((t, bool(supported)))
        while self.recent and t - self.recent[0][0] > self.support_window + 1e-8:
            self.recent.popleft()
        stable = (self.release_seen and not self.closed and sample["table_only"]
                  and not sample["left"] and not sample["right"]
                  and sample["tilt_deg"] < self.criteria.maximum_tilt_degrees
                  and sample["speed"] < self.criteria.maximum_speed_mps)
        self.stable_since = (t if self.stable_since is None else self.stable_since) if stable else None
        self.stable_seconds = 0.0 if self.stable_since is None else t - self.stable_since

    def result(self):
        grasp = self.longest_hold + 1e-8 >= self.criteria.minimum_hold_seconds
        placed = (not self.invalid and self.release_seen and self.support_before_release
                  and self.stable_seconds + 1e-8 >= self.criteria.minimum_stable_seconds)
        success = bool(grasp and placed)
        reason = (None if success else "non_finite_or_invalid_state" if self.invalid else
                  "insufficient_pick_hold" if not grasp else "release_not_observed" if not self.release_seen else
                  "unsupported_release" if not self.support_before_release else "unstable_placement")
        return {"success": success, "grasp_success": bool(grasp), "placement_success": bool(placed),
                "reason": reason, "peak_lift_m": self.peak_lift,
                "both_pads_unsupported_lift_seconds": self.longest_hold,
                "stable_table_only_seconds": self.stable_seconds,
                "release_observed": self.release_seen, "closed_support_before_release": self.support_before_release,
                "place_xy_shift_m": math.dist(self.last["pos"][:2], self.initial[:2]) if self.last else None,
                "final_tilt_deg": self.last["tilt_deg"] if self.last else None,
                "final_speed_mps": self.last["speed"] if self.last else None,
                "criteria": self.criteria.to_dict(), "placement_region": "tabletop_anywhere",
                "time_measurement": "elapsed_simulation_timestamps; no inferred first interval"}

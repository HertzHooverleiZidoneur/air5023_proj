"""Stretch Mini SLAM pose feedback and occupancy map persistence."""

from __future__ import annotations

import json
from pathlib import Path

from .mini_slam.core import MiniSLAM, OccupancyGrid, Pose as SlamPose
from .planning import StaticGrid
from .stretch_sensors import StretchSensors
from .tracking import Pose


class StretchSLAM:
    def __init__(self, robot, initial_pose, *, reference_grid=None):
        self.sensors = StretchSensors(robot.model, robot.data)
        reference = (None if reference_grid is None else
                     OccupancyGrid(reference_grid.resolution, dict(reference_grid.cells)))
        self.estimator = MiniSLAM(
            self.sensors.lidar, wheel_radius=self.sensors.wheel_radius,
            track=self.sensors.track, initial_pose=SlamPose(*initial_pose),
            reference_grid=reference,
        )
        self.data = robot.data
        self.latest_sample = None

    def read_pose(self):
        now = float(self.data.time)
        previous = self.latest_sample
        if previous is not None and now < previous.sim_time:
            raise ValueError("Simulation time rewound during SLAM navigation")
        if previous is None or now != previous.sim_time:
            sample = self.sensors.sample()
            self.estimator.update(sample)
            self.latest_sample = sample
        pose = self.estimator.pose
        return Pose(pose.x, pose.y, pose.yaw)


def load_map(path: Path) -> StaticGrid:
    """Load the Mini SLAM grids.json convention in navigation world coordinates."""
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        cells = {}
        for x, y, log_odds in payload["slam"]:
            if type(x) is not int or type(y) is not int or (x, y) in cells:
                raise ValueError("Expected unique integer cell indices")
            cells[x, y] = float(log_odds)
        return StaticGrid(float(payload["resolution_m"]), cells)
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"Invalid Mini SLAM map {path}: {error}") from error


def save_map(grid: OccupancyGrid, path: Path) -> None:
    if not grid.cells:
        raise ValueError("No measured cells to save")
    payload = {"resolution_m": grid.resolution,
               "slam": [[x, y, odds] for (x, y), odds in sorted(grid.cells.items())]}
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as handle:
        json.dump(payload, handle, separators=(",", ":"), allow_nan=False)
        handle.write("\n")

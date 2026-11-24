"""Minimal planar wheel/gyro/lidar SLAM."""
from __future__ import annotations

from dataclasses import dataclass, field
import math

from .matching import match_scan
from .geometry import WHEEL_RADIUS, TRACK


@dataclass(frozen=True)
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def transform(self, point):
        """Transform an (x,y) robot-frame point into world coordinates."""
        px, py = point
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c*px - s*py, self.y + s*px + c*py


@dataclass(frozen=True)
class SensorSample:
    wheel_pos: tuple[float, float]
    gyro_z: float
    ranges: tuple[float | None, ...]
    sim_time: float


@dataclass(frozen=True)
class LidarModel:
    angles: tuple[float, ...]
    blind_radius: float
    max_range: float
    origin: tuple[float, float] = (0.0, 0.0)

    def __post_init__(self):
        if not self.angles or not all(math.isfinite(angle) for angle in self.angles):
            raise ValueError("Lidar beam angles must be non-empty and finite")
        if not 0 <= self.blind_radius < self.max_range < math.inf:
            raise ValueError("Invalid lidar range limits")
        if len(self.origin) != 2 or not all(math.isfinite(v) for v in self.origin):
            raise ValueError("Lidar origin must be a finite base-frame x,y pair")


@dataclass
class OccupancyGrid:
    resolution: float = 0.05
    cells: dict[tuple[int, int], float] = field(default_factory=dict)
    max_abs_log_odds: float = 5.0

    def index(self, x, y):
        return math.floor(x / self.resolution), math.floor(y / self.resolution)

    def update_ray(self, start, end):
        """Mark traversed cells free and the measured endpoint occupied."""
        sx, sy = self.index(*start)
        ex, ey = self.index(*end)
        dx, dy = abs(ex-sx), abs(ey-sy)
        stepx = 1 if sx < ex else -1
        stepy = 1 if sy < ey else -1
        err = dx - dy
        x, y = sx, sy
        while (x, y) != (ex, ey):
            self._add((x, y), -0.35)
            e2 = 2*err
            if e2 > -dy:
                err -= dy
                x += stepx
            if e2 < dx:
                err += dx
                y += stepy
        self._add((ex, ey), 0.85)

    def _add(self, key, amount):
        self.cells[key] = max(-self.max_abs_log_odds,
                              min(self.max_abs_log_odds, self.cells.get(key, 0.0)+amount))


class MiniSLAM:
    WHEEL_RADIUS = WHEEL_RADIUS
    TRACK = TRACK

    def __init__(self, lidar: LidarModel, *, resolution: float | None = None,
                 gyro_bias: float = 0.0, gyro_weight: float = 0.85,
                 wheel_radius: float = WHEEL_RADIUS, track: float = TRACK,
                 reference_grid: OccupancyGrid | None = None, initial_pose: Pose = Pose()):
        """Build a new map or localize from a known pose in a fixed reference grid."""
        if resolution is None:
            resolution = reference_grid.resolution if reference_grid is not None else 0.05
        if reference_grid is not None and (reference_grid.resolution != resolution or
                                           not reference_grid.cells):
            raise ValueError("Reference grid must be non-empty with matching resolution")
        if not all(math.isfinite(value) for value in (initial_pose.x, initial_pose.y,
                                                       initial_pose.yaw)):
            raise ValueError("Initial pose must be finite")
        if (not 0 < resolution < math.inf or not math.isfinite(gyro_bias) or
                not 0 <= gyro_weight <= 1 or not math.isfinite(wheel_radius) or
                wheel_radius <= 0 or not math.isfinite(track) or track <= 0):
            raise ValueError("Invalid grid resolution or gyro calibration")
        self.lidar = lidar
        self.WHEEL_RADIUS = float(wheel_radius)
        self.TRACK = float(track)
        self.gyro_bias = float(gyro_bias)
        self.gyro_weight = float(gyro_weight)
        self._directions = tuple((math.cos(a), math.sin(a)) for a in lidar.angles)
        self._mapping = reference_grid is None
        self.grid = (OccupancyGrid(float(resolution)) if reference_grid is None else
                     OccupancyGrid(float(resolution), dict(reference_grid.cells),
                                   reference_grid.max_abs_log_odds))
        self.odom_grid = OccupancyGrid(float(resolution))
        initial_pose = Pose(initial_pose.x, initial_pose.y, _wrap(initial_pose.yaw))
        self.wheel_pose = initial_pose
        self.odom_pose = initial_pose
        self.pose = initial_pose
        self.matches_attempted = 0
        self.matches_accepted = 0
        self.last_match = {"accepted": False, "reason": "no_scan"}
        self._previous: SensorSample | None = None
        self._last_keyframe = self.odom_pose

    def update(self, sample: SensorSample):
        """Consume one sample; timestamps are seconds and wheel positions radians.

        Lidar distances originate at the sensor mount in the chassis frame.
        Rays begin at the configured blind radius. Only finite in-range
        returns map as hits; None means unknown (including misses and occlusion).
        """
        if (len(sample.ranges) != len(self.lidar.angles) or len(sample.wheel_pos) != 2 or
                not all(math.isfinite(value) for value in (*sample.wheel_pos, sample.gyro_z, sample.sim_time))):
            raise ValueError("Sensor sample does not match the lidar model or wheel/gyro format")
        previous = self._previous
        if previous is None:
            if self._mapping:
                scan = self._scan(sample)
                self._map_scan(scan, self.pose, self.grid)
                self._map_scan(scan, self.odom_pose, self.odom_grid)
            self._previous = sample
            return
        dt = sample.sim_time - previous.sim_time
        if dt <= 0:
            # Duplicate/out-of-order packet: don't integrate twice or rewind.
            return
        wl, wr = sample.wheel_pos
        dl = (wl - previous.wheel_pos[0]) * self.WHEEL_RADIUS
        dr = (wr - previous.wheel_pos[1]) * self.WHEEL_RADIUS
        ds = (dl + dr) * 0.5
        wheel_dyaw = (dr - dl) / self.TRACK
        gyro_rate = 0.5 * (previous.gyro_z + sample.gyro_z) - self.gyro_bias
        gyro_dyaw = gyro_rate*dt
        fused_dyaw = (1 - self.gyro_weight) * wheel_dyaw + self.gyro_weight * gyro_dyaw
        self.wheel_pose = _advance(self.wheel_pose, ds, wheel_dyaw)
        self.odom_pose = _advance(self.odom_pose, ds, fused_dyaw)
        self.pose = _advance(self.pose, ds, fused_dyaw)
        self._previous = sample

        if (_distance(self.odom_pose, self._last_keyframe) < 0.10 and
                abs(_wrap(self.odom_pose.yaw - self._last_keyframe.yaw)) < 0.08):
            return
        scan = self._scan(sample)
        if not scan:
            return
        self._last_keyframe = self.odom_pose
        self.matches_attempted += 1
        # Keep all rays for free-space coverage; bound the correlative search cost.
        match_stride = math.ceil(len(scan) / 90)
        result, self.last_match = match_scan(self.grid, (self.pose.x, self.pose.y, self.pose.yaw),
                                             [end for _, end in scan[::match_stride]])
        if self.last_match["accepted"]:
            self.matches_accepted += 1
            self.pose = Pose(*result)
        # Both maps use the same scans. Only the matched map feeds scan matching.
        if self._mapping:
            self._map_scan(scan, self.pose, self.grid)
            self._map_scan(scan, self.odom_pose, self.odom_grid)

    def _scan(self, sample):
        blind = self.lidar.blind_radius
        x, y = self.lidar.origin
        return [((x + blind * ux, y + blind * uy), (x + distance * ux, y + distance * uy))
                for (ux, uy), distance in zip(self._directions, sample.ranges)
                if distance is not None and blind <= distance <= self.lidar.max_range]

    @staticmethod
    def _map_scan(scan, pose, grid):
        for start, end in scan:
            grid.update_ray(pose.transform(start), pose.transform(end))


def _advance(pose, distance, yaw_delta):
    """Exact constant-curvature SE(2) integration."""
    if abs(yaw_delta) < 1e-9:
        dx, dy = distance*math.cos(pose.yaw), distance*math.sin(pose.yaw)
    else:
        radius = distance/yaw_delta
        dx = radius*(math.sin(pose.yaw+yaw_delta)-math.sin(pose.yaw))
        dy = -radius*(math.cos(pose.yaw+yaw_delta)-math.cos(pose.yaw))
    return Pose(pose.x+dx, pose.y+dy, _wrap(pose.yaw+yaw_delta))


def _distance(a, b):
    return math.hypot(a.x-b.x, a.y-b.y)


def _wrap(angle):
    return (angle+math.pi)%(2*math.pi)-math.pi

import math

import pytest

from navigation.docking import Station, station_depth
from navigation.map_exports import rasterize
from navigation.mini_slam.core import LidarModel, SensorSample
from navigation.planning import StaticGrid, plan_route
from navigation.prototype import load_profile
from navigation.tracking import Pose


def test_station_depth_projects_sensor_offset_and_wraps_heading():
    station = Station(Pose(1.0, 0.0, math.pi), 0.65)
    assert station.approach.x == pytest.approx(1.35)
    pose = Pose(1.30, 0.01, -math.pi + 0.07)
    lidar = LidarModel((-0.18, -0.12, -0.04, 0.0, .04, .12, .18), .05, 10.0, (.02, .01))
    origin = .02 * math.cos(.07) - .01 * math.sin(.07)
    ranges = tuple((.95 - origin) / math.cos(angle + .07) for angle in lidar.angles)
    sample = SensorSample((0.0, 0.0), 0.0, (*ranges[:6], 0.22), 1.0)
    assert station_depth(station, pose, lidar, sample) == pytest.approx(.30)
    assert station_depth(station, Pose(1.30, .30, pose.yaw), lidar, sample) is None
    assert station_depth(station, pose, lidar, SensorSample((0., 0.), 0., (None,) * 7, 1.)) is None


def test_trinary_raster_rotates_nothing_and_preserves_unknown():
    grid = StaticGrid(.05, {(-2, 0): -0.62, (-1, 0): 0.61, (-2, 1): 0.62})
    raster = rasterize(grid)
    assert (raster.width, raster.height) == (2, 2)
    assert raster.origin == (-.1, 0.0)
    assert raster.pixels == bytes([0, 205, 254, 205])


def test_kitchen_profile_and_measured_grid_allow_both_station_approaches():
    profile = load_profile()
    radius = profile.footprint_radius + 0.05
    start = profile.initial_pose[:2]
    for label in ("A", "B", "A"):
        station = profile.stations[label]
        approach = (station.approach.x, station.approach.y)
        goal = (station.pose.x, station.pose.y)
        assert plan_route(profile.grid, start, approach, footprint_radius=radius, keepouts=profile.keepouts)
        assert profile.grid.is_segment_clear(approach, goal, footprint_radius=radius, keepouts=profile.keepouts)
        start = goal

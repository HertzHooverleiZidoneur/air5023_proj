"""Project MuJoCo collision geometry into a local static navigation grid."""

from dataclasses import dataclass
from math import floor

import mujoco
import numpy as np

from .planning import StaticGrid


@dataclass(frozen=True)
class SceneGrid:
    grid: StaticGrid
    robot_radius: float


def build_scene_grid(model, data, points, *, resolution=0.05, floor_z=0.0):
    """Snapshot collision geometry around the route in MuJoCo world coordinates.

    The robot is expected to have a fixed arm configuration during navigation.
    A circumscribed footprint includes its collidable geometry at that posture.
    """
    base = model.body("base_link")
    robot_bodies = {base.id}
    for body_id in range(base.id + 1, model.nbody):
        if model.body_parentid[body_id] in robot_bodies:
            robot_bodies.add(body_id)

    robot_geoms = {i for i in range(model.ngeom) if model.geom_bodyid[i] in robot_bodies}
    explicit_pairs = set(zip(model.pair_geom1, model.pair_geom2))
    paired_geoms = {geom for pair in explicit_pairs for geom in pair}
    robot_colliders = [i for i in robot_geoms
                       if model.geom_contype[i] or model.geom_conaffinity[i] or i in paired_geoms]
    if not robot_colliders:
        raise ValueError("The robot model has no collision geometry")

    geometry = [_world_bounds(model, data, i) for i in robot_colliders]
    minimum = np.min([bounds[0] for bounds in geometry], axis=0)
    maximum = np.max([bounds[1] for bounds in geometry], axis=0)
    base_xy = data.xpos[base.id, :2]
    radius = float(max(np.linalg.norm(np.maximum(abs(lo[:2] - base_xy), abs(hi[:2] - base_xy)))
                       for lo, hi in geometry))
    if not np.isfinite(radius) or radius <= 0:
        raise ValueError("The robot model has an invalid collision footprint")

    robot_type = int(np.bitwise_or.reduce(model.geom_contype[robot_colliders]))
    robot_affinity = int(np.bitwise_or.reduce(model.geom_conaffinity[robot_colliders]))
    explicit_obstacles = {b for a, b in explicit_pairs if a in robot_geoms}
    explicit_obstacles.update(a for a, b in explicit_pairs if b in robot_geoms)
    padding = radius + 1.0
    ix0, iy0 = (floor((value - padding) / resolution) for value in np.min(points, axis=0))
    ix1, iy1 = (floor((value + padding) / resolution) for value in np.max(points, axis=0))
    if (ix1 - ix0 + 1) * (iy1 - iy0 + 1) > 250_000:
        raise ValueError("Waypoints span too large an area for the local navigation grid")
    cells = {(x, y): -3.0 for x in range(ix0, ix1 + 1) for y in range(iy0, iy1 + 1)}

    for i in range(model.ngeom):
        if i in robot_geoms:
            continue
        collides = model.geom_contype[i] & robot_affinity or model.geom_conaffinity[i] & robot_type
        if not collides and i not in explicit_obstacles:
            continue
        if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE:
            plane_normal = data.geom_xmat[i].reshape(3, 3)[:, 2]
            if abs(plane_normal[2] - 1.0) < 1e-6 and data.geom_xpos[i, 2] <= floor_z + 0.015:
                continue
            raise ValueError("Navigation only supports horizontal planes at floor height")
        lo, hi = _world_bounds(model, data, i)
        if hi[2] <= max(floor_z + 0.015, minimum[2]) or lo[2] > maximum[2]:
            continue
        for x in range(max(ix0, floor(lo[0] / resolution)), min(ix1, floor(hi[0] / resolution)) + 1):
            for y in range(max(iy0, floor(lo[1] / resolution)), min(iy1, floor(hi[1] / resolution)) + 1):
                cells[x, y] = 3.0
    return SceneGrid(StaticGrid(resolution, cells), radius)


def _world_bounds(model, data, geom_id):
    center, half_extents = model.geom_aabb[geom_id].reshape(2, 3)
    rotation = data.geom_xmat[geom_id].reshape(3, 3)
    center = data.geom_xpos[geom_id] + rotation @ center
    extents = abs(rotation) @ half_extents
    if not np.isfinite([center, extents]).all() or np.any(extents < 0):
        raise ValueError(f"Invalid collision bounds for geometry {geom_id}")
    return center - extents, center + extents

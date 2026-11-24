"""Static occupancy-grid loading and conservative eight-connected A* planning."""

from __future__ import annotations

from dataclasses import dataclass
from functools import cached_property
import heapq
import math
from types import MappingProxyType
from typing import Mapping

from .tracking import Route


LOG_ODDS_LIMIT = 0.62
_NEIGHBOURS = tuple(
    (dx, dy, math.sqrt(2) if dx and dy else 1.0)
    for dy in (-1, 0, 1)
    for dx in (-1, 0, 1)
    if dx or dy
)


@dataclass(frozen=True)
class StaticGrid:
    """Sparse log-odds map with world-frame cell indices and unknown space."""

    resolution: float
    cells: Mapping[tuple[int, int], float]

    def __post_init__(self):
        if not math.isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError("Grid resolution must be positive and finite")
        if not self.cells:
            raise ValueError("Grid must contain at least one observed cell")
        cells = dict(self.cells)
        if not all(math.isfinite(odds) for odds in cells.values()):
            raise ValueError("Grid log odds must be finite")
        object.__setattr__(self, "cells", MappingProxyType(cells))

    @cached_property
    def bounds(self) -> tuple[float, float, float, float]:
        """Outer map edges `(xmin, ymin, xmax, ymax)` in metres."""
        min_x = min(ix for ix, _ in self.cells)
        max_x = max(ix for ix, _ in self.cells)
        min_y = min(iy for _, iy in self.cells)
        max_y = max(iy for _, iy in self.cells)
        return (min_x * self.resolution, min_y * self.resolution,
                (max_x + 1) * self.resolution, (max_y + 1) * self.resolution)

    def world_to_cell(self, point: tuple[float, float]) -> tuple[int, int]:
        """Return the containing cell, with negative coordinates floored."""
        x, y = point
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("World coordinates must be finite")
        return math.floor(x / self.resolution), math.floor(y / self.resolution)

    def cell_center(self, cell: tuple[int, int]) -> tuple[float, float]:
        ix, iy = cell
        return ((ix + 0.5) * self.resolution, (iy + 0.5) * self.resolution)

    def state(self, cell: tuple[int, int]) -> str:
        """Return occupied, free, or unknown for a sparse-map cell."""
        odds = self.cells.get(cell)
        if odds is None or -LOG_ODDS_LIMIT < odds < LOG_ODDS_LIMIT:
            return "unknown"
        return "occupied" if odds >= LOG_ODDS_LIMIT else "free"

    def plan(self, start_xy: tuple[float, float], goal_xy: tuple[float, float], *,
             footprint_radius: float,
             keepouts: tuple[tuple[float, float, float, float], ...] = ()) -> list[tuple[float, float]] | None:
        """Plan a path and return cell-center waypoints, or None if unreachable.

        `footprint_radius` is the circular robot footprint radius in metres.
        Keepouts are `(xmin, ymin, xmax, ymax)` rectangles in map coordinates.
        Unknown and out-of-map cells are blocked. Returned paths contain the
        exact cell-center waypoint sequence, including start/goal cell centers.
        """
        self._validate_clearance(footprint_radius, keepouts)
        start, goal = self.world_to_cell(start_xy), self.world_to_cell(goal_xy)
        start_center, goal_center = self.cell_center(start), self.cell_center(goal)
        if not self.is_clear(start_center, footprint_radius=footprint_radius, keepouts=keepouts):
            return None
        if not self.is_clear(goal_center, footprint_radius=footprint_radius, keepouts=keepouts):
            return None

        edge_clearance = {}

        def traversable(cell):
            center = self.cell_center(cell)
            key = (center, center)
            if key not in edge_clearance:
                edge_clearance[key] = self.is_segment_clear(
                    center, center, footprint_radius=footprint_radius, keepouts=keepouts)
            return edge_clearance[key]

        def edge_is_clear(first, second):
            a, b = self.cell_center(first), self.cell_center(second)
            key = (a, b) if a <= b else (b, a)
            if key not in edge_clearance:
                edge_clearance[key] = self.is_segment_clear(
                    a, b, footprint_radius=footprint_radius, keepouts=keepouts)
            return edge_clearance[key]

        frontier = [(0.0, 0.0, start)]
        came_from = {}
        cost_so_far = {start: 0.0}
        while frontier:
            _, queued_cost, current = heapq.heappop(frontier)
            if queued_cost != cost_so_far.get(current):
                continue
            if current == goal:
                path = [goal]
                while path[-1] != start:
                    path.append(came_from[path[-1]])
                path.reverse()
                return [self.cell_center(cell) for cell in path]

            for dx, dy, step_cost in _NEIGHBOURS:
                neighbour = current[0] + dx, current[1] + dy
                if not traversable(neighbour) or not edge_is_clear(current, neighbour):
                    continue
                if dx and dy and (not edge_is_clear(current, (current[0] + dx, current[1]))
                                  or not edge_is_clear(current, (current[0], current[1] + dy))):
                    continue
                candidate = queued_cost + step_cost
                if candidate >= cost_so_far.get(neighbour, math.inf):
                    continue
                cost_so_far[neighbour] = candidate
                came_from[neighbour] = current
                heuristic = math.hypot(goal[0] - neighbour[0], goal[1] - neighbour[1])
                heapq.heappush(frontier, (candidate + heuristic, candidate, neighbour))
        return None

    def is_clear(self, point: tuple[float, float], *, footprint_radius: float,
                 keepouts: tuple[tuple[float, float, float, float], ...] = ()) -> bool:
        """Check a circular footprint centered at a continuous map point."""
        return self.is_segment_clear(point, point, footprint_radius=footprint_radius,
                                     keepouts=keepouts)

    def is_segment_clear(self, start: tuple[float, float], end: tuple[float, float], *,
                         footprint_radius: float,
                         keepouts: tuple[tuple[float, float, float, float], ...] = ()) -> bool:
        """Check continuous segment clearance using exact segment-to-square distance."""
        self._validate_clearance(footprint_radius, keepouts)
        x0, y0 = start
        x1, y1 = end
        if not all(math.isfinite(value) for value in (x0, y0, x1, y1)):
            raise ValueError("Segment coordinates must be finite")
        xmin, ymin, xmax, ymax = self.bounds
        # Include tangent cells despite rounding at metre-to-cell boundaries.
        radius = footprint_radius + 1e-9
        if (min(x0, x1) - radius < xmin or max(x0, x1) + radius > xmax
                or min(y0, y1) - radius < ymin or max(y0, y1) + radius > ymax):
            return False

        cell_size = self.resolution
        ix_min = math.floor((min(x0, x1) - radius) / cell_size)
        ix_max = math.floor((max(x0, x1) + radius) / cell_size)
        iy_min = math.floor((min(y0, y1) - radius) / cell_size)
        iy_max = math.floor((max(y0, y1) + radius) / cell_size)
        clearance_sq = radius * radius
        for ix in range(ix_min, ix_max + 1):
            for iy in range(iy_min, iy_max + 1):
                state = self.state((ix, iy))
                if state == "free":
                    continue
                cell_box = (ix * cell_size, iy * cell_size,
                            (ix + 1) * cell_size, (iy + 1) * cell_size)
                if _segment_box_distance_squared((x0, y0), (x1, y1), cell_box) <= clearance_sq:
                    return False

        for rectangle in keepouts:
            if _segment_box_distance_squared((x0, y0), (x1, y1), rectangle) <= clearance_sq:
                return False
        return True

    @staticmethod
    def _validate_clearance(footprint_radius, keepouts):
        if not math.isfinite(footprint_radius) or footprint_radius < 0:
            raise ValueError("Footprint radius must be non-negative and finite")
        for rectangle in keepouts:
            if (len(rectangle) != 4 or not all(math.isfinite(v) for v in rectangle)
                    or rectangle[0] >= rectangle[2] or rectangle[1] >= rectangle[3]):
                raise ValueError("Keepouts must be finite non-empty (xmin, ymin, xmax, ymax) rectangles")


def _segment_box_distance_squared(start, end, box):
    """Exact minimum squared distance between a segment and an axis-aligned box."""
    xmin, ymin, xmax, ymax = box
    x0, y0 = start
    x1, y1 = end
    dx, dy = x1 - x0, y1 - y0
    low, high = 0.0, 1.0
    for origin, delta, minimum, maximum in ((x0, dx, xmin, xmax), (y0, dy, ymin, ymax)):
        if delta == 0:
            if origin < minimum or origin > maximum:
                break
            continue
        first, second = (minimum - origin) / delta, (maximum - origin) / delta
        low, high = max(low, min(first, second)), min(high, max(first, second))
        if low > high:
            break
    else:
        return 0.0

    best = min(_point_box_distance_squared(start, box),
               _point_box_distance_squared(end, box))
    corners = ((xmin, ymin), (xmin, ymax), (xmax, ymin), (xmax, ymax))
    for px, py in corners:
        best = min(best, _point_segment_distance_squared((px, py), start, end))
    return best


def _point_box_distance_squared(point, box):
    x, y = point
    xmin, ymin, xmax, ymax = box
    dx = max(xmin - x, 0.0, x - xmax)
    dy = max(ymin - y, 0.0, y - ymax)
    return dx * dx + dy * dy


def _point_segment_distance_squared(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0:
        return (point[0] - start[0]) ** 2 + (point[1] - start[1]) ** 2
    fraction = max(0.0, min(1.0, ((point[0] - start[0]) * dx
                                  + (point[1] - start[1]) * dy) / length_squared))
    nearest_x, nearest_y = start[0] + fraction * dx, start[1] + fraction * dy
    return (point[0] - nearest_x) ** 2 + (point[1] - nearest_y) ** 2


def plan_route(grid: StaticGrid, start_xy: tuple[float, float],
               goal_xy: tuple[float, float], *, footprint_radius: float,
               goal_yaw: float | None = None,
               keepouts: tuple[tuple[float, float, float, float], ...] = ()) -> Route | None:
    """Plan and safely simplify a route on a static map.

    Unknown and out-of-map cells remain blocked during search and
    simplification. ``footprint_radius`` represents the robot's circular
    clearance in metres. Keepouts are optional ``(xmin, ymin, xmax, ymax)``
    world-coordinate rectangles.
    """
    if grid.is_segment_clear(start_xy, goal_xy, footprint_radius=footprint_radius,
                             keepouts=keepouts):
        points = (start_xy,) if start_xy == goal_xy else (start_xy, goal_xy)
        return Route(points, goal_yaw)

    centers = grid.plan(start_xy, goal_xy, footprint_radius=footprint_radius,
                        keepouts=keepouts)
    if centers is None:
        return None

    points = [start_xy, *centers, goal_xy]
    waypoints = [points[0]]
    first = 0
    while first < len(points) - 1:
        following = len(points) - 1
        while following > first + 1 and not grid.is_segment_clear(
                points[first], points[following], footprint_radius=footprint_radius,
                keepouts=keepouts):
            following -= 1
        if not grid.is_segment_clear(points[first], points[following],
                                     footprint_radius=footprint_radius, keepouts=keepouts):
            return None
        waypoints.append(points[following])
        first = following
    return Route(tuple(waypoints), goal_yaw)

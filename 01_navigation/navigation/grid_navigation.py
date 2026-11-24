"""Follow static-grid routes through the Stretch RobotInterface."""

from dataclasses import dataclass
import math

from .planning import plan_route
from .scene_grid import build_scene_grid
from .tracking import Pose, RouteTracker


class GridNavigationError(RuntimeError):
    """The planned or commanded movement has insufficient clearance."""


@dataclass(frozen=True)
class Visit:
    goal: Pose
    pose: Pose
    path: tuple[tuple[float, float], ...]
    sim_time: float


def navigate_grid(robot, waypoints, timeout=150, *, on_arrival=None):
    """Visit world-frame goals with simulator pose feedback and one static map."""
    try:
        goals = tuple(Pose(*map(float, waypoint)) for waypoint in waypoints)
        if any(not all(math.isfinite(v) for v in (goal.x, goal.y, goal.yaw)) for goal in goals):
            raise ValueError("Waypoints must contain finite world x,y,yaw")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("Navigation timeout must be positive and finite")
        if not goals:
            return []

        start = _read_pose(robot)
        points = [(point.x, point.y) for point in (start, *goals)]
        scene = build_scene_grid(robot.model, robot.data, points)
        deadline = robot.data.time + timeout
        visits = []
        for goal in goals:
            start = _read_pose(robot)
            route = plan_route(scene.grid, (start.x, start.y), (goal.x, goal.y),
                               footprint_radius=scene.robot_radius + 0.05, goal_yaw=goal.yaw)
            if route is None:
                raise GridNavigationError(f"No clear path to goal ({goal.x:g}, {goal.y:g})")
            pose = _follow(robot, scene, route, deadline)
            visit = Visit(goal, pose, route.waypoints, float(robot.data.time))
            visits.append(visit)
            if on_arrival is not None:
                on_arrival(visit)
        return visits
    finally:
        robot.move_base(0.0, 0.0)
        robot.server.push_command(robot.command)


def _read_pose(robot):
    return Pose(*map(float, robot.get_state()["base_pose"]))


def _follow(robot, scene, route, deadline):
    tracker = RouteTracker(route, max_vx=robot.base_limits[0], max_wz=robot.base_limits[1])
    dt = 1.0 / robot.control_hz
    while robot.data.time < deadline:
        pose = _read_pose(robot)
        command = tracker.update(pose)
        if command.reached:
            robot.move_base(0.0, 0.0)
            robot.hold(0.3)
            stopped = _read_pose(robot)
            if tracker.update(stopped).reached:
                return stopped
            continue

        start = (pose.x, pose.y)
        distance = command.vx * dt
        end = (pose.x + distance * math.cos(pose.yaw), pose.y + distance * math.sin(pose.yaw))
        # Bound arc deviation from the straight tangent during one control tick.
        radius = scene.robot_radius + 0.02 + abs(distance * command.wz * dt) / 2
        if not scene.grid.is_segment_clear(start, end, footprint_radius=radius):
            raise GridNavigationError("Tracking stopped because the next movement lacks clearance")
        robot.move_base(command.vx, command.wz)
        robot.step()
    raise TimeoutError("Grid navigation exceeded its simulation-time budget")

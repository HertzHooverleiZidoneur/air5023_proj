"""TEAM-01: waypoint feedback for the demo's known, obstacle-free corridor."""
import numpy as np


def angle_error(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def navigate(robot, waypoints, timeout=45):
    started = robot.data.time
    try:
        for goal in waypoints:
            goal = np.asarray(goal, dtype=float)
            if goal.shape != (3,) or not np.isfinite(goal).all():
                raise ValueError("Waypoints must contain finite world x,y,yaw")
            while robot.data.time - started < timeout:
                x, y, theta = robot.get_state()["base_pose"]
                delta = goal[:2] - [x, y]
                distance = np.linalg.norm(delta)
                if distance > 0.025:
                    error = angle_error(np.arctan2(delta[1], delta[0]) - theta)
                    v = min(0.18, distance * 0.8) if abs(error) < 0.25 else 0
                    w = float(np.clip(1.8 * error, -0.6, 0.6))
                else:
                    error = angle_error(goal[2] - theta)
                    if abs(error) < 0.025:
                        break
                    v, w = 0, float(np.clip(1.8 * error, -0.5, 0.5))
                robot.move_base(v, w)
                robot.step()
            else:
                raise TimeoutError("Waypoint navigation exceeded its simulation-time budget")
    finally:
        robot.move_base(0, 0)
        robot.server.push_command(robot.command)
    robot.hold(0.3)

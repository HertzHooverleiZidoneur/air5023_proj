"""05-owned adapters; upstream task and robot implementations stay unchanged."""
import json
from pathlib import Path


def g1(robot, metadata, output, instruction, *, on_phase):
    from manipulation.tcp_retarget import TcpReplay, run_retarget
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    on_phase("initialize", float(robot.data.time))
    replay = TcpReplay.load(metadata["resources"]["replay"])
    with (output / "controller_trace.jsonl").open("w", encoding="utf-8") as handle:
        previous = None

        def record(sample):
            nonlocal previous
            # Controller callbacks are diagnostic; independent contact measurements
            # occur inside MeasuredRobot.step before this callback.
            if sample["phase"] != previous:
                on_phase(sample["phase"], float(robot.data.time))
                previous = sample["phase"]
            handle.write(json.dumps(sample, allow_nan=False) + "\n")

        result = run_retarget(robot, replay, on_sample=record)
    (output / "controller_metrics.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


def dock_probe(robot, metadata, output, instruction, *, on_phase):
    """Oracle geometry probe for the specified nominal dock, not all grasps."""
    from navigation.scene_grid import build_scene_grid
    from navigation.planning import plan_route
    on_phase("initialize", float(robot.data.time))
    robot.move_base(0, 0)
    robot.set_gripper(open=True)
    robot.move_joints({"lift": 0.55, "arm": 0.0, "wrist_yaw": 0.0,
                       "wrist_pitch": 0.0, "wrist_roll": 0.0}, 2.5)
    robot.hold(0.5)
    on_phase("plan_dock", float(robot.data.time))
    start = tuple(robot.get_state()["base_pose"][:2])
    goal = tuple(metadata["navigation_waypoints"][-1][:2])
    scene = build_scene_grid(robot.model, robot.data, [start, goal])
    route = plan_route(scene.grid, start, goal, footprint_radius=scene.robot_radius + 0.05)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    evidence = {"planner": "navigation.planning.plan_route", "footprint_radius_m": scene.robot_radius + 0.05,
                "resolution_m": scene.grid.resolution, "start": start, "goal": goal,
                "route_found": route is not None, "scope": "conservative_static_grid_nominal_dock_only"}
    (output / "planning.json").write_text(json.dumps(evidence, indent=2) + "\n", encoding="utf-8")
    robot.move_base(0, 0)
    on_phase("stopping", float(robot.data.time))
    robot.hold(0.5)
    return {"outcome": "rejected" if route is None else "route_available",
            "reason": "nominal_dock_unreachable" if route is None else None, "planning": evidence}

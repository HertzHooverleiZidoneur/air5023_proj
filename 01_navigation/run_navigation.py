"""TEAM-01: run waypoint or static-grid navigation without grasping."""
import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from project_bootstrap import activate
ROOT = activate()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--planner", choices=("waypoints", "grid"), default="waypoints")
    parser.add_argument("--goal", type=float, action="append", nargs=3, metavar=("X", "Y", "YAW"),
                        help="World-frame grid goal in metres and radians. Repeat for a patrol.")
    parser.add_argument("--timeout", type=float, help="Simulated seconds (default 45 for waypoints, 150 for grid)")
    parser.add_argument("--capture-dir", type=Path, help="Save one head RGB image at each grid goal")
    args = parser.parse_args()
    if args.timeout is not None and (not math.isfinite(args.timeout) or args.timeout <= 0):
        parser.error("--timeout must be positive and finite")
    timeout = args.timeout if args.timeout is not None else (150 if args.planner == "grid" else 45)
    if args.planner != "grid" and (args.goal or args.capture_dir):
        parser.error("--goal and --capture-dir require --planner grid")
    if args.capture_dir is not None:
        args.capture_dir.mkdir(parents=True, exist_ok=True)
        if any(args.capture_dir.iterdir()):
            parser.error("--capture-dir must be empty")
    if args.headless:
        os.environ.setdefault("MUJOCO_GL", "egl")
    from robot.robot_interface import RobotInterface
    from navigation.navigation_interface import navigate
    scene = ROOT / "scenes/kitchen/generated/kitchen.xml"
    metadata = json.loads(scene.with_name("metadata.json").read_text())
    with RobotInterface(scene, gui=not args.headless, realtime=not args.headless,
                        cameras=args.capture_dir is not None) as robot:
        robot.move_joints({"lift": .55, "arm": 0.0}, 2.5)
        if robot.viewer:
            from perception.camera import configure_overview
            configure_overview(robot.viewer.cam)
        if args.planner == "grid":
            from navigation.grid_navigation import GridNavigationError, navigate_grid

            captures = []

            def capture(_visit):
                from PIL import Image

                output = args.capture_dir / f"head-{len(captures) + 1:03d}.png"
                Image.fromarray(robot.get_camera_image("head")).save(output)
                captures.append(str(output))

            try:
                visits = navigate_grid(robot, args.goal or metadata["navigation_waypoints"],
                                       timeout=timeout,
                                       on_arrival=capture if args.capture_dir else None)
            except (GridNavigationError, TimeoutError, ValueError) as error:
                parser.exit(1, f"Grid navigation: {error}\n")
            print(json.dumps({"visits": [asdict(visit) for visit in visits], "images": captures}))
        else:
            navigate(robot, metadata["navigation_waypoints"], timeout=timeout)
            print(json.dumps({"base_pose": robot.get_state()["base_pose"].tolist(),
                              "simulation_time": robot.data.time}))


if __name__ == "__main__":
    main()

"""TEAM-01: run the existing waypoint controller without grasping."""
import argparse
import json
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from project_bootstrap import activate
ROOT = activate()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    if args.headless:
        os.environ.setdefault("MUJOCO_GL", "egl")
    from robot.robot_interface import RobotInterface
    from navigation.navigation_interface import navigate
    scene = ROOT / "scenes/kitchen/generated/kitchen.xml"
    metadata = json.loads(scene.with_name("metadata.json").read_text())
    with RobotInterface(scene, gui=not args.headless, realtime=not args.headless,
                        cameras=False) as robot:
        robot.move_joints({"lift": .55, "arm": 0.0}, 2.5)
        if robot.viewer:
            from perception.camera import configure_overview
            configure_overview(robot.viewer.cam)
        navigate(robot, metadata["navigation_waypoints"])
        print(json.dumps({"base_pose": robot.get_state()["base_pose"].tolist(),
                          "simulation_time": robot.data.time}))


if __name__ == "__main__":
    main()

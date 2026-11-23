import argparse
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from project_bootstrap import activate
activate()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-cameras", action="store_true", help="Physics-only diagnostic")
    parser.add_argument("--instruction", default="Navigate to the table and pick up the red cup.")
    parser.add_argument("--output", type=Path, default=ROOT / "out/task")
    parser.add_argument("--linger", type=float, default=0, help="Keep GUI open after the task, seconds")
    args = parser.parse_args()
    if args.headless:
        os.environ.setdefault("MUJOCO_GL", "egl")
    os.environ.setdefault("NUMBA_CACHE_DIR", str(ROOT / ".cache/numba"))
    from robot.robot_interface import RobotInterface
    from tasks.pick_red_cup import run
    scene = ROOT / "scenes/kitchen/generated/kitchen.xml"
    if not scene.exists():
        parser.error("Run .venv/bin/python 03_data_generate/build_scene.py first")
    metadata = json.loads(scene.with_name("metadata.json").read_text())
    with RobotInterface(scene, gui=not args.headless, realtime=not args.headless,
                        cameras=not args.no_cameras) as robot:
        if robot.viewer:
            from perception.camera import configure_overview
            configure_overview(robot.viewer.cam)
        result = run(robot, metadata, args.output, args.instruction)
        print(json.dumps(result, indent=2))
        if args.linger > 0:
            robot.hold(args.linger)
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

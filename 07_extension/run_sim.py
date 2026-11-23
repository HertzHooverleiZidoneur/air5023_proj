"""Launch the kitchen GUI or a bounded headless observation smoke test."""
import argparse
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
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--scene", type=Path, default=ROOT / "scenes/kitchen/generated/kitchen.xml")
    args = parser.parse_args()
    if args.headless:
        os.environ.setdefault("MUJOCO_GL", "egl")
    from robot.robot_interface import RobotInterface
    from PIL import Image
    if not args.scene.exists():
        parser.error("Scene is missing. Run .venv/bin/python 03_data_generate/build_scene.py first")
    out = ROOT / "out"
    out.mkdir(exist_ok=True)
    with RobotInterface(args.scene, gui=not args.headless, realtime=not args.headless) as robot:
        robot.set_joint_targets(lift=0.55, arm=0.0, head_pan=-1.57, head_tilt=-0.45)
        if robot.viewer:
            from perception.camera import configure_overview
            configure_overview(robot.viewer.cam)
        try:
            robot.hold(args.seconds)
            for name in ("head", "wrist"):
                Image.fromarray(robot.get_camera_image(name)).save(out / f"{name}.png")
            print(robot.get_state())
        except InterruptedError:
            pass


if __name__ == "__main__":
    main()

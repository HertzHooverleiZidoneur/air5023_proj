"""Run the G1-to-Stretch round-table grasp and placement example from its dock."""

import argparse
from contextlib import nullcontext
import hashlib
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
SCENE_DIR = ROOT / "scenes/round_table"
EXAMPLE = ROOT / "02_manipulation_vla/examples/g1_tcp_round_table"


def _camera_view(camera):
    camera.lookat[:] = [3.51, -1.94, 0.67]
    camera.distance = 2.35
    camera.azimuth = 51
    camera.elevation = -24


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=SCENE_DIR / "generated/kitchen_round_table.xml")
    parser.add_argument("--source", type=Path, default=EXAMPLE / "g1_tcp_source.npz")
    parser.add_argument("--dock", type=Path, default=SCENE_DIR / "dock.json")
    parser.add_argument("--output", type=Path, required=True, help="New run directory")
    parser.add_argument("--no-support-projection", action="store_true")
    parser.add_argument("--no-release-adapt", action="store_true")
    view = parser.add_mutually_exclusive_group()
    view.add_argument("--gui", action="store_true", help="Watch at real-time speed in MuJoCo")
    view.add_argument("--render", action="store_true", help="Save an overview PNG per control step")
    args = parser.parse_args()
    if not args.scene.is_file():
        parser.error("Generate the round-table scene first with 03_data_generate/build_round_table.py")
    if args.output.exists():
        parser.error("Output directory already exists")
    if args.render:
        os.environ.setdefault("MUJOCO_GL", "egl")
    os.environ.setdefault("NUMBA_CACHE_DIR", str(ROOT / ".cache/numba"))
    os.environ.setdefault("MPLCONFIGDIR", str(ROOT / ".cache/matplotlib"))
    sys.path.insert(0, str(ROOT))
    from project_bootstrap import activate

    activate()
    import mujoco
    from manipulation.tcp_retarget import TcpReplay, restore_dock, run_retarget
    from robot.robot_interface import RobotInterface

    replay = TcpReplay.load(args.source)
    args.output.mkdir(parents=True)
    metrics = {"source_sha256": hashlib.sha256(args.source.read_bytes()).hexdigest(), "start_mode": "fixed_dock"}
    try:
        with RobotInterface(args.scene, gui=args.gui, realtime=args.gui, cameras=False) as robot:
            restore_dock(robot, args.dock)
            if robot.viewer:
                with robot.viewer.lock():
                    _camera_view(robot.viewer.cam)
            camera = None
            frames = args.output / "frames"
            if args.render:
                from PIL import Image

                camera = mujoco.MjvCamera()
                _camera_view(camera)
                frames.mkdir()
            video = mujoco.Renderer(robot.model, height=480, width=640) if camera else nullcontext()
            with video as renderer, (args.output / "trajectory.jsonl").open("w", encoding="utf-8") as trace:
                frame = 0

                def record(sample):
                    nonlocal frame
                    trace.write(json.dumps(sample) + "\n")
                    if renderer is not None:
                        renderer.update_scene(robot.data, camera)
                        Image.fromarray(renderer.render()).save(frames / f"{frame:05d}.png", compress_level=2)
                        frame += 1

                metrics.update(run_retarget(robot, replay, support_projection=not args.no_support_projection,
                                            release_adapt=not args.no_release_adapt, on_sample=record))
    except (RuntimeError, ValueError, InterruptedError) as error:
        metrics.update(passed=False, error=str(error))
    (args.output / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metrics, indent=2))
    return 0 if metrics["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

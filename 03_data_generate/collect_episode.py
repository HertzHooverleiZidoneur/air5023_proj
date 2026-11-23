"""TEAM-03: record a scripted episode at every control tick (no model training)."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from project_bootstrap import activate
ROOT = activate()
os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("NUMBA_CACHE_DIR", str(ROOT / ".cache/numba"))

import numpy as np
from robot.robot_interface import JOINTS, RobotInterface

STATE_ORDER = ("base_x", "base_y", "base_yaw", *JOINTS, "gripper_raw")


class CollectingRobot(RobotInterface):
    recording = False

    def vector(self):
        state = self.get_state()
        return np.array([*state["base_pose"], *(state["joints"][n] for n in JOINTS),
                         state["joints"]["gripper"]])

    def step(self, steps=1):
        for _ in range(steps):
            if not self.recording:
                super().step()
                continue
            before, started = self.vector(), float(self.data.time)
            super().step()
            image_index = -1
            if self.camera_manager:
                if self._camera_time != self.last_image_time:
                    from PIL import Image
                    image_index = len(self.images)
                    for camera in ("head", "wrist"):
                        Image.fromarray(self.get_camera_image(camera)).save(
                            self.output / "images" / f"{image_index:06d}_{camera}.png")
                    self.last_image_time = self._camera_time
                    self.images.append(float(self._camera_time))
                image_index = len(self.images) - 1
            self.rows.append((started, float(self.data.time), before, self.vector(),
                              self.data.ctrl.copy(), self.data.qpos.copy(),
                              self.data.qvel.copy(), image_index))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New episode directory")
    parser.add_argument("--with-images", action="store_true", help="Save synchronized 10 Hz RGB PNGs")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Episode directory exists; choose a new output directory")
    scene = ROOT / "scenes/kitchen/generated/kitchen.xml"
    metadata = json.loads(scene.with_name("metadata.json").read_text())
    from tasks.pick_red_cup import INSTRUCTION, run
    args.output.mkdir(parents=True)
    if args.with_images:
        (args.output / "images").mkdir()
    with CollectingRobot(scene, cameras=args.with_images) as robot:
        robot.output, robot.rows, robot.images = args.output, [], []
        robot.last_image_time = None
        robot.recording = True
        result = run(robot, metadata, args.output / "stages")
        robot.recording = False
        columns = tuple(zip(*robot.rows))
        np.savez_compressed(args.output / "trajectory.npz", **{
            name: np.asarray(values) for name, values in zip(
                ("time", "next_time", "state", "next_state", "actuator_ctrl", "qpos", "qvel", "image_index"), columns)
        }, image_time=np.asarray(robot.images))
        manifest = {
            "schema_version": "episode-1.0", "embodiment": "stretch3",
            "instruction": INSTRUCTION, "controller": "scripted_oracle_baseline",
            "vla_loaded": False, "state_order": STATE_ORDER,
            "state_units": "metres/radians; gripper_raw uses official [-0.376, 0.56] calibration",
            "control_hz": robot.control_hz, "camera_hz": robot.camera_hz,
            "samples": len(robot.rows), "images": len(robot.images),
            "image_alignment": "image_index refers to the observation after this control tick",
            "actuator_order": [robot.model.actuator(i).name for i in range(robot.model.nu)],
            "action_semantics": "actuator_ctrl is MuJoCo low-level control at tick end, NOT VLA Action",
            "privileged_fields": ["qpos", "qvel"],
            "scene_sha256": hashlib.sha256(scene.read_bytes()).hexdigest(),
            "seed": metadata["seed"], "result": result,
        }
        (args.output / "episode.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"episode": str(args.output), "samples": manifest["samples"],
                      "success": result["success"]}))
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

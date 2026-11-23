"""Deterministic oracle-goal baseline. Extension owner: TEAM-02 (task orchestration).

Numbered handoff: docs/development/README.md. VLA remains a separate plugin.
"""
import json
from pathlib import Path
import numpy as np
from PIL import Image
from evaluation.success_checker import SuccessChecker
from navigation.navigation_interface import navigate
from perception.red_cup import detect_red

INSTRUCTION = "Navigate to the table and pick up the red cup."


def run(robot, metadata, output, instruction=INSTRUCTION, *, on_phase=None):
    if instruction.strip().lower().rstrip(".!。") != INSTRUCTION.lower().rstrip("."):
        raise ValueError("This baseline implements only the documented red-cup instruction. Supply a Policy for other instructions.")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    stages = []

    def phase(name):
        if on_phase is not None:
            on_phase(name, float(robot.data.time))

    def record(stage):
        state = robot.get_state()
        target = robot.get_object_pose(metadata["target_body"])["position"]
        item = {"stage": stage, "time": float(robot.data.time),
                "base_pose": state["base_pose"].tolist(), "joints": state["joints"],
                "target": target.tolist(), "tcp": robot.data.body("link_grasp_center").xpos.tolist(),
                "grasp": checker.update()}
        if robot.camera_manager:
            image = robot.get_camera_image("head")
            item["red_detection"] = detect_red(image)
            for name in ("head", "wrist"):
                Image.fromarray(robot.get_camera_image(name)).save(output / f"{stage}_{name}.png")
            from perception.camera import save_overview
            save_overview(robot, output / f"{stage}_overview.png")
        stages.append(item)
        np.savez_compressed(output / f"{stage}_state.npz", qpos=robot.data.qpos,
                            qvel=robot.data.qvel, ctrl=robot.data.ctrl, time=robot.data.time)
        print(json.dumps(item), flush=True)
        (output / "stages.json").write_text(json.dumps(stages, indent=2) + "\n")

    phase("initialize")
    robot.set_gripper(open=True)
    robot.move_joints({"lift": 0.55, "arm": 0.0, "wrist_yaw": 0.0,
                       "wrist_pitch": 0.0, "wrist_roll": 0.0,
                       "head_pan": -1.57, "head_tilt": -0.45}, 2.5)
    robot.hold(0.5)
    from configs import load
    task_config = load("task")
    checker = SuccessChecker(robot, metadata["target_body"],
                             task_config["minimum_lift_metres"], task_config["minimum_hold_seconds"])
    record("start")
    phase("navigate")
    navigate(robot, metadata["navigation_waypoints"])
    record("navigated")
    cup = robot.get_object_pose(metadata["target_body"])["position"]
    # Head camera observation is exported, while this baseline takes its metric
    # goal from the scene. Future policies receive observation without this oracle.
    # On closing, the compliant finger pads retract about 2.8 cm from the
    # official link_grasp_center frame. Advance that frame to center the pads.
    target = cup + [0, -0.028, 0.025]
    phase("pregrasp")
    robot.move_joints({"lift": float(target[2] - 0.10)}, 2.5)
    robot.move_to_pose({"position": target + [0, 0.13, 0]}, 2.5)
    robot.hold(0.5)
    record("pregrasp")
    phase("approach")
    robot.move_to_pose({"position": target}, 3)
    robot.hold(0.8)
    record("approached")
    phase("close")
    robot.set_gripper(open=False)
    robot.hold(1.5)
    record("closed")
    phase("lift")
    current = robot.get_state()["joints"]
    robot.move_joints({"lift": min(current["lift"] + 0.14, 1.08)}, 3)
    record("lifted")
    phase("verify")
    trace = []
    for _ in range(round(2 * robot.control_hz)):
        robot.step()
        trace.append(checker.update())
    result = {**trace[-1], "controller": "scripted_oracle_baseline", "vla_loaded": False,
              "instruction": instruction, "seed": metadata["seed"],
              "physics_hz": 1 / robot.model.opt.timestep,
              "control_hz": robot.control_hz, "camera_hz": robot.camera_hz}
    (output / "metrics.json").write_text(json.dumps(result, indent=2) + "\n")
    (output / "success_trace.json").write_text(json.dumps(trace, indent=2) + "\n")
    record("finished")
    return result

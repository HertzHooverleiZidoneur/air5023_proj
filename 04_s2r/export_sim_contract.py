"""TEAM-04: export simulation units/limits for later real-robot calibration."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from project_bootstrap import activate
ROOT = activate()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output already exists")
    import mujoco
    from configs import load
    from robot.robot_interface import JOINTS
    model = mujoco.MjModel.from_xml_path(str(ROOT / "scenes/kitchen/generated/kitchen.xml"))
    data = {
        "schema_version": "s2r-contract-1.0", "source": "simulation_only",
        "hardware_verified": False, "real_backend_implemented": False,
        "robot_config": load("robot"),
        "joint_targets": {name: {"unit": "m" if name in ("lift", "arm") else "rad",
                                  "absolute": True, "range": model.actuator(name).ctrlrange.tolist()}
                          for name in JOINTS},
        "gripper": {"action_range": [0, 1], "raw_state_range": [-.376, .56]},
        "camera_resolution_hwc": {"head": [240, 424, 3], "wrist": [270, 480, 3]},
        "real_measurements_required": ["camera_intrinsics_extrinsics", "joint_zero_offsets",
                                       "gripper_mapping", "latency", "watchdog_and_stop_behavior"],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(data, indent=2) + "\n")
    print(f"Simulation contract exported to {args.output}; no hardware was contacted.")


if __name__ == "__main__":
    main()

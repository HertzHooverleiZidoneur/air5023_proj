"""TEAM-06: validate collected transitions and export nonprivileged state arrays."""
import argparse
import json
from pathlib import Path
import numpy as np


def process_episode(source, output):
    metadata = json.loads((source / "episode.json").read_text())
    if metadata.get("schema_version") != "episode-1.0":
        raise ValueError("Unsupported episode schema")
    with np.load(source / "trajectory.npz", allow_pickle=False) as archive:
        data = {name: archive[name].copy() for name in archive.files}
    n = metadata["samples"]
    state_dim = len(metadata["state_order"])
    if n < 1 or data["state"].shape != (n, state_dim) or data["next_state"].shape != (n, state_dim):
        raise ValueError("Invalid state shapes or empty episode")
    if data["actuator_ctrl"].shape != (n, len(metadata["actuator_order"])):
        raise ValueError("Invalid actuator control shape")
    for name in ("time", "next_time", "state", "next_state", "actuator_ctrl"):
        if len(data[name]) != n or not np.isfinite(data[name]).all():
            raise ValueError(f"Invalid or non-finite {name}")
    dt = 1 / metadata["control_hz"]
    if (not np.allclose(data["next_time"] - data["time"], dt, atol=1e-7, rtol=0)
            or not np.allclose(data["time"][1:], data["next_time"][:-1], atol=1e-7, rtol=0)
            or not np.allclose(data["state"][1:], data["next_state"][:-1], atol=1e-7, rtol=0)):
        raise ValueError("Discontinuous control transitions")
    for i in range(metadata["images"]):
        for camera in ("head", "wrist"):
            if not (source / "images" / f"{i:06d}_{camera}.png").exists():
                raise ValueError("Missing RGB frame")
    indices = data["image_index"]
    if indices.shape != (n,) or not np.issubdtype(indices.dtype, np.integer):
        raise ValueError("Invalid image index shape or dtype")
    if metadata["images"]:
        if (np.any(indices < 0) or np.any(indices >= metadata["images"])
                or len(data["image_time"]) != metadata["images"]):
            raise ValueError("Invalid RGB references")
        delay = data["next_time"] - data["image_time"][indices]
        if not np.isfinite(delay).all() or np.any(delay < -1e-7) or np.any(delay > 1 / metadata["camera_hz"] + 1e-7):
            raise ValueError("Invalid RGB timestamps")
    elif np.any(indices != -1):
        raise ValueError("Image references exist without RGB frames")
    if output.exists():
        raise FileExistsError("Processed output exists; choose a new directory")
    output.mkdir(parents=True)
    # qpos/qvel include the target and scene; deliberately exclude those fields.
    np.savez_compressed(output / "transitions.npz", **{key: data[key] for key in (
        "time", "next_time", "state", "next_state", "actuator_ctrl", "image_index", "image_time")})
    report = {"schema_version": "processed-episode-1.0", "source": str(source.resolve()),
              "samples": n, "duration_seconds": float(data["next_time"][-1] - data["time"][0]),
              "state_order": metadata["state_order"], "state_mean": data["state"].mean(axis=0).tolist(),
              "state_std": data["state"].std(axis=0).tolist(), "images": metadata["images"],
              "image_root": str((source / "images").resolve()),
              "action_semantics": metadata["action_semantics"],
              "actuator_order": metadata["actuator_order"], "state_units": metadata["state_units"],
              "instruction": metadata["instruction"], "controller": metadata["controller"],
              "seed": metadata["seed"], "scene_sha256": metadata["scene_sha256"],
              "control_hz": metadata["control_hz"], "camera_hz": metadata["camera_hz"],
              "result": metadata["result"], "excluded_privileged_fields": ["qpos", "qvel"],
              "vla_training_dataset": False}
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = process_episode(args.input, args.output)
    print(json.dumps({"samples": report["samples"], "duration_seconds": report["duration_seconds"]}))


if __name__ == "__main__":
    main()

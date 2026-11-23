"""Regression for the TEAM-03 -> TEAM-06 episode contract."""
import json
import numpy as np
import pytest
from process_episode import process_episode


@pytest.fixture
def episode(tmp_path):
    source = tmp_path / "raw"
    source.mkdir()
    metadata = {
        "schema_version": "episode-1.0", "samples": 2,
        "state_order": ["base_x", "base_y"], "actuator_order": ["test_actuator"],
        "images": 0, "control_hz": 20, "camera_hz": 10,
        "action_semantics": "low-level actuator control", "state_units": "m",
        "instruction": "test", "controller": "test_fixture", "seed": 7,
        "scene_sha256": "test_fixture", "result": {"success": False},
    }
    data = {
        "time": np.array([0, .05]), "next_time": np.array([.05, .1]),
        "state": np.array([[0., 0.], [1., 0.]]),
        "next_state": np.array([[1., 0.], [2., 0.]]),
        "actuator_ctrl": np.ones((2, 1)), "image_index": np.array([-1, -1]),
        "image_time": np.array([]), "qpos": np.zeros((2, 9)), "qvel": np.zeros((2, 8)),
    }
    return source, tmp_path / "processed", metadata, data


def save_raw(source, metadata, data):
    (source / "episode.json").write_text(json.dumps(metadata))
    np.savez_compressed(source / "trajectory.npz", **data)


def test_process_excludes_world_truth_and_preserves_transition_semantics(episode):
    source, output, metadata, data = episode
    save_raw(source, metadata, data)
    report = process_episode(source, output)
    with np.load(output / "transitions.npz", allow_pickle=False) as processed:
        assert "qpos" not in processed.files and "qvel" not in processed.files
        np.testing.assert_array_equal(processed["state"], data["state"])
        np.testing.assert_array_equal(processed["next_state"], data["next_state"])
    assert report["duration_seconds"] == pytest.approx(.1)
    assert report["vla_training_dataset"] is False
    assert report["action_semantics"] == metadata["action_semantics"]


@pytest.mark.parametrize("failure", ["nonfinite", "discontinuous_time", "missing_image"])
def test_reject_invalid_episode_before_writing_output(episode, failure):
    source, output, metadata, data = episode
    if failure == "nonfinite":
        data["state"][0, 0] = np.nan
    elif failure == "discontinuous_time":
        data["time"][1] += .01
    else:
        metadata["images"] = 1
        data["image_index"][:] = 0
        data["image_time"] = np.array([.05])
    save_raw(source, metadata, data)
    with pytest.raises(ValueError):
        process_episode(source, output)
    assert not output.exists()

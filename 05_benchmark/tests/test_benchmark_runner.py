from pathlib import Path
import sys
import pytest
import run_benchmark as runner
from evaluation.criteria import Criteria
from evaluation.protocol import load_suite, read_json, write_json
from evaluation.reporting import aggregate, compare_reports, wilson


@pytest.fixture
def suite(tmp_path):
    (tmp_path / "scene.xml").write_text("<mujoco/>")
    write_json(tmp_path / "metadata.json", {"seed": 7, "target_body": "red_cup_main",
                                           "navigation_waypoints": [[1, 0, 0]]})
    manifest = tmp_path / "suite.json"
    write_json(manifest, {"schema_version": "benchmark-suite-1.0", "conditions": [
        {"id": "condition-a", "scene": "scene.xml", "metadata": "metadata.json", "seed": 7}]})
    return manifest


def valid_metrics(success=True):
    return {"schema_version": "benchmark-episode-1.0", "task_success": success, "success": success,
            "criteria": Criteria().to_dict(), "navigation_success": True, "grasp_success": success, "collision_free": True,
            "reason": None if success else "missing_finger_contact", "last_phase": "verify"}


def test_manifest_resolves_paths_and_scene_seed_is_not_policy_seed(suite):
    conditions = load_suite(suite)
    assert Path(conditions[0]["scene"]).is_absolute()
    assert len(conditions[0]["scene_sha256"]) == 64
    raw = read_json(suite)
    raw["conditions"][0]["seed"] = 8
    write_json(suite, raw)
    with pytest.raises(ValueError, match="seed must match"):
        load_suite(suite)


def test_duplicate_ids_and_missing_scene_fail_preflight(suite):
    raw = read_json(suite)
    raw["conditions"] *= 2
    write_json(suite, raw)
    with pytest.raises(ValueError, match="unique"):
        load_suite(suite)
    (suite.parent / "scene.xml").unlink()
    with pytest.raises(ValueError, match="Missing scene"):
        load_suite(suite)


@pytest.mark.parametrize("payload", ['{"task_success": NaN}', '{broken', '[]'])
def test_bad_metrics_are_failed_episodes_not_batch_crashes(tmp_path, payload):
    (tmp_path / "metrics.json").write_text(payload)
    result = runner.collect_result(tmp_path, 0, None, Criteria().to_dict())
    assert not result["success"]
    assert result["reason"] == "invalid_metrics"


def test_timeout_and_nonzero_exit_cannot_be_hidden_by_success_file(tmp_path):
    write_json(tmp_path / "metrics.json", valid_metrics())
    result = runner.collect_result(tmp_path, 124, "wall_clock_timeout", Criteria().to_dict())
    assert result["reason"] == "wall_clock_timeout"
    assert not result["success"]
    assert runner.collect_result(tmp_path, 1, None, Criteria().to_dict())["reason"] == "nonzero_exit"


def test_missing_metrics_retains_last_stage(tmp_path):
    write_json(tmp_path / "progress.json", {"phase": "navigate"})
    result = runner.collect_result(tmp_path, 1, None, Criteria().to_dict())
    assert result["reason"] == "missing_metrics"
    assert result["failure_stage"] == "navigate"


def test_contradictory_success_metrics_are_rejected(tmp_path):
    write_json(tmp_path / "metrics.json", {**valid_metrics(), "grasp_success": False})
    assert runner.collect_result(tmp_path, 0, None, Criteria().to_dict())["reason"] == "invalid_metrics"


def test_real_subprocess_failure_and_timeout(tmp_path):
    assert runner.run_child([sys.executable, "-c", "raise SystemExit(3)"], tmp_path / "fail.log", 5) == (3, None)
    assert runner.run_child([sys.executable, "-c", "import time; time.sleep(20)"], tmp_path / "timeout.log", .2) == (124, "wall_clock_timeout")
    assert runner.run_child([str(tmp_path / "missing_python")], tmp_path / "launch.log", 1) == (127, "launch_error")


def test_full_batch_keeps_failures_and_writes_report(tmp_path, suite, monkeypatch):
    output, calls = tmp_path / "results", []

    def child(command, log_path, timeout, env):
        request = read_json(command[-1])
        calls.append(request)
        episode = Path(command[-1]).parent
        if len(calls) == 2:
            (episode / "metrics.json").write_text("invalid")
            return 1, None
        write_json(episode / "metrics.json", valid_metrics())
        return 0, None

    monkeypatch.setattr(runner, "run_child", child)
    monkeypatch.setattr(runner, "provenance", lambda: {"test_fixture": True})
    monkeypatch.setattr(sys, "argv", ["benchmark", "--manifest", str(suite), "--output", str(output),
                                    "--seeds", "7", "9", "--episodes", "2"])
    assert runner.main() == 1
    report = read_json(output / "summary.json")
    assert len(report["episodes"]) == 4
    assert report["success_rate"] == .75
    assert report["statistics"]["failures"] == {"invalid_metrics": 1}
    assert report["scene_variation"] is False
    assert [request["policy_seed"] for request in calls] == [7, 7, 9, 9]
    assert {request["condition"]["seed"] for request in calls} == {7}
    assert report["statistics"]["by_policy_seed"]["7"]["success_rate"] == .5
    assert (output / "episodes.csv").exists()
    assert (output / "report.md").exists()
    with pytest.raises(SystemExit):
        runner.main()


def test_preflight_writes_nothing(tmp_path, suite, monkeypatch):
    output = tmp_path / "results"
    monkeypatch.setattr(sys, "argv", ["benchmark", "--manifest", str(suite), "--output", str(output), "--preflight"])
    assert runner.main() == 0
    assert not output.exists()


def test_interruption_preserves_attempted_count_and_plan(tmp_path, suite, monkeypatch):
    output = tmp_path / "interrupted"
    monkeypatch.setattr(runner, "run_child", lambda *args: (130, "interrupted"))
    monkeypatch.setattr(runner, "provenance", lambda: {})
    monkeypatch.setattr(sys, "argv", ["benchmark", "--manifest", str(suite), "--output", str(output), "--episodes", "3"])
    assert runner.main() == 130
    report = read_json(output / "summary.json")
    assert report["status"] == "interrupted"
    assert report["planned_episodes"] == 3
    assert len(report["episodes"]) == 1
    assert report["success_rate"] == 0


def test_aggregation_denominator_and_observation_coverage():
    rows = [{"success": True, "reason": None, "metrics": {"navigation_seconds": 2, "grasp_success": True}},
            {"success": False, "reason": "wall_clock_timeout", "metrics": None}]
    stats = aggregate(rows)
    assert stats["success_rate"] == .5
    assert stats["grasp_success_rate"] == .5
    assert stats["navigation_seconds"]["mean"] == 2
    assert stats["navigation_seconds"]["observed_episodes"] == 1
    assert wilson(0, 20)[1] == pytest.approx(.161125158, abs=1e-7)


def test_paired_regression_requires_same_matrix(suite):
    row = {"condition_id": "condition-a", "strategy": "scripted_oracle_baseline", "policy_seed": 7,
           "repeat": 0, "success": True, "reason": None}
    old = {"schema_version": "benchmark-report-1.0", "criteria": Criteria().to_dict(),
           "conditions": load_suite(suite), "episodes": [row], "success_rate": 1}
    new = {**old, "episodes": [{**row, "success": False, "reason": "object_supported"}], "success_rate": 0}
    assert compare_reports(new, old)["regression_count"] == 1
    with pytest.raises(ValueError, match="identical conditions"):
        compare_reports({**new, "episodes": [{**row, "policy_seed": 8}]}, old)
    with pytest.raises(ValueError, match="identical criteria"):
        compare_reports({**new, "criteria": {}}, old)
    with pytest.raises(ValueError, match="identical instruction"):
        compare_reports({**new, "instruction": "different task"}, old)

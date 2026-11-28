"""Cross-task and negative-track regressions; synthetic traces are not scores."""
from pathlib import Path
import pytest

from evaluation.criteria import Criteria
from evaluation.placement import PlacementCheck, PlacementCriteria
from evaluation.protocol import read_json, write_json, load_suite, sha256
from evaluation.rejection import StopCheck
from evaluation.reporting import summarize, compare_reports
from evaluation.tasks import load_strategies, compatibility, MOBILE, PLACE
from evaluation.telemetry import Telemetry
from import_suite import convert
from run_benchmark import collect_result

STOP = {"minimum_stop_seconds": .3, "maximum_linear_speed_mps": .015, "maximum_angular_speed_rps": .04}


def observation(t, *, held=False, table=False, z=None):
    return {"t": t, "pos": [0., 0., z if z is not None else (1.1 if held else 1.)],
            "left": held, "right": held, "support": table, "table_only": table and not held,
            "contacts": ["g1_round_table_support"] if table else [], "speed": 0., "tilt_deg": 0.}


def lifted(check):
    check.start([0, 0, 1])
    check.gripper_command(False, 0)
    for tick in range(22):
        check.update(observation(tick * .05, held=True))
    check.update(observation(1.1, held=True, table=True, z=1.))
    check.gripper_command(True, 1.1)


def test_placement_uses_elapsed_time_and_final_state_not_sample_count():
    check = PlacementCheck(20)
    lifted(check)
    for tick in range(30):
        check.update(observation(1.15 + tick * .05, table=True))
    assert check.result()["stable_table_only_seconds"] == pytest.approx(1.45)
    assert not check.result()["success"]
    check.update(observation(2.65, table=True))
    assert check.result()["success"]
    check.update(observation(2.7, table=False, z=.5))
    assert not check.result()["success"]


def test_placement_gaps_do_not_count_as_continuous_hold_or_settle():
    check = PlacementCheck(20)
    check.start([0, 0, 1])
    check.update(observation(0, held=True))
    check.update(observation(2, held=True))
    assert check.result()["both_pads_unsupported_lift_seconds"] == 0
    check = PlacementCheck(20)
    lifted(check)
    check.update(observation(1.15, table=True))
    check.update(observation(4, table=True))
    assert not check.result()["success"]
    assert check.result()["stable_table_only_seconds"] == 0


def test_placement_rejects_supported_pick_missing_release_and_invalid_state():
    check = PlacementCheck(20)
    check.start([0, 0, 1])
    for tick in range(50):
        check.update(observation(tick * .05, held=True, table=True))
    assert not check.result()["grasp_success"]
    check = PlacementCheck(20)
    lifted(check)
    for tick in range(40):
        check.update(observation(1.15 + tick * .05, table=True))
    check.update(observation(3.2, table=True), finite=False)
    assert not check.result()["success"]
    assert PlacementCheck(20).result()["success"] is False


@pytest.mark.parametrize("controller,collision,error,passed", [
    ({"outcome": "rejected", "reason": "nominal_dock_unreachable"}, 0, None, True),
    ({"success": True}, 0, None, False),
    ({"outcome": "rejected", "reason": "nominal_dock_unreachable"}, 1, None, False),
    ({"outcome": "rejected", "reason": "nominal_dock_unreachable"}, 0, {"type": "TimeoutError"}, False)])
def test_safe_rejection_requires_explicit_stop_without_error_or_collision(controller, collision, error, passed):
    check = StopCheck(20, STOP)
    for tick in range(8):
        check.update(tick * .05, [0, 0, 0])
    assert check.result(controller, collision, error)["success"] is passed
    check.update(.4, [.1, 0, 0])
    assert not check.result(controller, collision, error)["success"]


def test_operation_collisions_are_separate_and_target_pairs_can_be_excluded():
    telemetry = Telemetry()
    telemetry.set_phase("approach", 0)
    telemetry.sample(0, [0, 0, 0], [("finger", "target")], [])
    telemetry.sample(.05, [0, 0, 0], [("finger", "table")], [("finger", "table")])
    telemetry.sample(.10, [0, 0, 0], [("finger", "table")], [("finger", "table")])
    result = telemetry.result(None, Criteria())
    assert result["operation_collision_events"] == 1
    assert result["operation_contact_samples"] == 2
    assert result["navigation_collision_events"] == 0


def row(track, success, task=MOBILE, info="oracle", seed=7):
    return {"track": track, "task": task, "information_class": info, "audit_status": "source_reviewed",
            "split": "unspecified", "strategy": "fixture", "condition_id": track + task, "policy_seed": seed,
            "success": success, "reason": None if success else "failed", "metrics": {
                "safe_rejection_success": success if track == "safe_rejection" else None,
                "placement_success": success if task == PLACE else None}}


def test_negative_success_never_inflates_feasible_rate_and_groups_do_not_mix():
    stats = summarize([row("feasible", False), row("safe_rejection", True), row("feasible", True, PLACE),
                       row("feasible", False, info="sensor_only")])
    assert stats["episodes"] == 3
    assert stats["success_rate"] == pytest.approx(1 / 3)
    assert stats["safe_rejection"]["episodes"] == 1
    assert stats["safe_rejection"]["success_rate"] == 1
    assert len(stats["primary_groups"]) == 4
    assert summarize([row("safe_rejection", True)])["success_rate"] is None


def test_importer_preserves_all_16_conditions_without_inventing_jar_controller(tmp_path):
    scene = tmp_path / "kitchen.xml"
    scene.write_text("<mujoco/>")
    metadata = tmp_path / "metadata.json"
    write_json(metadata, {"seed": 7, "target_body": "red_cup_main", "navigation_waypoints": [[1, 0, 0]]})
    records = []
    for table in ("round", "island"):
        for case in ("a", "b", "c", "d1", "d2", "e", "f1", "f2"):
            records.append({"id": f"{table}_{case}", "table": table, "case": case, "scene": scene.name,
                            "scene_sha256": sha256(scene), "target_body": "reference_jar" if table == "round" else "red_cup_main",
                            "expected_nominal_dock_reachable": case != "c", "nominal_dock_xyyaw": [1, 0, 0],
                            "static_and_settling_checks": "passed"})
    challenges = tmp_path / "challenges.json"
    write_json(challenges, {"conditions": records})
    manifest = convert(tmp_path / "suite", scene, metadata, g1_scene=scene, challenges=challenges)
    conditions = load_suite(manifest)
    assert len(conditions) == 18
    assert sum(c["availability"] == "pending_controller" for c in conditions) == 7
    assert sum(c["track"] == "safe_rejection" for c in conditions) == 2
    assert {c["split"] for c in conditions} == {"unspecified"}
    negative = next(c for c in conditions if c["id"] == "round_c")
    assert "expected_nominal_dock_reachable" not in read_json(negative["metadata"])
    assert compatibility(negative, load_strategies()["oracle_dock_probe"]) is None
    assert compatibility(negative, load_strategies()["scripted_oracle_baseline"])
    raw = read_json(manifest)
    raw["conditions"][2]["track"] = "safe_rejection"
    write_json(manifest, raw)
    with pytest.raises(ValueError, match="explicitly blocked"):
        load_suite(manifest)


def test_declared_sensor_only_cannot_claim_truth_pose(tmp_path):
    spec = {**load_strategies()["scripted_oracle_baseline"], "information_class": "sensor_only"}
    path = tmp_path / "strategies.json"
    write_json(path, {"schema_version": "benchmark-strategies-1.0", "strategies": {"bad": spec}})
    with pytest.raises(ValueError, match="simulator truth"):
        load_strategies(path)


def test_collector_rejects_placement_claim_without_actual_place(tmp_path):
    metrics = {"schema_version": "benchmark-episode-2.0", "task": PLACE, "track": "feasible",
               "criteria": Criteria().to_dict(), "task_success": True, "success": True,
               "navigation_success": None, "grasp_success": True, "placement_success": False,
               "placement": {"success": False, "criteria": PlacementCriteria().to_dict()}}
    write_json(tmp_path / "metrics.json", metrics)
    assert collect_result(tmp_path, 0, None, Criteria().to_dict())["reason"] == "invalid_metrics"
    metrics.update(task=MOBILE, track="safe_rejection", task_success=False, grasp_success=None,
                   safe_rejection_success=True, rejection={"success": True, "explicit_rejection": True,
                                                           "stopped": False, "collision_events": 0})
    write_json(tmp_path / "metrics.json", metrics)
    assert collect_result(tmp_path, 0, None, Criteria().to_dict())["reason"] == "invalid_metrics"

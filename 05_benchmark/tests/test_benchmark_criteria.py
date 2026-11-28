"""TEAM-05 success and failure fixtures; no simulation packages required."""
import math
import pytest
from evaluation.criteria import Criteria, HoldCheck, load_criteria
from evaluation.telemetry import Telemetry, failure_reason


def tick(check, time, **updates):
    return check.update(**{**dict(time=time, lift=0.1, left=True, right=True, supported=False), **updates})


@pytest.mark.parametrize("invalid", [dict(left=False), dict(right=False), dict(supported=True),
                                     dict(lift=0.049), dict(finite=False), dict(lift=math.nan)])
def test_invalid_grasp_never_passes(invalid):
    check = HoldCheck()
    for time in (0, 1, 2, 3):
        result = tick(check, time, **invalid)
        assert not result["success"]
        assert result["held_seconds"] == 0


def test_drop_resets_hold_and_success_is_not_latched():
    check = HoldCheck()
    assert not tick(check, 0)["success"]
    assert tick(check, 1)["success"]
    assert not tick(check, 1.1, right=False)["success"]
    assert not tick(check, 1.2)["success"]
    assert not tick(check, 2.1)["success"]
    assert tick(check, 2.2)["success"]


def test_unobserved_gap_clock_reset_and_duplicate_time():
    check = HoldCheck(max_sample_gap=0.15)
    for index in range(11):
        result = tick(check, index / 10)
    assert result["success"]
    assert tick(check, 1)["held_seconds"] == 1
    assert not tick(check, 3)["success"]
    assert not tick(check, 0)["success"]
    assert not tick(check, math.nan)["success"]


@pytest.mark.parametrize("field,value", [("minimum_hold_seconds", 0), ("minimum_lift_metres", -1),
    ("navigation_position_tolerance_metres", math.inf), ("maximum_navigation_collision_events", 1.5),
    ("minimum_hold_seconds", True)])
def test_invalid_thresholds_rejected(field, value):
    with pytest.raises(ValueError):
        Criteria(**{field: value})


def test_shared_config_rejects_relaxed_contact_requirements(tmp_path):
    config = tmp_path / "task.yaml"
    config.write_text("minimum_lift_metres: 0.05\nminimum_hold_seconds: 1\nrequire_both_fingers: false\n")
    with pytest.raises(ValueError, match="must remain true"):
        load_criteria(config)


def test_navigation_distance_contacts_and_wrapped_heading():
    telemetry = Telemetry()
    telemetry.sample(0, [0, 0, 0])
    telemetry.set_phase("navigate", 0)
    telemetry.sample(1, [0.3, 0.4, 0], [("robot", "table")])
    telemetry.sample(2, [0.6, 0.8, 0], [("robot", "table")])
    telemetry.sample(3, [0.6, 0.8, 0])
    telemetry.sample(4, [0.6, 0.8, -math.pi + 0.01], [("robot", "table")])
    telemetry.set_phase("pregrasp", 4)
    telemetry.sample(5, [0.7, 0.8, 0], [("robot", "cup")])
    result = telemetry.result([0.6, 0.8, math.pi - 0.01], Criteria())
    assert result["navigation_seconds"] == 4
    assert result["navigation_distance_metres"] == pytest.approx(1)
    assert result["navigation_collision_events"] == 2
    assert result["navigation_contact_samples"] == 3
    assert result["navigation_heading_error_radians"] == pytest.approx(0.02)
    assert result["navigation_success"]


def test_missing_navigation_not_silently_zero_or_success():
    result = Telemetry().result([0, 0, 0], Criteria())
    assert result["navigation_seconds"] is None
    assert not result["navigation_success"]


def test_second_navigation_must_complete_independently():
    telemetry = Telemetry()
    telemetry.set_phase("navigate", 0)
    telemetry.sample(1, [0, 0, 0])
    telemetry.set_phase("pregrasp", 1)
    assert telemetry.result([0, 0, 0], Criteria())["navigation_success"]
    telemetry.set_phase("navigate", 1)
    assert not telemetry.result([0, 0, 0], Criteria())["navigation_success"]


@pytest.mark.parametrize("grasp,extra,expected", [
    ({"environment_contact": True}, {}, "object_supported"),
    ({"left_contact": False}, {}, "missing_finger_contact"),
    ({"left_contact": False}, {"ever_grasp_success": True}, "dropped_after_grasp"),
    ({"left_contact": True, "right_contact": True, "lift_metres": 0.01}, {}, "insufficient_lift"),
    ({"left_contact": True, "right_contact": True, "lift_metres": 0.1}, {}, "insufficient_hold"),
    ({"state_finite": False}, {}, "non_finite_state"),
])
def test_structured_failure_classification(grasp, extra, expected):
    assert failure_reason({"navigation_success": True, "collision_free": True,
                           "criteria": Criteria().to_dict(), "grasp": grasp, **extra}) == expected

"""Task identities and disclosed information privileges (not a security sandbox)."""
from copy import deepcopy

MOBILE = "mobile_pick_hold"
PLACE = "fixed_dock_pick_place"
TASKS = {MOBILE, PLACE}
TRACKS = {"feasible", "safe_rejection"}

BUILTINS = {
    "scripted_oracle_baseline": {
        "entrypoint": "tasks.pick_red_cup:run", "tasks": [MOBILE],
        "tracks": ["feasible"], "targets": ["red_cup_main"],
        "information_class": "oracle", "base_pose": "simulator_truth",
        "target_pose": "simulator_truth", "scene_information": ["navigation_waypoints"],
        "observations": ["joint_state", "optional_rgb"],
        "audit_status": "source_reviewed", "audit_notes": "Reads get_state(), get_object_pose() and metadata waypoints; no VLA.",
    },
    "g1_tcp_retarget": {
        "entrypoint": "benchmark_adapters:g1", "tasks": [PLACE],
        "tracks": ["feasible"], "targets": ["g1_bottle"],
        "information_class": "oracle", "base_pose": "simulator_truth",
        "target_pose": "simulator_truth", "scene_information": ["fixed_dock", "tcp_replay", "contact_truth"],
        "observations": ["joint_state"], "audit_status": "source_reviewed",
        "audit_notes": "Uses target truth to align replay, simulator TCP for IK and contact truth for release; no navigation or VLA.",
    },
    "oracle_dock_probe": {
        "entrypoint": "benchmark_adapters:dock_probe", "tasks": [MOBILE],
        "tracks": ["safe_rejection"], "targets": ["red_cup_main", "reference_jar"],
        "information_class": "oracle", "base_pose": "simulator_truth",
        "target_pose": "unused", "scene_information": ["nominal_dock", "collision_geometry"],
        "observations": ["joint_state"], "audit_status": "source_reviewed",
        "audit_notes": "Conservative static-grid nominal-dock reachability probe; stops without moving to goal. Does not read condition labels and is not a manipulation controller.",
    },
}


def load_strategies(path=None):
    from .protocol import read_json
    registry = deepcopy(BUILTINS)
    if path is None:
        return registry
    raw = read_json(path)
    if not isinstance(raw, dict) or raw.get("schema_version") != "benchmark-strategies-1.0":
        raise ValueError("Expected benchmark-strategies-1.0")
    if not isinstance(raw.get("strategies"), dict) or not raw["strategies"]:
        raise ValueError("Strategies must be a nonempty mapping")
    for name, spec in raw["strategies"].items():
        if not isinstance(name, str) or not name.strip():
            raise ValueError("Strategy names must be nonempty strings")
        if name in registry:
            raise ValueError(f"Cannot replace built-in declaration: {name}")
        required = {"entrypoint", "tasks", "tracks", "targets", "information_class", "base_pose", "target_pose",
                    "scene_information", "observations", "audit_status", "audit_notes"}
        if not isinstance(spec, dict) or set(spec) != required:
            raise ValueError(f"Incomplete strategy declaration: {name}")
        for key in ("tasks", "tracks", "targets", "scene_information", "observations"):
            if not isinstance(spec[key], list) or not all(isinstance(s, str) and s for s in spec[key]):
                raise ValueError(f"Invalid {key} for {name}")
        if not spec["tasks"] or not set(spec["tasks"]) <= TASKS or not spec["tracks"] or not set(spec["tracks"]) <= TRACKS or not spec["targets"]:
            raise ValueError(f"Invalid supported tasks/tracks/targets for {name}")
        if spec["information_class"] not in {"oracle", "estimated_state", "sensor_only", "undeclared"}:
            raise ValueError(f"Invalid information class for {name}")
        if spec["audit_status"] not in {"declared_only", "source_reviewed"}:
            raise ValueError(f"Invalid audit status for {name}")
        for key in required - {"tasks", "tracks", "targets", "scene_information", "observations"}:
            if not isinstance(spec[key], str) or not spec[key]:
                raise ValueError(f"Invalid {key} for {name}")
        if spec["information_class"] == "sensor_only" and (spec["base_pose"] == "simulator_truth" or spec["target_pose"] == "simulator_truth"):
            raise ValueError("sensor_only cannot declare simulator truth")
        registry[name] = spec
    return registry


def compatibility(condition, spec):
    if condition.get("availability", "ready") != "ready":
        return condition.get("pending_reason", "controller_pending")
    if condition["task"] not in spec["tasks"] or condition["track"] not in spec["tracks"]:
        return "unsupported_task_or_track"
    if "*" not in spec["targets"] and condition["target_body"] not in spec["targets"]:
        return "unsupported_target"
    return None


def audit_access(spec, calls):
    findings = []
    if spec.get("information_class", "undeclared") == "undeclared":
        findings.append("information_permissions_not_declared")
    if spec.get("base_pose") not in {None, "simulator_truth"} and calls.get("get_state_simulator_base_pose", 0):
        findings.append("truth_base_pose_api_called; review consumption including helper calls")
    if spec.get("target_pose") not in {None, "simulator_truth"} and calls.get("get_object_pose_truth", 0):
        findings.append("truth_object_pose_api_called")
    return {"declaration": spec, "public_api_calls": dict(calls), "findings": findings,
            "review_required": bool(findings), "enforcement": "audit_only",
            "direct_model_data_access_tracked": False}

"""Versioned scene handoff, reproducibility metadata and atomic result files."""
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def read_json(path):
    def reject(value):
        raise ValueError(f"Non-standard JSON number: {value}")
    return json.loads(Path(path).read_text(encoding="utf-8-sig"), parse_constant=reject)


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def provenance():
    def git(*args):
        try:
            return subprocess.check_output(["git", "-C", str(ROOT), *args], stderr=subprocess.DEVNULL,
                                           timeout=10).decode("utf-8", errors="replace").strip()
        except (OSError, subprocess.SubprocessError):
            return None
    status = git("status", "--porcelain")
    hashes = {str(path.relative_to(ROOT)).replace("\\", "/"): sha256(path)
              for directory in ("05_benchmark", "configs", "01_navigation", "02_manipulation_vla", "07_extension")
              for path in sorted((ROOT / directory).rglob("*")) if path.suffix in (".py", ".yaml")
              and not {".test_deps", "artifacts", "__pycache__"}.intersection(path.parts)}
    return {"git_commit": git("rev-parse", "HEAD"), "git_dirty": None if status is None else bool(status),
            "git_status": status, "python": sys.version, "python_executable": sys.executable,
            "source_sha256": hashes, "requirements_lock_sha256": sha256(ROOT / "requirements.lock")}


def load_suite(manifest=None):
    from .tasks import MOBILE, PLACE, TASKS, TRACKS
    version = "benchmark-suite-1.0"
    if manifest:
        manifest = Path(manifest).resolve()
        raw = read_json(manifest)
        if not isinstance(raw, dict) or raw.get("schema_version") not in {"benchmark-suite-1.0", "benchmark-suite-2.0"}:
            raise ValueError("Expected benchmark-suite-1.0 or benchmark-suite-2.0")
        version = raw["schema_version"]
        conditions, base = raw.get("conditions"), manifest.parent
    else:
        base = ROOT
        conditions = [{"id": "fixed-kitchen", "scene": "scenes/kitchen/generated/kitchen.xml",
                       "metadata": "scenes/kitchen/generated/metadata.json"}]
    if not isinstance(conditions, list) or not conditions:
        raise ValueError("Suite must contain conditions")
    seen, resolved = set(), []
    for condition in conditions:
        if not isinstance(condition, dict):
            raise ValueError("Each condition must be an object")
        identifier = condition.get("id")
        if not isinstance(identifier, str) or not identifier or identifier in seen:
            raise ValueError("Condition IDs must be nonempty and unique")
        seen.add(identifier)
        extra = {"task", "track", "split", "layout", "object", "variation_axis", "availability", "pending_reason",
                 "instruction", "expected_nominal_dock_reachable", "resources", "source_condition"} if version.endswith("2.0") else set()
        if set(condition) - ({"id", "scene", "metadata", "seed", "description"} | extra):
            raise ValueError(f"Unsupported fields for {identifier}")
        scene, metadata = (base / condition[key] for key in ("scene", "metadata"))
        scene, metadata = scene.resolve(), metadata.resolve()
        if not scene.is_file() or not metadata.is_file():
            raise ValueError(f"Missing scene/metadata for {identifier}: {scene}, {metadata}")
        info = read_json(metadata)
        if not isinstance(info, dict):
            raise ValueError(f"Metadata must be an object for {identifier}")
        seed = condition.get("seed", info.get("seed"))
        if type(seed) is not int or type(info.get("seed")) is not int or not 0 <= seed < 2**32 or info.get("seed") != seed:
            raise ValueError(f"Scene seed must match metadata for {identifier}")
        task, track = condition.get("task", MOBILE), condition.get("track", "feasible")
        if task not in TASKS or track not in TRACKS or (task == PLACE and track != "feasible"):
            raise ValueError(f"Invalid task/track for {identifier}")
        split = condition.get("split", "unspecified")
        if split not in {"unspecified", "development", "test"}:
            raise ValueError(f"Invalid split for {identifier}")
        availability = condition.get("availability", "ready")
        if availability not in {"ready", "pending_controller"}:
            raise ValueError(f"Invalid availability for {identifier}")
        if availability != "ready" and not condition.get("pending_reason"):
            raise ValueError(f"Pending condition requires a reason: {identifier}")
        if track == "safe_rejection" and condition.get("expected_nominal_dock_reachable") is not False:
            raise ValueError("safe_rejection requires an explicitly blocked nominal dock")
        if track == "feasible" and condition.get("expected_nominal_dock_reachable") is False:
            raise ValueError("Blocked nominal dock must use safe_rejection track")
        waypoints = info.get("navigation_waypoints", [])
        if not isinstance(waypoints, list) or (task == MOBILE and not waypoints):
            raise ValueError(f"Missing navigation_waypoints for {identifier}")
        for pose in waypoints:
            if not isinstance(pose, list) or len(pose) != 3 or not all(
                    type(value) in (int, float) and math.isfinite(value) for value in pose):
                raise ValueError(f"Invalid waypoint for {identifier}")
        if not isinstance(info.get("target_body"), str) or not info["target_body"]:
            raise ValueError(f"Missing target_body for {identifier}")
        resources = {}
        for name, filename in condition.get("resources", {}).items():
            if name not in {"dock", "replay"} or not isinstance(filename, str):
                raise ValueError(f"Invalid task resource for {identifier}")
            path = (base / filename).resolve()
            if not path.is_file():
                raise ValueError(f"Missing {name} resource: {path}")
            resources[name] = {"path": str(path), "sha256": sha256(path)}
        if task == PLACE and (info["target_body"] != "g1_bottle" or set(resources) != {"dock", "replay"}):
            raise ValueError("G1 placement requires g1_bottle, dock and replay")
        resolved.append({**{key: condition[key] for key in extra if key in condition},
                         "id": identifier, "scene": str(scene), "metadata": str(metadata), "seed": seed,
                         "task": task, "track": track, "split": split, "availability": availability,
                         "target_body": info["target_body"], "resources": resources,
                         "scene_sha256": sha256(scene), "metadata_sha256": sha256(metadata)})
    return resolved

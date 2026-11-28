"""TEAM-05: isolated, reproducible multi-condition and multi-strategy evaluation."""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from project_bootstrap import activate
activate()
from configs import load
from evaluation.criteria import load_criteria
from evaluation.protocol import load_suite, provenance, read_json, sha256, write_json
from evaluation.reporting import compare_reports, save_report, summarize
from evaluation.tasks import MOBILE, PLACE, load_strategies, compatibility
from evaluation.placement import PlacementCriteria


def stop_process(process):
    if os.name == "nt":
        subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    else:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    if process.poll() is None:
        process.kill()
    process.wait()


def run_child(command, log_path, timeout, env=None):
    with Path(log_path).open("w", encoding="utf-8") as log:
        try:
            process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                       env=env, start_new_session=os.name != "nt")
        except OSError as exc:
            log.write(str(exc))
            return 127, "launch_error"
        try:
            return process.wait(timeout=timeout), None
        except subprocess.TimeoutExpired:
            stop_process(process)
            return 124, "wall_clock_timeout"
        except KeyboardInterrupt:
            stop_process(process)
            return 130, "interrupted"


def collect_result(episode, exit_code, reason, criteria, condition=None, rejection_criteria=None):
    metrics, phase = None, None
    try:
        progress = read_json(episode / "progress.json")
        phase = progress.get("phase")
    except (OSError, ValueError, AttributeError):
        pass
    try:
        metrics = read_json(episode / "metrics.json")
        if (not isinstance(metrics, dict) or metrics.get("schema_version") not in {"benchmark-episode-1.0", "benchmark-episode-2.0"}
                or type(metrics.get("task_success")) is not bool or metrics.get("criteria") != criteria
                or type(metrics.get("success")) is not bool
                or metrics.get("reason") is not None and not isinstance(metrics["reason"], str)):
            raise ValueError("Invalid metrics schema or criteria")
        kind, track = metrics.get("task", MOBILE), metrics.get("track", "feasible")
        if condition and (kind != condition["task"] or track != condition["track"]):
            raise ValueError("Metrics task/track mismatch")
        if kind not in {MOBILE, PLACE} or track not in {"feasible", "safe_rejection"}:
            raise ValueError("Unknown metrics task/track")
        if track == "safe_rejection":
            rejection = metrics.get("rejection", {})
            if rejection_criteria is not None and rejection.get("criteria") != rejection_criteria:
                raise ValueError("Safe-rejection criteria mismatch")
            if (kind != MOBILE or metrics["task_success"] is not False
                    or metrics["success"] is not metrics.get("safe_rejection_success")
                    or metrics["success"] is not rejection.get("success")
                    or metrics.get("navigation_success") is not None or metrics.get("grasp_success") is not None):
                raise ValueError("Invalid negative-track metrics")
            if metrics["success"] and (rejection.get("explicit_rejection") is not True or rejection.get("stopped") is not True
                                       or rejection.get("collision_events") != 0):
                raise ValueError("Unsafe rejection cannot pass")
        elif kind == PLACE:
            placement = metrics.get("placement", {})
            if (metrics["success"] is not metrics["task_success"] or metrics.get("navigation_success") is not None
                    or type(metrics.get("grasp_success")) is not bool or type(metrics.get("placement_success")) is not bool
                    or placement.get("criteria") != PlacementCriteria().to_dict()):
                raise ValueError("Invalid placement metrics")
            if metrics["success"] and (not metrics["grasp_success"] or not metrics["placement_success"] or placement.get("success") is not True):
                raise ValueError("Placement success contradicts component metrics")
        else:
            if (metrics["success"] is not metrics["task_success"]
                    or any(type(metrics.get(key)) is not bool for key in ("navigation_success", "grasp_success"))):
                raise ValueError("Invalid mobile metrics")
            if metrics["task_success"] and (not metrics["navigation_success"] or not metrics["grasp_success"] or metrics.get("collision_free") is not True):
                raise ValueError("Task success contradicts component metrics")
        if metrics["success"] and metrics.get("error") is not None:
            raise ValueError("Execution errors cannot pass")
        phase = metrics.get("last_phase", phase)
    except FileNotFoundError:
        reason = reason or ("setup_error" if (episode / "error.json").exists() else "missing_metrics")
    except (OSError, ValueError, KeyError, TypeError):
        metrics = None
        reason = reason or "invalid_metrics"
    success = reason is None and exit_code == 0 and metrics is not None and metrics["success"]
    if not success and reason is None:
        reason = (metrics or {}).get("reason") or ("nonzero_exit" if exit_code else "task_failed")
    return {"success": bool(success), "exit_code": exit_code, "reason": reason,
            "failure_stage": None if success else phase, "metrics": metrics}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=1, help="Repeats per condition/strategy/policy seed")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--with-images", action="store_true")
    parser.add_argument("--manifest", type=Path, help="benchmark-suite-1.0 or 2.0 JSON")
    parser.add_argument("--strategies", nargs="+", help="Declared strategy names; default: compatible built-ins")
    parser.add_argument("--strategy-file", type=Path, help="benchmark-strategies-1.0 declarations for custom adapters")
    parser.add_argument("--seeds", nargs="+", type=int, default=[7], help="Policy RNG seeds; does not regenerate scenes")
    parser.add_argument("--timeout", type=float, default=300, help="Wall-clock seconds per isolated episode")
    parser.add_argument("--preflight", action="store_true", help="Validate and print plan without writing or simulating")
    parser.add_argument("--compare", type=Path, help="Previous summary.json with identical experiment matrix")
    args = parser.parse_args()
    if args.episodes < 1 or args.output.exists() or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("Require positive --episodes/--timeout and a new --output directory")
    if len(set(args.seeds)) != len(args.seeds) or any(not 0 <= seed < 2**32 for seed in args.seeds):
        parser.error("Policy seeds must be unique integers in [0, 2**32)")
    try:
        registry = load_strategies(args.strategy_file)
        strategies = args.strategies or ["scripted_oracle_baseline", "g1_tcp_retarget", "oracle_dock_probe"]
        if len(set(strategies)) != len(strategies):
            raise ValueError("Strategies must be unique")
        for name in strategies:
            if name not in registry:
                # Legacy module:function remains usable, but can never be presented as sensor-only.
                if not re.fullmatch(r"[A-Za-z_]\w*(?:\.[A-Za-z_]\w*)*:[A-Za-z_]\w*", name):
                    raise ValueError(f"Unknown strategy: {name}; supply --strategy-file")
                registry[name] = {"entrypoint": name, "tasks": [MOBILE], "tracks": ["feasible"], "targets": ["*"],
                                  "information_class": "undeclared", "audit_status": "declared_only",
                                  "audit_notes": "Legacy adapter without information declaration"}
            if not re.fullmatch(r"[A-Za-z_]\w*(?:\.[A-Za-z_]\w*)*:[A-Za-z_]\w*", registry[name]["entrypoint"]):
                raise ValueError(f"Invalid entrypoint for {name}")
        conditions = load_suite(args.manifest)
        criteria = load_criteria().to_dict()
        task_config = load("task")
        task_config_hash = sha256(ROOT / "configs/task.yaml")
        previous = read_json(args.compare) if args.compare else None
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    matrix, exclusions = [], []
    for condition in conditions:
        for strategy in strategies:
            unsupported = compatibility(condition, registry[strategy])
            if unsupported:
                exclusions.append({"condition_id": condition["id"], "strategy": strategy, "reason": unsupported})
                continue
            matrix.extend({"condition": condition, "strategy": strategy, "strategy_spec": registry[strategy],
                           "policy_seed": seed, "repeat": repeat}
                          for seed in args.seeds for repeat in range(args.episodes))
    if not matrix:
        parser.error("No compatible runnable condition/strategy pairs; pending or unsupported cases are not scores")
    rejection_criteria = task_config.get("safe_rejection")
    if any(item["condition"]["track"] == "safe_rejection" for item in matrix):
        from evaluation.rejection import StopCheck
        try:
            StopCheck(20, rejection_criteria or {})
        except ValueError as exc:
            parser.error(f"Safe-rejection criteria must be confirmed/configured: {exc}")
    variation = len({condition["scene_sha256"] for condition in conditions}) > 1
    report = {"schema_version": "benchmark-report-2.0", "created_utc": datetime.now(timezone.utc).isoformat(),
              "instruction": task_config["instruction"], "with_images": args.with_images, "timeout_seconds": args.timeout,
              "status": "running", "planned_episodes": len(matrix), "criteria": criteria, "conditions": conditions,
              "scene_variation": variation, "controller": strategies[0] if len(strategies) == 1 else "multiple",
              "strategy_declarations": {name: registry[name] for name in strategies}, "excluded_pairs": exclusions,
              "placement_criteria": PlacementCriteria().to_dict(), "rejection_criteria": rejection_criteria,
              "operation_collision_policy": "report_only", "simulation_budget": "not_frozen",
              "note": ("Multiple scene XML hashes; variation must still be audited by TEAM-03."
                       if variation else "Repeated fixed scene; measures repeatability, not cross-scene generalization"),
              "episodes": [], "manifest": str(args.manifest.resolve()) if args.manifest else None}
    if previous is not None:
        preview = [{"condition_id": item["condition"]["id"], "strategy": item["strategy"], "policy_seed": item["policy_seed"],
                    "repeat": item["repeat"], "success": False, "reason": "preflight"} for item in matrix]
        try:
            compare_reports({**report, "episodes": preview, "success_rate": 0}, previous)
        except (ValueError, KeyError, TypeError) as exc:
            parser.error(str(exc))
    if args.preflight:
        print(json.dumps({"planned_episodes": len(matrix), "conditions": conditions, "criteria": criteria,
                          "excluded_pairs": exclusions, "strategy_declarations": report["strategy_declarations"],
                          "scene_variation": variation, "note": report["note"]}, ensure_ascii=False, indent=2))
        return 0
    args.output.mkdir(parents=True)
    report["provenance"] = provenance()
    write_json(args.output / "plan.json", {**report, "matrix": matrix})
    rows = []
    for index, item in enumerate(matrix):
        episode = args.output.resolve() / f"episode_{index:04d}"
        episode.mkdir()
        request = {**item, "criteria": criteria, "with_images": args.with_images,
                   "instruction": item["condition"].get("instruction", task_config["instruction"]),
                   "rejection_criteria": rejection_criteria, "task_config_sha256": task_config_hash}
        write_json(episode / "request.json", request)
        command = [sys.executable, str(ROOT / "05_benchmark/run_episode.py"), "--request", str(episode / "request.json")]
        write_json(episode / "command.json", command)
        start = time.monotonic()
        env = {**os.environ, "PYTHONHASHSEED": str(item["policy_seed"]), "PYTHONUNBUFFERED": "1"}
        exit_code, reason = run_child(command, episode / "run.log", args.timeout, env)
        row = {"episode": index, "condition_id": item["condition"]["id"], "strategy": item["strategy"],
               "task": item["condition"]["task"], "track": item["condition"]["track"], "split": item["condition"]["split"],
               "information_class": item["strategy_spec"]["information_class"], "audit_status": item["strategy_spec"]["audit_status"],
               "scene_seed": item["condition"]["seed"], "policy_seed": item["policy_seed"], "repeat": item["repeat"],
               "wall_seconds": time.monotonic() - start, **collect_result(episode, exit_code, reason, criteria, item["condition"], rejection_criteria)}
        rows.append(row)
        stats = summarize(rows)
        report.update(episodes=rows, statistics=stats, success_rate=stats["success_rate"])
        if reason == "interrupted":
            report["status"] = "interrupted"
        elif index == len(matrix) - 1:
            report["status"] = "complete"
        save_report(args.output, report)
        print(json.dumps({"episode": index, "success": row["success"], "reason": row["reason"]}), flush=True)
        if reason == "interrupted":
            return 130
    if previous is not None:
        report["comparison"] = compare_reports(report, previous)
        save_report(args.output, report)
    print(json.dumps({"episodes": len(rows), "success_rate": report["success_rate"]}))
    return 0 if all(row["success"] for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())

"""Failure-inclusive summaries, readable reports and paired regression checks."""
from collections import Counter
import csv
import math
from pathlib import Path
import statistics

from .protocol import write_json


def wilson(successes, count):
    if not count:
        return None
    quantile = 1.959963984540054
    rate, denominator = successes / count, 1 + quantile * quantile / count
    center = (rate + quantile * quantile / (2 * count)) / denominator
    half = quantile * math.sqrt(rate * (1 - rate) / count + quantile * quantile / (4 * count * count)) / denominator
    return [max(0, center - half), min(1, center + half)]


def aggregate(rows):
    successes = sum(row["success"] for row in rows)
    result = {"episodes": len(rows), "successes": successes,
              "success_rate": successes / len(rows) if rows else None, "success_rate_wilson95": wilson(successes, len(rows)),
              "failures": dict(sorted(Counter(row["reason"] for row in rows if not row["success"]).items()))}
    for field in ("navigation_success", "grasp_success", "placement_success", "safe_rejection_success"):
        applicable = [row for row in rows if (
            row.get("track", "feasible") == "safe_rejection" if field == "safe_rejection_success" else
            row.get("track", "feasible") == "feasible" and (field == "grasp_success" or
            row.get("task", "mobile_pick_hold") == ("fixed_dock_pick_place" if field == "placement_success" else "mobile_pick_hold")))]
        result[field + "_rate"] = sum((row.get("metrics") or {}).get(field) is True for row in applicable) / len(applicable) if applicable else None
        result[field + "_attempts"] = len(applicable)
    for field in ("navigation_seconds", "navigation_distance_metres", "simulation_seconds", "navigation_collision_events", "operation_collision_events"):
        values = [(row.get("metrics") or {}).get(field) for row in rows]
        values = [value for value in values if type(value) in (int, float) and math.isfinite(value)]
        result[field] = {"observed_episodes": len(values), "mean": statistics.mean(values) if values else None,
                         "median": statistics.median(values) if values else None,
                         "max": max(values) if values else None}
    condition_ids = {row.get("condition_id", "unknown") for row in rows}
    rates = [statistics.mean(row["success"] for row in rows if row.get("condition_id", "unknown") == key) for key in condition_ids]
    result["condition_count"] = len(condition_ids)
    result["condition_equal_weight_success_rate"] = statistics.mean(rates) if rates else None
    result["information_audit_review_episodes"] = sum(
        (row.get("metrics") or {}).get("information_audit", {}).get("review_required") is True for row in rows)
    return result


def summarize(rows):
    feasible = [row for row in rows if row.get("track", "feasible") == "feasible"]
    result = aggregate(feasible)
    result["attempted_checks"] = len(rows)
    result["success_rate_scope"] = "feasible_only; mixed-task overview, use primary_groups for comparisons"
    negatives = [row for row in rows if row.get("track") == "safe_rejection"]
    result["safe_rejection"] = aggregate(negatives)
    for key in ("strategy", "condition_id", "policy_seed"):
        result["by_" + key] = {value: aggregate([row for row in feasible if row[key] == value])
                               for value in sorted({row[key] for row in feasible})}
    axes = ("strategy", "task", "track", "split", "information_class", "audit_status")
    defaults = ("unknown", "mobile_pick_hold", "feasible", "unspecified", "undeclared", "declared_only")
    groups = {}
    for row in rows:
        key = tuple(row.get(axis, default) for axis, default in zip(axes, defaults))
        groups.setdefault(key, []).append(row)
    result["primary_groups"] = [{**dict(zip(axes, key)), **aggregate(values)} for key, values in sorted(groups.items())]
    return result


def compare_reports(current, previous):
    def identities(report):
        conditions = {condition["id"]: condition for condition in report["conditions"]}
        def semantic(condition):
            import json
            fields = ("task", "track", "split", "target_body", "instruction", "resources", "availability", "source_condition")
            return json.dumps({key: condition.get(key) for key in fields}, sort_keys=True)
        return {(row["condition_id"], conditions[row["condition_id"]]["scene_sha256"],
                 conditions[row["condition_id"]]["metadata_sha256"], semantic(conditions[row["condition_id"]]),
                 row["strategy"], row["policy_seed"], row["repeat"]): row
                for row in report["episodes"]}
    if (previous.get("schema_version") not in {"benchmark-report-1.0", "benchmark-report-2.0"}
            or previous.get("schema_version") != current.get("schema_version") or previous.get("criteria") != current["criteria"]):
        raise ValueError("Regression comparison requires matching schema and identical criteria")
    for field in ("instruction", "with_images", "timeout_seconds", "strategy_declarations", "placement_criteria",
                  "rejection_criteria", "operation_collision_policy", "simulation_budget"):
        if previous.get(field) != current.get(field):
            raise ValueError(f"Regression comparison requires identical {field}")
    old, new = identities(previous), identities(current)
    if old.keys() != new.keys():
        raise ValueError("Regression comparison requires identical conditions, scene hashes, strategies, seeds and repeats")
    regressions = [{"condition_id": new[key]["condition_id"], "strategy": new[key]["strategy"],
                    "policy_seed": new[key]["policy_seed"], "repeat": new[key]["repeat"], "reason": new[key]["reason"]}
                   for key in new if old[key]["success"] and not new[key]["success"]]
    return {"paired_episodes": len(new), "regressions": regressions, "regression_count": len(regressions),
            "success_rate_change": (current["success_rate"] - previous["success_rate"]
                                    if current["success_rate"] is not None and previous["success_rate"] is not None else None)}


def save_report(output, report):
    output = Path(output)
    write_json(output / "summary.json", report)
    fields = ["episode", "condition_id", "strategy", "task", "track", "split", "information_class", "audit_status", "scene_seed", "policy_seed", "repeat", "success",
              "reason", "exit_code", "failure_stage", "wall_seconds", "navigation_seconds",
              "navigation_distance_metres", "navigation_collision_events", "navigation_success", "grasp_success",
              "placement_success", "safe_rejection_success", "operation_collision_events"]
    with (output / "episodes.csv").open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in report["episodes"]:
            merged = {**(row.get("metrics") or {}), **row}
            writer.writerow({key: ("'" + value if isinstance(value, str) and value.startswith(("=", "+", "-", "@")) else value)
                             for key, value in merged.items() if key in fields})
    stats = report["statistics"]
    def percent(value):
        return "N/A" if value is None else f"{value:.1%}"
    def cell(value):
        return str(value).replace("|", "/").replace("\n", " ")
    lines = ["# TEAM-05 Benchmark report", "", f"Status: {report['status']}",
             f"Completed / planned: {len(report['episodes'])} / {report['planned_episodes']}",
             f"Feasible-task overview only: {stats['successes']} / {stats['episodes']} ({percent(stats['success_rate'])})",
             f"Safe-rejection checks (separate): {stats['safe_rejection']['successes']} / {stats['safe_rejection']['episodes']}",
             "", report["note"], "",
             "The interval is descriptive: deterministic repeats are not independent generalization trials.",
             "Navigation/grasp rates include runtime failures in their denominator. Missing numeric observations are not zero.",
             "Contacts are sampled at control ticks; brief contacts between ticks may be missed.", "",
             "Information permissions are declarations/source review, not process isolation. Direct model/data access cannot be fully audited.",
             "Operation collisions are diagnostic only. Split=unspecified does not establish held-out testing.", "",
             "| Strategy | Task | Track | Split | Information / audit | Conditions | Attempts | Pass rate | Condition mean | Navigation | Grasp | Placement | Operation collisions (mean) |",
             "|---|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for group in stats["primary_groups"]:
        values = [group["strategy"], group["task"], group["track"], group["split"],
                  f"{group['information_class']} / {group['audit_status']}", group["condition_count"], group["episodes"],
                  percent(group["success_rate"]), percent(group["condition_equal_weight_success_rate"]),
                  percent(group["navigation_success_rate"]), percent(group["grasp_success_rate"]),
                  percent(group["placement_success_rate"]), group["operation_collision_events"]["mean"]]
        lines.append("| " + " | ".join(cell(value) for value in values) + " |")
    lines.extend(["", "## Failures", ""])
    failures = Counter(row["reason"] for row in report["episodes"] if not row["success"])
    lines.extend(f"- {cell(reason)}: {count}" for reason, count in failures.items())
    if not failures:
        lines.append("None.")
    lines.extend(["", "## Not evaluated (excluded before execution)", ""])
    lines.extend(f"- {cell(item['condition_id'])} / {cell(item['strategy'])}: {cell(item['reason'])}"
                 for item in report.get("excluded_pairs", []))
    lines.extend(["", "## Information declarations", ""])
    for name, spec in report.get("strategy_declarations", {}).items():
        lines.append(f"- {cell(name)}: {cell(spec.get('audit_notes', 'undeclared'))}")
    flagged = [row for row in report["episodes"] if (row.get("metrics") or {}).get("information_audit", {}).get("review_required")]
    lines.append(f"- Episodes requiring information-access review: {len(flagged)}")
    for row in flagged:
        lines.append(f"  - episode {row['episode']}: {cell(row['metrics']['information_audit']['findings'])}")
    if "comparison" in report:
        lines.extend(["", f"Paired regressions: {report['comparison']['regression_count']}"])
    (output / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

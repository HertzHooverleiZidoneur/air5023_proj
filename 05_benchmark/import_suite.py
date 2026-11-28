"""Convert existing TEAM-03 outputs into an explicit, unsplit TEAM-05 suite."""
import argparse
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from project_bootstrap import activate
activate()
from evaluation.protocol import read_json, sha256, write_json, load_suite
from evaluation.tasks import MOBILE, PLACE


def convert(output, kitchen_scene, kitchen_metadata, *, g1_scene=None, challenges=None):
    output = Path(output).resolve()
    if output.exists():
        raise ValueError("Use a new suite output directory")
    base_info = read_json(kitchen_metadata)
    if type(base_info.get("seed")) is not int or not base_info.get("navigation_waypoints"):
        raise ValueError("Kitchen metadata requires scene seed and navigation waypoints")
    entries, metadata = [], {}

    def relative(path):
        return os.path.relpath(Path(path).resolve(), output).replace("\\", "/")

    def add(identifier, scene, info, **fields):
        scene = Path(scene).resolve()
        if not scene.is_file():
            raise ValueError(f"Missing scene: {scene}")
        filename = f"metadata_{identifier}.json"
        metadata[filename] = info
        entries.append({"id": identifier, "scene": relative(scene), "metadata": filename, "seed": info["seed"],
                        "task": MOBILE, "track": "feasible", "split": "unspecified", "availability": "ready", **fields})

    add("fixed-kitchen", kitchen_scene, base_info, layout="kitchen_island", object="red_cup",
        variation_axis="reference", instruction="Navigate to the table and pick up the red cup.")
    if g1_scene:
        add("g1-fixed-dock", g1_scene, {"seed": base_info["seed"], "target_body": "g1_bottle"},
            task=PLACE, layout="g1_round_table", object="g1_bottle", variation_axis="reference",
            instruction="Pick up the bottle and place it stably on the round table from the fixed dock.",
            resources={"dock": relative(ROOT / "scenes/round_table/dock.json"),
                       "replay": relative(ROOT / "02_manipulation_vla/examples/g1_tcp_round_table/g1_tcp_source.npz")})
    if challenges:
        manifest = Path(challenges).resolve()
        raw = read_json(manifest)
        seen = set()
        for record in raw["conditions"]:
            identifier, table, case = record["id"], record["table"], record["case"]
            if table not in {"round", "island"} or case not in {"a", "b", "c", "d1", "d2", "e", "f1", "f2"}:
                raise ValueError("Unknown challenge table/case")
            if identifier != f"{table}_{case}" or identifier in seen:
                raise ValueError("Invalid or duplicate challenge ID")
            seen.add(identifier)
            scene = manifest.parent / record["scene"]
            if sha256(scene) != record["scene_sha256"] or record.get("static_and_settling_checks") != "passed":
                raise ValueError(f"Challenge hash/static check mismatch: {identifier}")
            if record.get("expected_nominal_dock_reachable") is not (case != "c"):
                raise ValueError(f"Challenge reachability label mismatch: {identifier}")
            if record["target_body"] != ("reference_jar" if table == "round" else "red_cup_main"):
                raise ValueError(f"Unexpected target: {identifier}")
            info = {"seed": base_info["seed"], "target_body": record["target_body"],
                    "navigation_waypoints": [record["nominal_dock_xyyaw"]]}
            pending = table == "round" and case != "c"
            add(identifier, scene, info, layout=table, object=record["target_body"], variation_axis=case,
                track="safe_rejection" if case == "c" else "feasible",
                availability="pending_controller" if pending else "ready",
                pending_reason="No reference_jar mobile grasp controller supplied" if pending else "",
                expected_nominal_dock_reachable=case != "c", source_condition=record,
                instruction=("Evaluate reachability of the specified nominal dock." if case == "c" else
                             "Navigate to the table and pick up the red cup." if table == "island" else
                             "Navigate to the round table and pick up the jar."))
    output.mkdir(parents=True)
    for name, info in metadata.items():
        write_json(output / name, info)
    target = output / "suite.json"
    write_json(target, {"schema_version": "benchmark-suite-2.0", "conditions": entries})
    load_suite(target)
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kitchen-scene", type=Path, required=True)
    parser.add_argument("--kitchen-metadata", type=Path, required=True)
    parser.add_argument("--g1-scene", type=Path)
    parser.add_argument("--challenges", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        path = convert(args.output, args.kitchen_scene, args.kitchen_metadata,
                       g1_scene=args.g1_scene, challenges=args.challenges)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    print(path)


if __name__ == "__main__":
    main()

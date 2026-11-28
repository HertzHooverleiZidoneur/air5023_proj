"""Build the round-table and RoboCasa island obstacle benchmarks."""

import argparse
import hashlib
import json
import math
from pathlib import Path
from tempfile import NamedTemporaryFile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from add_round_table import KITCHEN, ROOT, merge


CONFIG = ROOT / "configs/tabletop_challenges.json"


def vector(values):
    return " ".join(f"{value:.8g}" for value in values)


def add_prop(world, name, spec, surface_z):
    x, y = spec["xy"]
    dimensions = spec["size"]
    shape = spec["shape"]
    if shape == "box":
        width, depth, height = dimensions
        size = (width / 2, depth / 2, height / 2)
    elif shape == "cylinder":
        diameter, height = dimensions
        size = (diameter / 2, height / 2)
    else:
        raise ValueError(f"Unknown shape {shape!r} for {name}")

    # A 1 mm drop permits contact to settle without initial penetration.
    mobile = surface_z > 0
    z = surface_z + height / 2 + (0.001 if mobile else 0)
    body = ET.SubElement(world, "body", name=name, pos=vector((x, y, z)))
    if mobile:
        ET.SubElement(body, "freejoint")
    ET.SubElement(body, "geom", name=name, type=shape, size=vector(size),
                  rgba=vector((*spec["rgb"], 1)), friction="0.65 0.005 0.0001",
                  density="250", contype="1", conaffinity="1")


def validate(model, floor_props, tabletop_objects, support_geoms):
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_SENSOR)
    state = mujoco.MjData(model)
    mujoco.mj_forward(model, state)
    table_ids = {model.body(name).id for name in tabletop_objects}
    initial = {index: state.xpos[index].copy() for index in table_ids}
    for contact in state.contact:
        first, second = (int(model.geom_bodyid[g]) for g in contact.geom)
        if first != second and first in table_ids and second in table_ids:
            raise ValueError("Objects overlap at initialization")

    # Fixed fixtures do not collide with each other during ordinary simulation.
    # Check each added floor prop directly against the existing collision geoms.
    floor = set(floor_props)
    for name in floor:
        geom = model.geom(name).id
        for other in range(model.ngeom):
            other_name = model.geom(other).name
            if geom == other or not model.geom_contype[other] or "floor" in other_name:
                continue
            if mujoco.mj_geomDistance(model, state, geom, other, 0.001, None) < -0.001:
                raise ValueError(f"{name} overlaps {other_name or model.body(int(model.geom_bodyid[other])).name}")

    mujoco.mj_step(model, state, nstep=round(5 / model.opt.timestep))
    if not (np.isfinite(state.qpos).all() and np.isfinite(state.qvel).all()):
        raise ValueError("Tabletop settling produced a non-finite state")
    supported = set()
    for contact in state.contact:
        first, second = (int(g) for g in contact.geom)
        for supporter, object_geom in ((first, second), (second, first)):
            if model.geom(supporter).name.startswith(support_geoms):
                supported.add(int(model.geom_bodyid[object_geom]))
    for index, position in initial.items():
        if (index not in supported or np.linalg.norm(state.xpos[index][:2] - position[:2]) > 0.005
                or state.xmat[index, 8] < math.cos(math.radians(5))):
            raise ValueError(f"{model.body(index).name} did not settle upright on the table")


def build(kitchen, output, config):
    definitions = json.loads(config.read_text(encoding="utf-8"))
    cases = definitions["cases"]
    records = []
    generated = {}
    with NamedTemporaryFile(suffix=".xml", dir=kitchen.parent) as round_base:
        # Reuse the original reconstruction with its verified world placement.
        merge(kitchen, definitions["round_table_xy_m"], output=Path(round_base.name))
        for table, settings in definitions["tables"].items():
            base = kitchen if table == "island" else Path(round_base.name)
            variants = settings["conditions"]
            for case_id, design in cases.items():
                obstacles = variants["b"] | variants["d1"] if case_id == "e" else variants[case_id]
                scene = ET.parse(base)
                world = scene.getroot().find("worldbody")
                tabletop = settings["original_objects"].copy()
                floor = []
                for zone, surface_z, names in (("floor", 0, floor), ("table", settings["top_z"], tabletop)):
                    for index, prop in enumerate(obstacles.get(zone, []), start=1):
                        name = f"challenge_{zone}_{index}"
                        add_prop(world, name, prop, surface_z)
                        names.append(name)

                ET.indent(scene)
                content = ET.tostring(scene.getroot(), encoding="utf-8")
                filename = f"challenge_{table}_{case_id}.xml"
                with NamedTemporaryFile(suffix=".xml", dir=output) as candidate:
                    Path(candidate.name).write_bytes(content)
                    model = mujoco.MjModel.from_xml_path(candidate.name)
                try:
                    validate(model, floor, tabletop, settings["support_geom_prefix"])
                except ValueError as error:
                    raise ValueError(f"{filename}: {error}") from error

                generated[filename] = content
                record = {
                    "id": f"{table}_{case_id}",
                    "table": table,
                    "case": case_id,
                    "scene": filename,
                    "scene_sha256": hashlib.sha256(content).hexdigest(),
                    "target_body": settings["target_body"],
                    "design": design,
                    "expected_nominal_dock_reachable": case_id != "c",
                    "nominal_dock_xyyaw": settings["nominal_dock_xyyaw"],
                    "floor_obstacles": floor,
                    "tabletop_objects": tabletop,
                    "static_and_settling_checks": "passed",
                }
                if case_id in {"b", "e"}:
                    record["candidate_bypass_xy_m"] = settings["bypass_xy_m"]
                records.append(record)
                print(f"Validated {filename}", flush=True)

    for filename, content in generated.items():
        (output / filename).write_bytes(content)
    manifest = output / "tabletop_challenges.json"
    manifest.write_text(json.dumps({"settling_seconds": 5, "conditions": records}, indent=2) + "\n")
    print(f"Wrote {len(records)} conditions to {manifest}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kitchen", type=Path, default=KITCHEN)
    parser.add_argument("--config", type=Path, default=CONFIG)
    args = parser.parse_args()
    kitchen = args.kitchen.resolve()
    if not kitchen.is_file():
        parser.error("Build kitchen.xml with build_scene.py first")
    build(kitchen, kitchen.parent, args.config)


if __name__ == "__main__":
    main()

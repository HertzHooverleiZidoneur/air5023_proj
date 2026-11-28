"""Add the round-table reconstruction to the generated Stretch kitchen."""

import argparse
from copy import deepcopy
import hashlib
import json
from math import hypot, isfinite, radians
from pathlib import Path
import re
from tempfile import NamedTemporaryFile
import xml.etree.ElementTree as ET

import mujoco


ROOT = Path(__file__).resolve().parents[1]
KITCHEN = ROOT / "scenes/kitchen/generated/kitchen.xml"
ROUND_TABLE = ROOT / "scenes/round_table/round_table.xml"
PREFIX = "reference_"


def merge(kitchen, xy, *, output=None, offsets=None):
    scene = ET.parse(kitchen).getroot()
    table = ET.parse(ROUND_TABLE).getroot()
    compiler = scene.find("compiler")
    radians_in_kitchen = compiler is not None and compiler.get("angle") == "radian"
    table_friction = table.find("./default/geom").attrib["friction"]

    defaults = scene.find("default")
    if defaults is None:
        defaults = ET.SubElement(scene, "default")
    for definition in table.findall("./default/default"):
        definition = deepcopy(definition)
        definition.set("class", PREFIX + definition.attrib["class"])
        defaults.append(definition)

    world = scene.find("worldbody")
    offsets = offsets or {}
    for source in table.findall("./worldbody/body"):
        body = deepcopy(source)
        x, y, z = map(float, body.get("pos", "0 0 0").split())
        dx, dy = offsets.get(body.attrib["name"], (0, 0))
        body.set("pos", f"{x + xy[0] + dx:.8g} {y + xy[1] + dy:.8g} {z:.8g}")
        for element in body.iter():
            if "name" in element.attrib:
                element.set("name", PREFIX + element.attrib["name"])
            if "class" in element.attrib:
                element.set("class", PREFIX + element.attrib["class"])
            if element.tag == "geom":
                element.attrib.setdefault("friction", table_friction)
            if radians_in_kitchen and "euler" in element.attrib:
                angles = map(radians, map(float, element.attrib["euler"].split()))
                element.set("euler", " ".join(f"{angle:.12g}" for angle in angles))
        world.append(body)

    option = scene.find("option")
    if option is None:
        option = ET.SubElement(scene, "option")
    flags = option.find("flag")
    if flags is None:
        flags = ET.SubElement(option, "flag")
    # Flat-bottomed objects need multiple contacts on the round tabletop.
    flags.set("multiccd", "enable")

    output = output or kitchen.with_name("kitchen_round_table.xml")
    # Compile with the kitchen directory as the base for any relative assets.
    ET.indent(scene)
    with NamedTemporaryFile(suffix=".xml", dir=kitchen.parent) as staged:
        ET.ElementTree(scene).write(staged.name, encoding="utf-8")
        model = mujoco.MjModel.from_xml_path(staged.name)
        output.write_bytes(Path(staged.name).read_bytes())
    print(f"Loaded {model.ngeom} geoms and wrote {output}")
    return output


def check_tabletop(scene):
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    names = ("bottle", "jar", "package")
    initial = {name: data.body(PREFIX + name).xpos.copy() for name in names}
    object_ids = {model.body(PREFIX + name).id for name in names}
    for contact in data.contact:
        bodies = {int(model.geom_bodyid[geom]) for geom in contact.geom}
        if len(bodies & object_ids) == 2:
            raise ValueError(f"{scene.name}: tabletop objects overlap at initialization")
    mujoco.mj_step(model, data, nstep=round(5 / model.opt.timestep))
    tabletop = model.geom(PREFIX + "tabletop").id
    supported = set()
    for contact in data.contact:
        if tabletop in contact.geom:
            other = next(geom for geom in contact.geom if geom != tabletop)
            supported.add(int(model.geom_bodyid[other]))
    positions = {}
    for name in names:
        body = data.body(PREFIX + name)
        dx, dy, _ = body.xpos - initial[name]
        if body.id not in supported or hypot(dx, dy) > 0.001 or body.xmat[8] < 0.999:
            raise ValueError(f"{scene.name}: {name} did not settle upright on the tabletop")
        positions[name] = [float(value) for value in initial[name]]
    return positions


def build_conditions(kitchen, xy, config):
    cases = json.loads(config.read_text(encoding="utf-8"))
    if not cases:
        raise ValueError("Must specify at least one tabletop condition")
    records = []
    verified_scenes = {}
    for case_id, offsets in cases.items():
        if not re.fullmatch(r"[a-z0-9_]+", case_id):
            raise ValueError(f"Invalid scene id: {case_id}")
        for name, delta in offsets.items():
            if name not in {"bottle", "jar", "package"} or len(delta) != 2 or not all(map(isfinite, delta)):
                raise ValueError(f"{case_id}: invalid XY offset for {name}")
        output = kitchen.with_name(f"round_table_{case_id}.xml")
        with NamedTemporaryFile(suffix=".xml", dir=kitchen.parent) as staged:
            candidate = Path(staged.name)
            merge(kitchen, xy, output=candidate, offsets=offsets)
            positions = check_tabletop(candidate)
            contents = candidate.read_bytes()
        verified_scenes[output] = contents
        records.append({
            "id": case_id,
            "scene": output.name,
            "scene_sha256": hashlib.sha256(contents).hexdigest(),
            "offsets_xy_m": offsets,
            "initial_positions_m": positions,
        })
    for path, contents in verified_scenes.items():
        path.write_bytes(contents)
    manifest = kitchen.with_name("round_table_conditions.json")
    manifest.write_text(json.dumps({"table_xy_m": xy, "settling_seconds": 5, "conditions": records}, indent=2) + "\n")
    print(f"Verified {len(records)} tabletop conditions: {manifest}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kitchen", type=Path, default=KITCHEN)
    parser.add_argument("--xy", type=float, nargs=2, metavar=("X", "Y"), required=True)
    parser.add_argument("--conditions", type=Path, help="Generate a batch from a JSON file of tabletop offsets")
    args = parser.parse_args()
    if not args.kitchen.is_file():
        parser.error(f"Missing {args.kitchen}. Run 03_data_generate/build_scene.py first.")
    if args.conditions:
        build_conditions(args.kitchen.resolve(), args.xy, args.conditions)
    else:
        merge(args.kitchen.resolve(), args.xy)


if __name__ == "__main__":
    main()

"""Build the round-table variant from the installed RoboCasa kitchen."""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
ROUND_TABLE = ROOT / "scenes/round_table"
BASE_SCENE = ROOT / "scenes/kitchen/generated/kitchen.xml"
ROUND_SCENE = ROUND_TABLE / "generated/kitchen_round_table.xml"


def build_round_table(base_scene=BASE_SCENE, output=ROUND_SCENE):
    import mujoco

    base_scene, output = Path(base_scene).resolve(), Path(output).resolve()
    if output == base_scene:
        raise ValueError("The round-table output path must differ from the base scene")
    if not base_scene.is_file():
        raise FileNotFoundError("Generate the base kitchen first with 03_data_generate/build_scene.py")

    profile = json.loads((ROUND_TABLE / "scene.json").read_text())
    scene = ET.parse(base_scene).getroot()
    addon = ET.parse(ROUND_TABLE / "objects.xml").getroot()
    world, assets = scene.find("worldbody"), scene.find("asset")
    bodies = {body.get("name"): body for body in world.findall("body")}
    removed = set(profile["replace_bodies"])
    missing = (removed | {"red_cup_main"}) - bodies.keys()
    if missing:
        raise ValueError(f"Expected the configured RoboCasa kitchen, missing bodies: {sorted(missing)}")
    for name in removed:
        world.remove(bodies[name])
    cup_index = list(world).index(bodies["red_cup_main"])
    world.remove(bodies["red_cup_main"])
    world.insert(cup_index, addon.find("worldbody/body[@name='g1_bottle']"))
    world.append(addon.find("worldbody/body[@name='g1_round_table']"))
    keyframe = scene.find("keyframe")
    if keyframe is not None:
        scene.remove(keyframe)  # Saved vectors describe the old joint layout.

    compiler = scene.find("compiler")
    for asset in assets:
        filename = asset.get("file")
        if filename and not Path(filename).is_absolute():
            directory = "meshdir" if asset.tag == "mesh" else "texturedir"
            asset_dir = compiler.get(directory, compiler.get("assetdir", ""))
            asset.set("file", str((base_scene.parent / asset_dir / filename).resolve()))
    for asset in addon.find("asset"):
        if filename := asset.get("file"):
            asset.set("file", str((ROUND_TABLE / filename).resolve()))
        assets.append(asset)
    scene.set("model", "stretch_kitchen_g1_round_table")
    xml = ET.tostring(scene, encoding="unicode")
    model = mujoco.MjModel.from_xml_string(xml)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(xml + "\n", encoding="utf-8")
    print(f"Generated {output} ({model.ngeom} geoms)")
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-scene", type=Path, default=BASE_SCENE)
    parser.add_argument("--output", type=Path, default=ROUND_SCENE)
    args = parser.parse_args()
    build_round_table(args.base_scene, args.output)


if __name__ == "__main__":
    main()

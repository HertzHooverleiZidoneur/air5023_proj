"""TEAM-03: generate one RoboCasa kitchen through Stretch's integration."""
import argparse
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from project_bootstrap import activate
activate()
os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("NUMBA_CACHE_DIR", str(ROOT / ".cache/numba"))
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / ".cache/matplotlib"))


def adapt_cup_asset(destination):
    """Translate newer asset bbox metadata into the pinned v0.2 site schema.

    Only a generated copy changes. Collision and visual meshes stay official.
    """
    import numpy as np
    source = ROOT / "third_party/stretch_mujoco/third_party/robocasa/robocasa/models/assets/objects/objaverse/cup/cup_2/model.xml"
    root = ET.parse(source).getroot()
    bbox = root.find(".//geom[@name='reg_bbox']")
    if bbox is None:
        raise ValueError("Expected official cup_2 bbox metadata")
    center = np.fromstring(bbox.get("pos"), sep=" ")
    half = np.fromstring(bbox.get("size"), sep=" ")
    parent = root.find("worldbody/body")
    for name, pos in {"bottom_site": center - [0, 0, half[2]],
                      "top_site": center + [0, 0, half[2]],
                      "horizontal_radius_site": [half[0], half[1], 0]}.items():
        ET.SubElement(parent, "site", name=name, pos=" ".join(map(str, pos)),
                      size="0.001", rgba="0 0 0 0")
    for body in root.iter("body"):
        for geom in list(body.findall("geom")):
            if geom.get("class") == "region":
                body.remove(geom)  # nonphysical semantic volumes, not cup geometry
    for asset in root.find("asset"):
        if asset.get("file"):
            asset.set("file", str((source.parent / asset.get("file")).resolve()))
    destination.write_text(ET.tostring(root, encoding="unicode"))
    return destination


def build(seed=None):
    from configs import load
    config, robot_config = load("scene"), load("robot")
    seed = config["seed"] if seed is None else seed
    import mujoco
    import numpy as np
    from robocasa.environments.kitchen.kitchen import Kitchen
    from robocasa.models.objects.objects import MJCFObject
    from stretch_mujoco.robocasa_gen import model_generation_wizard

    out = ROOT / "scenes/kitchen/generated"
    out.mkdir(parents=True, exist_ok=True)
    cup_asset = adapt_cup_asset(out / "cup_2_legacy.xml")
    missing_visual_assets = []

    class StretchRedCupKitchen(Kitchen):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, seed=seed, obj_registries=("objaverse",), **kwargs)
            (out / "robocasa.xml").write_text(self.sim.model.get_xml())

        def _setup_kitchen_references(self):
            super()._setup_kitchen_references()
            self.table = self.register_fixture_ref("table", dict(id=config["table_fixture"]))
            self.init_robot_base_pos = self.table

        def _get_obj_cfgs(self):
            return [dict(name="red_cup", obj_groups="cup", graspable=True,
                         max_size=(0.075, 0.075, 0.12),
                         placement=dict(fixture=self.table, size=(0.25, 0.25),
                                        pos=(-0.6, 0.7), rotation=(0, 0)))]

        def _create_obj(self, cfg):
            return MJCFObject(name=cfg["name"], mjcf_path=str(cup_asset), scale=0.85), {
                "cat": "cup", "mjcf_path": str(cup_asset), "groups": ["cup"],
                "groups_containing_sampled_obj": ["cup", "receptacle"], "split": None}

        def edit_model_xml(self, xml_str):
            # The official HF bundle no longer includes old appliance visual
            # meshes. Keep every original collision primitive, and show those
            # primitives for affected fixtures. Fail on missing collision meshes.
            root = ET.fromstring(super().edit_model_xml(xml_str))
            asset = root.find("asset")
            missing_meshes, missing_textures = set(), set()
            for element in list(asset):
                file = element.get("file")
                if file and not Path(file).exists():
                    missing_visual_assets.append(file)
                    if element.tag == "mesh":
                        missing_meshes.add(element.get("name"))
                    elif element.tag == "texture":
                        missing_textures.add(element.get("name"))
                    else:
                        raise ValueError(f"Unsupported missing asset {file}")
                    asset.remove(element)
            for material in asset.findall("material"):
                if material.get("texture") in missing_textures:
                    material.attrib.pop("texture")
                    material.set("rgba", "0.55 0.57 0.59 1")
            for body in root.iter("body"):
                missing_visual = False
                for geom in list(body.findall("geom")):
                    if geom.get("mesh") in missing_meshes:
                        if geom.get("contype", "1") != "0" or geom.get("conaffinity", "1") != "0":
                            raise ValueError(f"Missing physical mesh {geom.get('mesh')}")
                        # Preserve geometry names required by robosuite's ID maps.
                        geom.attrib.pop("mesh")
                        geom.set("type", "sphere")
                        geom.set("size", "0.001")
                        geom.set("rgba", "0 0 0 0")
                        geom.set("mass", "0.001")
                        missing_visual = True
                if missing_visual:
                    if body.find("inertial") is None and not any(
                        g.get("contype", "1") != "0" for g in body.findall("geom")
                    ):
                        ET.SubElement(body, "inertial", pos="0 0 0", mass="0.001",
                                      diaginertia="0.000001 0.000001 0.000001")
                    for geom in body.findall("geom"):
                        if geom.get("contype", "1") != "0":
                            geom.set("rgba", "0.55 0.57 0.59 1")
            return ET.tostring(root, encoding="unicode")

    _, xml, placements = model_generation_wizard(
        task="StretchRedCupKitchen", layout=config["layout"], style=config["style"],
        robot_spawn_pose={"pos": " ".join(map(str, config["robot_start"])), "quat": "1 0 0 0"})
    root = ET.fromstring(xml)
    cup = root.find(".//body[@name='red_cup_main']")
    if cup is None:
        raise RuntimeError("RoboCasa did not create red_cup_main")
    # Reposition the sampled asset on the same island's clear northern edge.
    # Preserve the asset's height offset, collision meshes and physical properties.
    pos = np.fromstring(cup.get("pos"), sep=" ")
    pos[:2] = config["target_xy"]
    cup.set("pos", " ".join(map(str, pos)))
    for geom in cup.iter("geom"):
        geom.set("rgba", "0.85 0.015 0.02 1")
        geom.attrib.pop("material", None)
    # Export expanded MJCF so future generator invocations cannot overwrite its include.
    staged = out / "staged.xml"
    staged.write_text(ET.tostring(root, encoding="unicode"))
    model = mujoco.MjModel.from_xml_path(str(staged))
    model.opt.timestep = 1 / robot_config["physics_hz"]
    scene = out / "kitchen.xml"
    mujoco.mj_saveLastXML(str(scene), model)
    staged.unlink()
    metadata = {"seed": seed, "layout": config["layout"], "style": config["style"], "task": "StretchRedCupKitchen",
                "target_body": "red_cup_main", "table_fixture": "island_counter_island_group",
                "table_description": "RoboCasa kitchen island dining counter",
                "target_initial_position": pos.tolist(), "robot_start": config["robot_start"],
                "navigation_waypoints": config["navigation_waypoints"],
                "observation_source": "robot RGB and proprioception",
                "baseline_goal_source": "simulator ground truth"}
    (out / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    (out / "asset_compatibility.json").write_text(json.dumps({
        "visual_fallback": "Official collision primitives shown for unavailable legacy appliance meshes",
        "missing_visual_assets": sorted(set(missing_visual_assets)),
        "missing_collision_assets": [], "cup_asset": "objaverse/cup/cup_2/model.xml",
        "cup_scale": 0.85, "cup_compatibility": "reg_bbox metadata converted to legacy sites"
    }, indent=2) + "\n")
    print(f"Generated {scene}; target {pos.tolist()}; ngeom={model.ngeom}")
    return scene


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int)
    build(parser.parse_args().seed)

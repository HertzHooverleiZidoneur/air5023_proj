"""Build a separate navigation scene from the round-table kitchen scene."""

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
KITCHEN = ROOT / "scenes/kitchen/generated/kitchen.xml"
ROUND_TABLE = ROOT / "scenes/round_table/generated/kitchen_round_table.xml"
PROFILE = ROOT / "01_navigation/maps/round_table/profile.json"
OBSTACLE_HEIGHT_M = 0.38


def build(base_scene, profile_path):
    import mujoco

    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    output = (profile_path.parent / profile["scene"]).resolve()
    if output in (base_scene.resolve(), KITCHEN.resolve(), ROUND_TABLE.resolve()):
        raise ValueError("Navigation requires a separate scene output")
    scene = ET.parse(base_scene).getroot()
    # Keep asset references relative to the base scene even if output is elsewhere.
    compiler = scene.find("compiler")
    for asset in scene.find("asset"):
        filename = asset.get("file")
        if filename and not Path(filename).is_absolute():
            directory = "meshdir" if asset.tag == "mesh" else "texturedir"
            asset_dir = compiler.get(directory, compiler.get("assetdir", ""))
            asset.set("file", (base_scene.parent / asset_dir / filename).resolve().as_posix())

    world = scene.find("worldbody")
    base = world.find("body[@name='base_link']")
    x, y, yaw = profile["initial_pose"]
    height = base.get("pos", "0 0 0").split()[2]
    base.set("pos", f"{x} {y} {height}")
    base.set("quat", f"{math.cos(yaw / 2)} 0 0 {math.sin(yaw / 2)}")
    keyframe = scene.find("keyframe")
    if keyframe is not None:
        scene.remove(keyframe)  # The original keys encode the old spawn.
    scene.set("model", "stretch_kitchen_round_table_navigation")
    camera_x, camera_y, camera_z = profile["overview_camera"]
    ET.SubElement(world, "camera", name="navigation_overview",
                  pos=f"{camera_x} {camera_y} {camera_z}", xyaxes="1 0 0 0 1 0", fovy="36")

    names = []
    for index, (xmin, ymin, xmax, ymax) in enumerate(profile["obstacles"], start=1):
        if not all(map(math.isfinite, (xmin, ymin, xmax, ymax))) or xmin >= xmax or ymin >= ymax:
            raise ValueError("Obstacle bounds must describe a finite non-empty rectangle")
        name = f"navigation_obstacle_{index}"
        names.append(name)
        z = OBSTACLE_HEIGHT_M / 2
        body = ET.SubElement(world, "body", name=name, pos=f"{(xmin + xmax) / 2} {(ymin + ymax) / 2} {z}")
        ET.SubElement(body, "geom", name=name, type="box", size=f"{(xmax - xmin) / 2} {(ymax - ymin) / 2} {z}",
                      rgba="0.86 0.34 0.14 1", contype="1", conaffinity="1")

    xml = ET.tostring(scene, encoding="unicode")
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    for name in names:
        obstacle = model.geom(name).id
        for other in range(model.ngeom):
            collidable = model.geom_contype[other] or model.geom_conaffinity[other]
            if other == obstacle or not collidable or "floor" in model.geom(other).name:
                continue
            if mujoco.mj_geomDistance(model, data, obstacle, other, 0.001, None) < -0.001:
                raise ValueError(f"{name} overlaps {model.geom(other).name or model.geom_bodyid[other]}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(xml + "\n", encoding="utf-8")
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-scene", type=Path, default=ROUND_TABLE)
    parser.add_argument("--profile", type=Path, default=PROFILE)
    args = parser.parse_args()
    print(build(args.base_scene, args.profile))


if __name__ == "__main__":
    main()

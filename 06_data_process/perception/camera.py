"""TEAM-06/TEAM-07: external debug overview; never supplied to the VLA."""
import mujoco
from PIL import Image


def configure_overview(camera):
    camera.lookat[:] = [3.2, -1.8, 0.85]
    camera.distance = 3.0
    camera.azimuth = 45
    camera.elevation = -35


def save_overview(robot, path):
    camera = mujoco.MjvCamera()
    configure_overview(camera)
    option = mujoco.MjvOption()
    option.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
    with mujoco.Renderer(robot.model, height=480, width=640) as renderer:
        renderer.update_scene(robot.data, camera=camera, scene_option=option)
        Image.fromarray(renderer.render()).save(path)

import math

import mujoco
import pytest

from navigation.mini_slam.core import LidarModel, MiniSLAM, SensorSample
from navigation.slam_navigation import load_map, save_map
from navigation.stretch_sensors import StretchSensors


def test_stretch_sample_has_current_wheels_gyro_and_lidar_in_base_frame():
    model = mujoco.MjModel.from_xml_string("""
      <mujoco>
        <worldbody>
          <geom name="wall" type="box" pos="2 0 .5" size=".1 2 .5"/>
          <body name="base_link" pos="0 0 .25">
            <freejoint name="base"/>
            <geom type="box" size=".1 .1 .06"/>
            <geom type="sphere" pos=".004 .3 .1664" size=".05"/>
            <body name="left" pos="0 .15 0">
              <joint name="joint_left_wheel"/><geom size=".02"/>
            </body>
            <body name="right" pos="0 -.15 0">
              <joint name="joint_right_wheel"/><geom size=".02"/>
            </body>
            <site name="imu" euler="180 0 90"/>
            <body name="laser" pos=".004 0 .1664" quat="0 0 0 1">
              <replicate count="360" euler="0 0 1"><site name="lidar" zaxis="1 0 0"/></replicate>
            </body>
          </body>
        </worldbody>
        <sensor>
          <gyro name="base_gyro" site="imu"/>
          <rangefinder name="base_lidar" site="lidar" cutoff="10"/>
        </sensor>
      </mujoco>
    """)
    data = mujoco.MjData(model)
    sensor_flag = int(mujoco.mjtDisableBit.mjDSBL_SENSOR)
    model.opt.disableflags |= sensor_flag
    sensors = StretchSensors(model, data)
    forward = min(range(360), key=lambda beam: abs(sensors.lidar.angles[beam]))
    left = min(range(360), key=lambda beam: abs(sensors.lidar.angles[beam] - math.pi / 2))
    right = min(range(360), key=lambda beam: abs(sensors.lidar.angles[beam] + math.pi / 2))

    initial = sensors.sample()
    assert sensors.lidar.origin == pytest.approx((.004, 0))
    assert initial.ranges[forward] == pytest.approx(1.896)
    assert initial.ranges[left] is None  # self-occluded
    data.joint("base").qpos[0] = .3
    data.joint("base").qpos[3:7] = (math.sqrt(.5), 0, 0, math.sqrt(.5))
    data.joint("base").qvel[5] = .4
    data.joint("joint_left_wheel").qpos[0] = 1.5
    data.joint("joint_right_wheel").qpos[0] = 2.5
    data.time = .05
    turned = sensors.sample()
    assert turned.ranges[right] == pytest.approx(1.6)
    assert turned.ranges[forward] is None
    assert turned.gyro_z == pytest.approx(.4)
    assert turned.wheel_pos == (1.5, 2.5)
    assert turned.sim_time == .05
    assert model.opt.disableflags & sensor_flag


def test_lidar_mount_and_saved_map_round_trip(tmp_path):
    slam = MiniSLAM(LidarModel((0.0,), .05, 10.0, origin=(.12, -.12)), resolution=.1)
    slam.update(SensorSample((0., 0.), 0., (1.04,), 0.))
    output = tmp_path / "grids.json"
    save_map(slam.grid, output)
    frozen = load_map(output)
    assert frozen.resolution == slam.grid.resolution
    assert frozen.cells == slam.grid.cells
    assert frozen.state((11, -2)) == "occupied"
    assert frozen.state((10, 0)) == "unknown"
    with pytest.raises(FileExistsError):
        save_map(slam.grid, output)


def test_reject_duplicate_cells_in_reference_map(tmp_path):
    path = tmp_path / "grids.json"
    path.write_text('{"resolution_m":0.05,"slam":[[1,2,0.8],[1,2,-0.8]]}', encoding="utf-8")
    with pytest.raises(ValueError, match="unique integer cell indices"):
        load_map(path)

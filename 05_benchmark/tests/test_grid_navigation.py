"""Exercise the navigation bridge with a compiled MuJoCo collision scene."""

from math import cos, pi, sin
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from navigation.grid_navigation import GridNavigationError, navigate_grid
from navigation.tracking import Pose
from navigation.scene_grid import build_scene_grid


class KinematicRobot:
    """Use real geom transforms and a deterministic RobotInterface stand-in."""

    control_hz = 20
    base_limits = (0.3, 1.0)

    def __init__(self, scene=""):
        self.model = mujoco.MjModel.from_xml_string(f"""
        <mujoco>
          <worldbody>
            <geom type="plane" size="5 5 .1" />
            <body name="base_link" pos="0 0 .15">
              <freejoint />
              <geom type="box" size=".11 .08 .14" />
            </body>
            {scene}
          </worldbody>
        </mujoco>
        """)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.yaw = 0.0
        self.command = (0.0, 0.0)
        self.commands = []
        self.pushed = []
        self.server = SimpleNamespace(push_command=self.pushed.append)

    def get_state(self):
        return {"base_pose": np.array([*self.data.qpos[:2], self.yaw])}

    def move_base(self, v, w):
        self.command = (v, w)
        self.commands.append(self.command)

    def step(self):
        v, w = self.command
        dt = 1 / self.control_hz
        self.data.qpos[0] += v * dt * cos(self.yaw + w * dt / 2)
        self.data.qpos[1] += v * dt * sin(self.yaw + w * dt / 2)
        self.yaw += w * dt
        self.data.qpos[3:7] = (cos(self.yaw / 2), 0.0, 0.0, sin(self.yaw / 2))
        self.data.time += dt
        mujoco.mj_forward(self.model, self.data)

    def hold(self, seconds):
        for _ in range(round(seconds * self.control_hz)):
            self.step()


def test_static_grid_uses_collisions_in_the_robot_height_range():
    robot = KinematicRobot("""
      <geom type="box" pos="0 0 -.05" size="5 5 .05" />
      <geom type="box" pos=".50 0 .10" size=".05 .10 .10" />
      <geom type="box" pos=".75 0 .10" size=".05 .10 .10" contype="0" conaffinity="0" />
      <geom type="box" pos="1 0 1.00" size=".05 .10 .10" />
    """)
    snapshot = build_scene_grid(robot.model, robot.data, [(0, 0), (1, 0)])
    assert snapshot.robot_radius == pytest.approx(np.hypot(.11, .08))
    assert snapshot.grid.state(snapshot.grid.world_to_cell((.50, 0))) == "occupied"
    for x in (0, .75, 1):
        assert snapshot.grid.state(snapshot.grid.world_to_cell((x, 0))) == "free"


def test_navigates_around_obstacle_and_calls_arrival_for_each_goal():
    robot = KinematicRobot('<geom type="box" pos=".55 0 .1" size=".05 .10 .1" />')
    observed = []
    visits = navigate_grid(robot, [(1.1, 0, 0), (1.1, .4, pi / 2)], on_arrival=observed.append)
    assert observed == visits
    assert len(visits) == 2 and len(visits[0].path) > 2
    assert (visits[-1].pose.x, visits[-1].pose.y) == pytest.approx((1.1, .4), abs=.025)
    assert visits[-1].pose.yaw == pytest.approx(pi / 2, abs=.035)
    assert robot.pushed[-1] == (0.0, 0.0)


def test_blocked_route_stops_without_driving():
    robot = KinematicRobot('<geom type="box" pos=".55 0 .1" size=".05 4 .1" />')
    with pytest.raises(GridNavigationError, match="No clear path"):
        navigate_grid(robot, [(1.1, 0, 0)])
    assert robot.commands == [(0.0, 0.0)]
    assert robot.pushed == [(0.0, 0.0)]


def test_timeout_pushes_zero_speed():
    robot = KinematicRobot()
    with pytest.raises(TimeoutError, match="simulation-time budget"):
        navigate_grid(robot, [(1.1, 0, 0)], timeout=.05)
    assert any(v > 0 for v, _ in robot.commands)
    assert robot.pushed[-1] == (0.0, 0.0)


def test_supplied_grid_and_pose_source_drive_the_whole_route(monkeypatch):
    robot = KinematicRobot()
    measured = build_scene_grid(robot.model, robot.data, [(0, 0), (1.1, 0)]).grid

    def unavailable(*args, **kwargs):
        pytest.fail("Navigation accessed simulator pose or obstacle map")

    monkeypatch.setattr(robot, "get_state", unavailable)
    monkeypatch.setattr("navigation.grid_navigation.build_scene_grid", unavailable)
    sampled_times = []

    def estimate():
        sampled_times.append(robot.data.time)
        return Pose(*robot.data.qpos[:2], robot.yaw)

    visits = navigate_grid(robot, [(1.1, 0, 0), (.6, .3, pi / 2)],
                           navigation_grid=measured, pose_source=estimate)
    assert len(visits) == 2
    assert (visits[-1].pose.x, visits[-1].pose.y) == pytest.approx((.6, .3), abs=.025)
    assert sorted(set(sampled_times)) == pytest.approx(
        np.arange(round(sampled_times[-1] * robot.control_hz) + 1) / robot.control_hz)
    assert robot.pushed[-1] == (0.0, 0.0)

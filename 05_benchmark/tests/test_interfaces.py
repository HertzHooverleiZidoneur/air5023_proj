import os
os.environ.setdefault("MUJOCO_GL", "egl")
from types import SimpleNamespace
import mujoco
import numpy as np
import pytest
from robot.robot_interface import RobotInterface
from manipulation.arm_controller import solve_ik
from vla.action_adapter import Action, apply_action
from evaluation.success_checker import SuccessChecker


@pytest.fixture
def robot():
    with RobotInterface(cameras=False) as instance:
        yield instance


@pytest.mark.parametrize("action", [Action(base_velocity=(float("nan"), 0)),
                                     Action(joint_targets={"lift": 9}),
                                     Action(gripper=-1), Action(embodiment="droid"),
                                     np.zeros(8)])
def test_reject_invalid_actions_without_moving(robot, action):
    before = robot.command.to_dict()
    with pytest.raises(ValueError):
        apply_action(robot, action)
    assert robot.command.to_dict() == before
    assert robot.data.time == 0


def test_real_actuation_and_control_timestep(robot):
    apply_action(robot, Action(joint_targets={"lift": .6, "arm": .2}, gripper=1))
    robot.hold(3)
    state = robot.get_state()
    assert state["time"] == pytest.approx(3)
    assert state["joints"]["lift"] == pytest.approx(.6, abs=.02)
    assert state["joints"]["arm"] == pytest.approx(.2, abs=.02)


def test_unreachable_ik_preserves_sim_state(robot):
    before = robot.data.qpos.copy()
    with pytest.raises(ValueError, match="unreachable"):
        solve_ik(robot, {"position": [50, 50, 50]})
    np.testing.assert_array_equal(robot.data.qpos, before)


def test_lift_alone_is_not_grasp_success():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
    <geom name="floor" type="plane" size="2 2 .1"/>
    <body name="base_link"><body name="link_gripper_finger_left" pos="1 0 1">
    <geom name="left" type="sphere" size=".02"/></body>
    <body name="link_gripper_finger_right" pos="-1 0 1">
    <geom name="right" type="sphere" size=".02"/></body></body>
    <body name="red_cup_main" pos="0 0 .05"><freejoint/>
    <geom name="cup" type="sphere" size=".05" mass=".1"/></body>
    </worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    checker = SuccessChecker(SimpleNamespace(model=model, data=data))
    assert not checker.update()["success"]
    data.qpos[2] += .2
    mujoco.mj_forward(model, data)
    for t in [1, 2, 3]:
        data.time = t
        result = checker.update()
        assert result["lift_metres"] > .05
        assert not result["success"]
        assert not result["left_contact"]

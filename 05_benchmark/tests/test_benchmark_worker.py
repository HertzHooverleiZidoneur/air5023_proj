"""Exercise the real worker with a deterministic robot double, not a physics claim."""
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import numpy as np
import pytest
from evaluation.criteria import Criteria, HoldCheck
from evaluation.protocol import ROOT, read_json, sha256
import run_episode


class FakeRobot:
    def __init__(self, scene, **kwargs):
        self.control_hz = 2
        self.names = ["floor_room_g0", "finger", "cup", "table"]
        self.model = SimpleNamespace(ngeom=4, geom=lambda index: SimpleNamespace(name=self.names[index]),
                                     geom_type=[0, 1, 1, 1])
        self.base = SimpleNamespace(xpos=np.array([0., 0., 0.]), xmat=np.eye(3).flatten())
        self.data = SimpleNamespace(time=0., body=lambda name: self.base,
                                    geom_xmat=np.tile(np.eye(3).flatten(), (4, 1)),
                                    geom_xpos=np.zeros((4, 3)), contact=[])
        self.lift = 0.0
        self.dropped = False

    def step(self):
        self.data.time += 0.5

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return None


class FakeChecker:
    def __init__(self, robot, target, lift, hold):
        self.robot = robot
        self.check = HoldCheck(lift, hold, 0.75)
        self.last = {}

    def update(self):
        self.last = self.check.update(time=self.robot.data.time, lift=self.robot.lift,
                                      left=not self.robot.dropped, right=True, supported=False)
        return self.last


def success_adapter(robot, metadata, output, instruction, *, on_phase):
    on_phase("initialize", robot.data.time)
    robot.step(2)
    on_phase("navigate", robot.data.time)
    robot.base.xpos[0] = 0.5
    robot.step()
    robot.base.xpos[0] = 1.0
    robot.step()
    on_phase("pregrasp", robot.data.time)
    robot.step()
    on_phase("lift", robot.data.time)
    robot.lift = 0.1
    robot.step()
    on_phase("verify", robot.data.time)
    robot.step(3)
    return {"success": False}


def drop_adapter(robot, metadata, output, instruction, *, on_phase):
    success_adapter(robot, metadata, output, instruction, on_phase=on_phase)
    robot.dropped = True
    robot.step()
    return {"success": True}


def collision_adapter(robot, metadata, output, instruction, *, on_phase):
    robot.data.contact = [SimpleNamespace(geom1=1, geom2=3)]
    return success_adapter(robot, metadata, output, instruction, on_phase=on_phase)


def floor_adapter(robot, metadata, output, instruction, *, on_phase):
    robot.data.contact = [SimpleNamespace(geom1=1, geom2=0)]
    return success_adapter(robot, metadata, output, instruction, on_phase=on_phase)


def timeout_adapter(robot, metadata, output, instruction, *, on_phase):
    on_phase("initialize", robot.data.time)
    robot.step()
    on_phase("navigate", robot.data.time)
    robot.step()
    raise TimeoutError("Fixture navigation timeout")


@pytest.fixture
def worker_request(tmp_path, monkeypatch):
    for name, attributes in {
        "robot.robot_interface": {"RobotInterface": FakeRobot},
        "evaluation.success_checker": {"SuccessChecker": FakeChecker, "descendant_geoms": lambda model, name: [1]},
        "mujoco": {"mjtGeom": SimpleNamespace(mjGEOM_PLANE=0)},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    scene, metadata = tmp_path / "scene.xml", tmp_path / "metadata.json"
    scene.write_text("<mujoco/>")
    metadata.write_text('{"seed":7,"target_body":"cup","navigation_waypoints":[[1,0,0]]}')
    return {"condition": {"scene": str(scene), "metadata": str(metadata), "seed": 7,
                          "scene_sha256": sha256(scene), "metadata_sha256": sha256(metadata)},
            "criteria": Criteria().to_dict(), "policy_seed": 7, "instruction": "fixture",
            "with_images": False, "task_config_sha256": sha256(ROOT / "configs/task.yaml")}


@pytest.mark.parametrize("adapter,expected,reason", [
    ("success_adapter", True, None), ("drop_adapter", False, "dropped_after_grasp"),
    ("collision_adapter", False, "navigation_collision"), ("floor_adapter", True, None),
    ("timeout_adapter", False, "navigation_timeout")])
def test_worker_samples_path_and_overrules_controller_claims(tmp_path, worker_request, adapter, expected, reason):
    worker_request["strategy"] = f"{__name__}:{adapter}"
    assert run_episode.run(worker_request, tmp_path) == (0 if expected else 1)
    result = read_json(tmp_path / "metrics.json")
    assert result["task_success"] is expected
    assert result["reason"] == reason
    assert (tmp_path / "runtime.json").exists()
    assert (tmp_path / "telemetry.jsonl").read_text()
    if expected:
        assert result["controller_reported_success"] is False
        assert result["navigation_distance_metres"] == pytest.approx(1)
        assert result["navigation_seconds"] == pytest.approx(1)
    if adapter == "drop_adapter":
        assert result["controller_reported_success"] is True
        assert result["ever_grasp_success"] is True


def test_worker_rejects_inputs_changed_after_plan(tmp_path, worker_request):
    Path(worker_request["condition"]["scene"]).write_text("changed")
    with pytest.raises(ValueError, match="scene changed"):
        run_episode.run(worker_request, tmp_path)


def test_worker_rejects_threshold_changes_between_episodes(tmp_path, worker_request):
    worker_request["task_config_sha256"] = "stale"
    with pytest.raises(ValueError, match="configuration changed"):
        run_episode.run(worker_request, tmp_path)

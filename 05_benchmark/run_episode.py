"""TEAM-05 isolated simulation worker; all instrumentation stays in this group."""
import argparse
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import random
import sys
import traceback
from collections import Counter

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from project_bootstrap import activate
activate()
from evaluation.criteria import Criteria
from evaluation.protocol import read_json, sha256, write_json
from evaluation.telemetry import Telemetry, failure_reason
from evaluation.tasks import MOBILE, PLACE, BUILTINS, audit_access
from evaluation.placement import PlacementCheck
from evaluation.rejection import StopCheck


def run(request, output):
    os.environ.setdefault("MUJOCO_GL", "glfw" if sys.platform == "win32" else "egl")
    os.environ.setdefault("NUMBA_CACHE_DIR", str(ROOT / ".cache/numba"))
    if sha256(ROOT / "configs/task.yaml") != request["task_config_sha256"]:
        raise ValueError("Task configuration changed since preflight")
    from robot.robot_interface import RobotInterface
    from evaluation.success_checker import SuccessChecker, descendant_geoms
    import mujoco
    import numpy as np

    condition = request["condition"]
    task_kind = condition.get("task", MOBILE)
    track = condition.get("track", "feasible")
    for key in ("scene", "metadata"):
        if sha256(condition[key]) != condition[key + "_sha256"]:
            raise ValueError(f"{key} changed since suite preflight")
    metadata = read_json(condition["metadata"])
    resources = condition.get("resources", {})
    for resource in resources.values():
        if sha256(resource["path"]) != resource["sha256"]:
            raise ValueError("Task resource changed since preflight")
    metadata = {**metadata, "resources": {name: value["path"] for name, value in resources.items()}}
    criteria = Criteria(**{key: value for key, value in request["criteria"].items() if key in Criteria.__dataclass_fields__})
    random.seed(request["policy_seed"])
    np.random.seed(request["policy_seed"])
    telemetry = Telemetry()
    write_json(output / "progress.json", {"phase": "setup", "simulation_time": 0.0})

    class MeasuredRobot(RobotInterface):
        measuring = False
        checker = None
        ever_grasp_success = False
        placement = None
        contacts = None
        stop_check = None

        def configure_measurement(self, trace):
            self.trace = trace
            self.api_calls = Counter()
            self.target_geoms = set(descendant_geoms(self.model, metadata["target_body"]))
            if task_kind == PLACE:
                from evaluation.round_table import RoundTableContacts
                self.contacts = RoundTableContacts(self)
                self.placement = PlacementCheck(self.control_hz)
            if track == "safe_rejection":
                self.stop_check = StopCheck(self.control_hz, request["rejection_criteria"])
            self.robot_geoms = set(descendant_geoms(self.model, "base_link"))
            self.floor_geoms = {index for index in range(self.model.ngeom)
                                if self.model.geom(index).name in {"floor_room_g0", "floor_backing_room_g0"}
                                or (self.model.geom_type[index] == mujoco.mjtGeom.mjGEOM_PLANE
                                    and abs(self.data.geom_xmat[index][8] - 1.0) < 1e-6
                                    and self.data.geom_xpos[index][2] <= 0.015)}
            self.measuring = True
            self.sample()

        def sample(self):
            base = self.data.body("base_link")
            pose = [float(base.xpos[0]), float(base.xpos[1]),
                    float(np.arctan2(base.xmat[3], base.xmat[0]))]
            pairs, operation_pairs = [], []
            for contact in self.data.contact:
                pair = {int(contact.geom1), int(contact.geom2)}
                if len(pair & self.robot_geoms) == 1 and not pair & self.floor_geoms:
                    names = tuple(self.model.geom(index).name or f"geom:{index}" for index in sorted(pair))
                    pairs.append(names)
                    if not pair & self.target_geoms:
                        operation_pairs.append(names)
            row = telemetry.sample(float(self.data.time), pose, pairs, operation_pairs)
            if self.stop_check:
                self.stop_check.update(float(self.data.time), pose)
            if self.placement:
                sample = self.contacts.sample()
                # The G1 replay initializes for exactly 1 s before tracking.
                if self.placement.initial is None and self.data.time >= 1.0 - 1e-8:
                    self.placement.start(sample["pos"])
                self.placement.update(sample, finite=bool(np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all()))
                row["placement_observation"] = sample
            if self.checker:
                row["grasp"] = self.checker.update()
                self.ever_grasp_success |= row["grasp"]["success"]
            self.trace.write(json.dumps(row, allow_nan=False) + "\n")
            self.trace.flush()

        def set_gripper(self, open=True, *, position=None):
            super().set_gripper(open, position=position)
            if self.placement:
                self.placement.gripper_command(bool(open) if position is None else position > 0.5, float(self.data.time))

        def get_state(self):
            if self.measuring:
                self.api_calls["get_state_simulator_base_pose"] += 1
            return super().get_state()

        def get_object_pose(self, body_name):
            if self.measuring:
                self.api_calls["get_object_pose_truth"] += 1
            return super().get_object_pose(body_name)

        def get_observation(self, instruction=""):
            if self.measuring:
                self.api_calls["get_observation_includes_truth_state"] += 1
            return super().get_observation(instruction)

        def get_camera_image(self, camera="head"):
            if self.measuring:
                self.api_calls["get_camera_image"] += 1
            return super().get_camera_image(camera)

        def step(self, steps=1):
            for _ in range(steps):
                super().step()
                if self.measuring:
                    self.sample()

    with (output / "telemetry.jsonl").open("w", encoding="utf-8") as trace:
        with MeasuredRobot(condition["scene"], cameras=request["with_images"], realtime=False) as robot:
            if task_kind == PLACE:
                from manipulation.tcp_retarget import restore_dock
                restore_dock(robot, Path(resources["dock"]["path"]))
            robot.configure_measurement(trace)

            def on_phase(name, time):
                if abs(float(time) - float(robot.data.time)) > 1e-6:
                    raise ValueError("Phase timestamp must match simulation clock")
                telemetry.set_phase(name, float(time))
                if task_kind == MOBILE and track == "feasible" and name == "navigate" and robot.checker is None:
                    robot.checker = SuccessChecker(robot, metadata["target_body"],
                                                   criteria.minimum_lift_metres, criteria.minimum_hold_seconds)
                    robot.checker.update()
                write_json(output / "progress.json", {"phase": name, "simulation_time": float(time)})

            controller_result = None
            error = None
            try:
                strategy = request["strategy"]
                spec = request.get("strategy_spec", BUILTINS.get(strategy, {}))
                module, symbol = spec.get("entrypoint", strategy).split(":")
                task = getattr(importlib.import_module(module), symbol)
                versions = {}
                for package in ("numpy", "mujoco", "robosuite", "stretch_mujoco", "robocasa"):
                    try:
                        versions[package] = importlib.metadata.version(package)
                    except importlib.metadata.PackageNotFoundError:
                        versions[package] = "not_registered"
                source = Path(sys.modules[task.__module__].__file__)
                write_json(output / "runtime.json", {"packages": versions, "strategy_source": str(source),
                                                     "strategy_source_sha256": sha256(source)})
                controller_result = task(robot, metadata, output / "stages", request["instruction"], on_phase=on_phase)
            except Exception as exc:
                error = {"type": type(exc).__name__, "message": str(exc), "phase": telemetry.phase}
                traceback.print_exc()
            metrics = telemetry.result(metadata.get("navigation_waypoints", [None])[-1] if metadata.get("navigation_waypoints") else None, criteria)
            grasp = robot.checker.last if robot.checker else {}
            collision_free = metrics["navigation_collision_events"] <= criteria.maximum_navigation_collision_events
            success = bool(error is None and metrics["navigation_success"] and collision_free and grasp.get("success"))
            metrics.update(schema_version="benchmark-episode-1.0", success=success, task_success=success,
                           grasp_success=bool(grasp.get("success")), grasp=grasp,
                           ever_grasp_success=robot.ever_grasp_success, collision_free=collision_free,
                           criteria=criteria.to_dict(), strategy=request["strategy"], seed=condition["seed"],
                           policy_seed=request["policy_seed"], error=error,
                           controller_reported_success=controller_result.get("success", controller_result.get("passed")) if isinstance(controller_result, dict) else None)
            metrics["reason"] = ("navigation_timeout" if error and error["type"] == "TimeoutError" and error["phase"] == "navigate"
                                 else "execution_error" if error else failure_reason(metrics))
            metrics.update(schema_version="benchmark-episode-2.0", task=task_kind, track=track,
                           split=condition.get("split", "unspecified"), placement_success=None,
                           safe_rejection_success=None, navigation_applicable=task_kind == MOBILE and track == "feasible")
            if task_kind == PLACE:
                placement = robot.placement.result()
                success = bool(error is None and placement["success"])
                metrics.update(success=success, task_success=success, navigation_success=None,
                               grasp_success=placement["grasp_success"], placement_success=placement["placement_success"],
                               placement=placement, collision_free=None,
                               reason="execution_error" if error else placement["reason"])
            if track == "safe_rejection":
                rejection = robot.stop_check.result(controller_result, metrics["all_robot_environment_collision_events"], error)
                success = rejection["success"]
                metrics.update(success=success, task_success=False, navigation_success=None, grasp_success=None,
                               safe_rejection_success=success, rejection=rejection, reason=rejection["reason"])
            metrics["information_audit"] = audit_access(request.get("strategy_spec", BUILTINS.get(request["strategy"], {})), robot.api_calls)
            write_json(output / "metrics.json", metrics)
            return 0 if success else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--request", type=Path, required=True)
    args = parser.parse_args()
    output = args.request.resolve().parent
    try:
        return run(read_json(args.request), output)
    except Exception as exc:
        traceback.print_exc()
        write_json(output / "error.json", {"type": type(exc).__name__, "message": str(exc), "phase": "setup_or_finalize"})
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

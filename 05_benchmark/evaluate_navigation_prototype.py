"""Evaluate continuous A/B/A docking and cancellation in the Navigation Prototype."""

import argparse
import csv
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from project_bootstrap import activate

activate()


class NavigationEvaluation:
    """Record independent truth and sensor estimates at each control sample."""

    def __init__(self, session, profile, output):
        import mujoco
        from navigation.scene_grid import _robot_colliders

        self.session = session
        self.robot = session.robot
        self.profile = profile
        self.output = output
        self.contacts = set()
        self.robot_geoms, _ = _robot_colliders(self.robot.model)
        self.floor_geoms = {
            geom.id for geom in (self.robot.model.geom(i) for i in range(self.robot.model.ngeom))
            if geom.name in {"floor_room_g0", "floor_backing_room_g0"}
            or geom.type == mujoco.mjtGeom.mjGEOM_PLANE
        }
        self.truth = []
        self.estimates = []
        self.visits = []

    def record(self, _pose, phase):
        robot = self.robot
        base = robot.data.body("base_link")
        x, y = map(float, base.xpos[:2])
        yaw = math.atan2(base.xmat[3], base.xmat[0])
        time = float(robot.data.time)
        self.truth.append((time, x, y, yaw))
        slam = self.session.slam.estimator
        self.estimates.append((time, phase, *(value for pose in (slam.wheel_pose, slam.odom_pose, slam.pose)
                                              for value in (pose.x, pose.y, pose.yaw))))
        for contact in robot.data.contact:
            pair = {int(contact.geom1), int(contact.geom2)}
            if len(pair & self.robot_geoms) == 1 and not pair & self.floor_geoms:
                self.contacts.add(tuple(robot.model.geom(i).name or f"geom_{i}" for i in sorted(pair)))

    def arrival(self, visit, label):
        from navigation.docking import angle_difference, station_depth

        _, x, y, yaw = self.truth[-1]
        slam = self.session.slam
        station = visit.goal
        result = {"station": label, **asdict(visit), "truth_pose": [x, y, yaw],
                  "position_error_m": math.dist((x, y), (station.x, station.y)),
                  "heading_error_rad": abs(angle_difference(yaw, station.yaw)),
                  "depth_residual_m": station_depth(self.profile.stations[label], visit.pose,
                                                    slam.sensors.lidar, slam.latest_sample)}
        if self.robot.camera_manager:
            from PIL import Image

            image = f"{len(self.visits) + 1:02d}-{label}.png"
            Image.fromarray(self.robot.get_camera_image("head")).save(self.output / image)
            result["head_rgb"] = image
        return result

    def run(self):
        for label in ("A", "B", "A"):
            station = self.profile.stations[label]
            if label == "B":
                cancel_at = self.robot.data.time + 1.5
                cancelled = self.session.go(station.pose, station=station, on_update=self.record,
                                            cancelled=lambda: self.robot.data.time >= cancel_at)
                if cancelled is not None:
                    raise RuntimeError("Expected the first B request to cancel before arrival")
                cancellation = float(self.robot.data.time)
                cancel_pose = asdict(self.session.pose)
                cancel_velocity = self.robot.get_state()["base_velocity"].tolist()
            visit = self.session.go(station.pose, station=station, on_update=self.record)
            self.visits.append(self.arrival(visit, label))
        return {"visits": self.visits, "cancelled_at_s": cancellation, "cancelled_pose": cancel_pose,
                "cancelled_velocity_m_s_rad_s": cancel_velocity}

    def save(self, result):
        from navigation.docking import angle_difference

        trackers = ("wheel", "fused", "slam")
        pose_fields = ("x_m", "y_m", "yaw_rad")
        for name, columns, records in (
                ("truth", ("sim_time_s", *pose_fields), self.truth),
                ("trajectory", ("sim_time_s", "phase", *(f"{name}_{field}" for name in trackers
                                                          for field in pose_fields)), self.estimates)):
            with (self.output / f"{name}.csv").open("w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(columns)
                writer.writerows(records)
        slam = self.session.slam.estimator
        result.update(matches_accepted=slam.matches_accepted, matches_attempted=slam.matches_attempted,
                      contact_pairs_at_control_ticks=sorted(self.contacts), sample_count=len(self.truth))
        if self.truth:
            result["pose_errors"] = {}
            for i, name in enumerate(trackers):
                offset = 2 + 3 * i
                pos2 = yaw2 = 0.0
                for (_, x, y, yaw), row in zip(self.truth, self.estimates):
                    ex, ey, eyaw = row[offset:offset + 3]
                    pos2 += (ex - x) ** 2 + (ey - y) ** 2
                    yaw2 += angle_difference(eyaw, yaw) ** 2
                count = len(self.truth)
                result["pose_errors"][name] = {"position_rmse_m": math.sqrt(pos2 / count),
                                               "heading_rmse_rad": math.sqrt(yaw2 / count)}
        (self.output / "metrics.json").write_text(json.dumps(result, indent=2), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--no-cameras", action="store_true")
    args = parser.parse_args()
    os.environ.setdefault("MUJOCO_GL", "glfw" if sys.platform == "win32" else "egl")
    from navigation.prototype import load_profile, open_session

    profile = load_profile(args.profile)
    args.out.mkdir(parents=True)
    with open_session(profile, realtime=False, cameras=not args.no_cameras) as session:
        evaluation = NavigationEvaluation(session, profile, args.out)
        try:
            result = evaluation.run()
        except Exception as error:
            evaluation.save({"visits": evaluation.visits, "error": str(error)})
            raise
        evaluation.save(result)
        if evaluation.contacts:
            parser.exit(1, "Robot made contact with a non-floor object. See metrics.json.\n")
    print(args.out / "metrics.json")


if __name__ == "__main__":
    main()

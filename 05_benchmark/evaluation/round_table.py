"""Force-based grasp and placement measurements for the round-table example."""

import mujoco
import numpy as np

from evaluation.success_checker import descendant_geoms

TABLETOP = "g1_round_table_support"
SETTLE_SECONDS = 1.5


class RoundTableContacts:
    def __init__(self, robot):
        self.model, self.data = robot.model, robot.data
        self.bottle = self.data.body("g1_bottle")
        bottle_joint = self.model.body("g1_bottle").jntadr[0]
        self.bottle_velocity_start = self.model.jnt_dofadr[bottle_joint]
        self.bottle_geoms = set(descendant_geoms(self.model, "g1_bottle"))
        self.robot_geoms = set(descendant_geoms(self.model, "base_link"))
        self.table_geoms = set(descendant_geoms(self.model, "g1_round_table"))
        self.pad_ids = {geom: side for side, body in enumerate(("rubber_tip_left", "rubber_tip_right"))
                        for geom in descendant_geoms(self.model, body)}
        self.geom_names = [self.model.geom(i).name or self.model.body(int(body_id)).name
                           for i, body_id in enumerate(self.model.geom_bodyid)]
        self.force = np.zeros(6)

    def sample(self):
        contacts = set()
        robot_table_contacts = []
        forces = [0.0, 0.0]
        supported = False
        for i, contact in enumerate(self.data.contact):
            g1, g2 = int(contact.geom1), int(contact.geom2)
            robot_table = ((g1 in self.robot_geoms and g2 in self.table_geoms)
                           or (g2 in self.robot_geoms and g1 in self.table_geoms))
            bottle_contact = g1 in self.bottle_geoms or g2 in self.bottle_geoms
            if contact.efc_address < 0 or not (robot_table or bottle_contact):
                continue
            mujoco.mj_contactForce(self.model, self.data, i, self.force)
            if self.force[0] <= 0.01:
                continue
            if robot_table:
                robot_table_contacts.append([self.geom_names[g1], self.geom_names[g2]])
            if bottle_contact:
                other = g2 if g1 in self.bottle_geoms else g1
                contacts.add(self.geom_names[other])
                if other in self.pad_ids:
                    forces[self.pad_ids[other]] += float(self.force[0])
                supported |= other not in self.robot_geoms
        velocity = self.data.qvel[self.bottle_velocity_start:self.bottle_velocity_start + 3]
        return {
            "t": float(self.data.time), "left": forces[0] > 0, "right": forces[1] > 0,
            "support": supported, "table_only": contacts == {TABLETOP},
            "pos": self.bottle.xpos.tolist(), "contacts": sorted(contacts),
            "tilt_deg": float(np.degrees(np.arccos(np.clip(self.bottle.xmat[8], -1, 1)))),
            "speed": float(np.linalg.norm(velocity)), "finger_forces": forces,
            "robot_table_contacts": robot_table_contacts,
        }


def summarize(trace, initial_position, control_hz, supported_before_release):
    lift = [sample["pos"][2] - initial_position[2] for sample in trace]
    streak = longest = 0
    for sample, height in zip(trace, lift):
        held = sample["left"] and sample["right"] and not sample["support"] and height > 0.05
        streak = streak + 1 if held else 0
        longest = max(streak, longest)
    final = [sample for sample in trace if sample["phase"] == "settle"]
    stable_place = len(final) >= round(SETTLE_SECONDS * control_hz) and all(
        sample["table_only"] and sample["tilt_deg"] < 6 and sample["speed"] < 0.025
        for sample in final
    )
    tracking_error = np.array([
        np.linalg.norm(np.asarray(sample["goal"]) - sample["tcp"]) for sample in trace
        if sample["phase"] in ("source_open", "source_clamped")
    ])
    return {
        "passed": bool(longest >= control_hz and supported_before_release and stable_place),
        "peak_lift_m": float(max(lift)),
        "both_pads_unsupported_lift_seconds": longest / control_hz,
        "closed_support_before_release": supported_before_release,
        "stable_table_only_seconds": len(final) / control_hz if stable_place else 0.0,
        "place_xy_shift_m": float(np.linalg.norm(np.asarray(final[-1]["pos"][:2]) - initial_position[:2])),
        "final_tilt_deg": final[-1]["tilt_deg"], "final_speed_mps": final[-1]["speed"],
        "final_support": final[-1]["contacts"],
        "sampled_robot_table_contact_count": sum(bool(sample["robot_table_contacts"]) for sample in trace),
        "measured_tcp_tracking_rmse_m": float(np.sqrt(np.mean(tracking_error ** 2))),
        "measured_tcp_tracking_p95_m": float(np.percentile(tracking_error, 95)),
        "force_threshold_n": 0.01,
    }

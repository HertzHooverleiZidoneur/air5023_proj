"""Export the round-table example's TCP trajectory from its aligned Isaac replay."""

import argparse
import json
from pathlib import Path

import numpy as np


def extract(replay, trace):
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(replay.resolve()))
    rows = [json.loads(line) for line in trace.read_text().splitlines()]
    if len(rows) < 224:
        raise ValueError("Replay must include the example's pinch calibration frames 183–223")
    (wrist,) = (prim for prim in stage.Traverse() if prim.GetName() == "right_wrist_yaw_link"
                and prim.GetAttribute("xformOp:transform:recordedNewton"))
    matrices = np.array([np.asarray(UsdGeom.XformCache(i).GetLocalToWorldTransform(wrist))
                         for i in range(len(rows))])
    rotation = np.transpose(matrices[:, :3, :3], (0, 2, 1))
    rotation /= np.linalg.norm(rotation, axis=1, keepdims=True)
    wrist_xyz = matrices[:, 3, :3]
    if not np.allclose(wrist_xyz, [row["wrist_xyz"] for row in rows], atol=1e-6, rtol=0):
        raise ValueError("USD replay and JSONL wrist positions do not match")
    fingers = np.array([row["finger_xyz"] for row in rows])
    if fingers.shape != (len(rows), 16, 3):
        raise ValueError("Expected the aligned replay's 16 fingertip/link positions")
    # The replay orders index, middle, and thumb tip origins at 2, 5, and 15.
    pinch = (fingers[:, 15] + (fingers[:, 2] + fingers[:, 5]) * 0.5) * 0.5
    local_offsets = np.einsum("nji,nj->ni", rotation, pinch - wrist_xyz)
    pinch_offset = np.median(local_offsets[183:224], axis=0)
    tcp = wrist_xyz + np.einsum("nij,j->ni", rotation, pinch_offset)

    time = np.array([row["source_time_s"] for row in rows])
    time -= time[0]
    close = np.array([row["right_grasp_cmd"] for row in rows])
    if not np.isin(close, (0, 1)).all() or close[0] != 0 or close[-1] != 0:
        raise ValueError("Expected binary gripper commands beginning and ending open")
    edges = np.flatnonzero(np.diff(close)) + 1
    if len(edges) != 2:
        raise ValueError("Expected one close event and one open event")
    direction = np.asarray(rows[0]["bottle_xyz"])[:2] - wrist_xyz[edges[0], :2]
    yaw = -np.pi / 2 - np.arctan2(direction[1], direction[0])
    c, s = np.cos(yaw), np.sin(yaw)
    return dict(time=time, tcp=tcp, rotation_xy=np.array([[c, -s], [s, c]]), close=close)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay", type=Path)
    parser.add_argument("trace", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    arrays = extract(args.replay, args.trace)
    with args.output.open("xb") as output:
        np.savez_compressed(output, **arrays)


if __name__ == "__main__":
    main()

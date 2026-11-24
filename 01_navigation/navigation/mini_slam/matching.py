"""Bounded correlative scan matching against an existing log-odds grid."""

from __future__ import annotations

import math


def match_scan(grid, prior, points):
    """Return a corrected pose and diagnostics for base-frame scan endpoints."""
    if len(points) < 8 or not grid.cells:
        return prior, {"accepted": False, "reason": "insufficient_scan_or_map"}

    # The same neighbourhood may be queried by many candidate poses.
    support = {}
    x0, y0, yaw0 = prior
    best = (0.0, 0.0, 0.0)
    rotation = _rotate(points, yaw0)
    prior_score, prior_overlap = _score(grid, support, x0, y0, rotation)
    best_score, overlap = prior_score, prior_overlap
    min_overlap = max(8, math.ceil(len(points) * 0.18))
    max_xy, max_yaw = 0.16, 0.10

    # The fine search is centred on the best coarse offset. All offsets stay
    # relative to the prior, including when yaw crosses the ±pi boundary.
    for xy_step, yaw_step, xy_steps, yaw_steps in ((0.04, 0.04, 4, 2), (0.02, 0.02, 2, 2)):
        cx, cy, ca = best
        for i in range(-yaw_steps, yaw_steps + 1):
            da = ca + i * yaw_step
            if abs(da) > max_yaw + 1e-9:
                continue
            rotation = _rotate(points, yaw0 + da)
            for j in range(-xy_steps, xy_steps + 1):
                dx = cx + j * xy_step
                if abs(dx) > max_xy + 1e-9:
                    continue
                for k in range(-xy_steps, xy_steps + 1):
                    dy = cy + k * xy_step
                    if abs(dy) > max_xy + 1e-9:
                        continue
                    score, used = _score(grid, support, x0 + dx, y0 + dy, rotation)
                    # Stretch encoder/gyro updates are dense. Prefer small
                    # corrections unless the scan has clear geometric support.
                    score -= 15.0 * (dx * dx + dy * dy) + 30.0 * da * da
                    if used >= min_overlap and score > best_score:
                        best, best_score, overlap = (dx, dy, da), score, used

    accepted = prior_overlap > 0 and overlap >= min_overlap and best_score - prior_score >= 0.015
    dx, dy, da = best
    corrected = (x0 + dx, y0 + dy, (yaw0 + da + math.pi) % (2 * math.pi) - math.pi)
    return (corrected if accepted else prior), {
        "accepted": accepted, "score": best_score, "prior_score": prior_score,
        "overlap": overlap, "reason": "matched" if accepted else "weak_overlap_or_improvement",
    }


def _rotate(points, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(c * x - s * y, s * x + c * y) for x, y in points]


def _score(grid, support, x, y, rotated_pts):
    floor = math.floor
    inv_cell = 1.0 / grid.resolution
    occupied, used = 0.0, 0
    for px, py in rotated_pts:
        ix, iy = floor((x + px) * inv_cell), floor((y + py) * inv_cell)
        key = (ix, iy)
        evidence = support.get(key)
        if evidence is None:
            # Allow for quantisation, with stronger support in the hit cell.
            cell = grid.cells.get
            nearby = max(cell((ix - 1, iy), 0.0), cell((ix + 1, iy), 0.0),
                         cell((ix, iy - 1), 0.0), cell((ix, iy + 1), 0.0))
            evidence = max(0.0, min(3.0, cell(key, 0.0)), 0.55 * min(3.0, nearby))
            support[key] = evidence
        if evidence > 0:
            occupied += evidence
            used += 1
    # A candidate that drops endpoints cannot increase its average support.
    return occupied / len(rotated_pts), used

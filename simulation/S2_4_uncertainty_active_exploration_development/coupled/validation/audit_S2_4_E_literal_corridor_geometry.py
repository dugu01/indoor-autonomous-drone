#!/usr/bin/env python3
"""Static geometry contract for the literal S2.4-E competing-corridors world.

This audit is deliberately independent of MATLAB. It verifies only the physical
scenario geometry and the design contract used by the later coupled mission:
- both fork branches remain feasible after the inherited safety inflation;
- the target and decoy are initially occluded by the fork wall;
- the target is in the upper branch and the decoy probe is in the lower branch;
- the east-side branches are physically separated by the divider; and
- the lower decoy branch has more free area, so it is genuinely tempting.

It does NOT validate the coupled autonomy mission. MATLAB remains the release
authority for target selection, execution, mapping, RTL and landing.
"""
from __future__ import annotations

import heapq
import json
import math
import pathlib
from collections import deque
from typing import Iterable

ROOT = pathlib.Path(__file__).resolve().parents[2]
GEOMETRY = ROOT / "coupled" / "scenarios" / "literal_competing_corridors_geometry.json"
OUT_JSON = ROOT / "coupled" / "evidence" / "S2_4_E_LITERAL_CORRIDOR_GEOMETRY.json"
OUT_TXT = ROOT / "coupled" / "evidence" / "S2_4_E_LITERAL_CORRIDOR_GEOMETRY.txt"


def point_in_rect(p: tuple[float, float], r: list[float]) -> bool:
    x, y = p
    rx, ry, w, h = r
    return rx <= x <= rx + w and ry <= y <= ry + h


def segment_hits_rect(a: tuple[float, float], b: tuple[float, float], r: list[float]) -> bool:
    """Liang-Barsky segment/axis-aligned-rectangle intersection."""
    x0, y0 = a
    x1, y1 = b
    xmin, ymin, w, h = r
    xmax, ymax = xmin + w, ymin + h
    dx, dy = x1 - x0, y1 - y0
    p = (-dx, dx, -dy, dy)
    q = (x0 - xmin, xmax - x0, y0 - ymin, ymax - y0)
    u1, u2 = 0.0, 1.0
    for pi, qi in zip(p, q):
        if abs(pi) < 1e-12:
            if qi < 0:
                return False
            continue
        t = qi / pi
        if pi < 0:
            u1 = max(u1, t)
        else:
            u2 = min(u2, t)
        if u1 > u2:
            return False
    return True


def inflate_rect(r: list[float], d: float) -> list[float]:
    x, y, w, h = r
    return [x - d, y - d, w + 2 * d, h + 2 * d]


def build_grid(room: list[float], resolution: float, inflation: float, rects: list[list[float]]):
    nx = int(round(room[0] / resolution)) + 1
    ny = int(round(room[1] / resolution)) + 1
    xs = [i * resolution for i in range(nx)]
    ys = [j * resolution for j in range(ny)]
    inflated = [inflate_rect(r, inflation) for r in rects]
    free = [[True] * nx for _ in range(ny)]
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            if x < inflation or x > room[0] - inflation or y < inflation or y > room[1] - inflation:
                free[j][i] = False
                continue
            if any(point_in_rect((x, y), r) for r in inflated):
                free[j][i] = False
    return xs, ys, free, inflated


def nearest_cell(p: tuple[float, float], xs: list[float], ys: list[float]) -> tuple[int, int]:
    i = min(range(len(xs)), key=lambda k: abs(xs[k] - p[0]))
    j = min(range(len(ys)), key=lambda k: abs(ys[k] - p[1]))
    return j, i


def neighbors(cell: tuple[int, int], free: list[list[bool]]) -> Iterable[tuple[int, int]]:
    j, i = cell
    ny, nx = len(free), len(free[0])
    for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
        jj, ii = j + dj, i + di
        if not (0 <= jj < ny and 0 <= ii < nx) or not free[jj][ii]:
            continue
        if dj and di:
            # No diagonal corner cutting.
            if not free[j][ii] or not free[jj][i]:
                continue
        yield jj, ii


def astar(start: tuple[int, int], goal: tuple[int, int], free: list[list[bool]]) -> list[tuple[int, int]]:
    if not free[start[0]][start[1]] or not free[goal[0]][goal[1]]:
        return []
    q: list[tuple[float, float, tuple[int, int]]] = []
    heapq.heappush(q, (0.0, 0.0, start))
    parent: dict[tuple[int, int], tuple[int, int]] = {}
    g = {start: 0.0}
    while q:
        _, gc, cur = heapq.heappop(q)
        if gc != g.get(cur):
            continue
        if cur == goal:
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            path.reverse()
            return path
        for nxt in neighbors(cur, free):
            dj, di = nxt[0] - cur[0], nxt[1] - cur[1]
            step = math.sqrt(2.0) if dj and di else 1.0
            ng = gc + step
            if ng < g.get(nxt, math.inf):
                g[nxt] = ng
                parent[nxt] = cur
                h = math.hypot(nxt[0] - goal[0], nxt[1] - goal[1])
                heapq.heappush(q, (ng + h, ng, nxt))
    return []


def connected_with_x_floor(a: tuple[int, int], b: tuple[int, int], free, xs, x_min: float) -> bool:
    if xs[a[1]] < x_min or xs[b[1]] < x_min:
        return False
    q = deque([a])
    seen = {a}
    while q:
        cur = q.popleft()
        if cur == b:
            return True
        for nxt in neighbors(cur, free):
            if xs[nxt[1]] + 1e-9 < x_min or nxt in seen:
                continue
            seen.add(nxt)
            q.append(nxt)
    return False


def path_length(path: list[tuple[int, int]], resolution: float) -> float:
    total = 0.0
    for a, b in zip(path, path[1:]):
        total += resolution * math.hypot(a[0] - b[0], a[1] - b[1])
    return total


def fmt_metric(value, digits=3):
    return 'N/A' if value is None else f'{value:.{digits}f}'

def main() -> int:
    geom = json.loads(GEOMETRY.read_text())
    room = [float(v) for v in geom["room_xy_m"]]
    res = float(geom["grid_resolution_m"])
    inflation = float(geom["design_inflation_m"])
    start = tuple(float(v) for v in geom["start_xy_m"])
    goal = tuple(float(v) for v in geom["goal_xy_m"])
    decoy = tuple(float(v) for v in geom["decoy_probe_xy_m"])
    rects = [[float(v) for v in r] for r in geom["obstacles_xywh_m"]]
    target_y0, target_y1 = map(float, geom["target_branch_y_m"])
    decoy_y0, decoy_y1 = map(float, geom["decoy_branch_y_m"])
    branch_x_min = float(geom["branch_analysis_x_min_m"])

    checks: dict[str, bool] = {}
    checks["schema"] = geom.get("schema") == "S2_4_E_LITERAL_COMPETING_CORRIDORS_GEOMETRY_V1"
    checks["two_truth_walls"] = len(rects) == 2
    checks["walls_inside_room"] = all(
        0 <= r[0] < r[0] + r[2] <= room[0] and 0 <= r[1] < r[1] + r[3] <= room[1]
        for r in rects
    )

    # Analytic widths after applying the same 0.602 m centre-envelope inflation.
    vertical = rects[0]
    top_fork_raw = room[1] - (vertical[1] + vertical[3])
    bottom_fork_raw = vertical[1]
    top_fork_safe = top_fork_raw - 2 * inflation
    bottom_fork_safe = bottom_fork_raw - 2 * inflation
    target_branch_safe = (target_y1 - target_y0) - 2 * inflation
    decoy_branch_safe = (decoy_y1 - decoy_y0) - 2 * inflation
    checks["upper_fork_positive_clearance"] = top_fork_safe >= 0.50
    checks["lower_fork_positive_clearance"] = bottom_fork_safe >= 0.50
    checks["target_branch_positive_clearance"] = target_branch_safe >= 0.75
    checks["decoy_branch_positive_clearance"] = decoy_branch_safe >= 0.75
    checks["decoy_branch_wider_than_target"] = decoy_branch_safe > target_branch_safe

    # Both branch destinations must be initially occluded by the vertical fork wall.
    checks["target_initially_occluded"] = segment_hits_rect(start, goal, vertical)
    checks["decoy_initially_occluded"] = segment_hits_rect(start, decoy, vertical)

    xs, ys, free, inflated = build_grid(room, res, inflation, rects)
    sc = nearest_cell(start, xs, ys)
    gc = nearest_cell(goal, xs, ys)
    dc = nearest_cell(decoy, xs, ys)
    checks["start_free_after_inflation"] = free[sc[0]][sc[1]]
    checks["goal_free_after_inflation"] = free[gc[0]][gc[1]]
    checks["decoy_probe_free_after_inflation"] = free[dc[0]][dc[1]]

    target_path = astar(sc, gc, free)
    decoy_path = astar(sc, dc, free)
    checks["target_route_exists"] = bool(target_path)
    checks["decoy_route_exists"] = bool(decoy_path)

    # A real fork means the upper and lower routes pass opposite ends of the fork wall.
    fork_x_lo = vertical[0] - inflation
    fork_x_hi = vertical[0] + vertical[2] + inflation
    target_fork_y = [ys[j] for j, i in target_path if fork_x_lo <= xs[i] <= fork_x_hi]
    decoy_fork_y = [ys[j] for j, i in decoy_path if fork_x_lo <= xs[i] <= fork_x_hi]
    inflated_top = vertical[1] + vertical[3] + inflation
    inflated_bottom = vertical[1] - inflation
    checks["target_route_uses_upper_fork"] = bool(target_fork_y) and min(target_fork_y) >= inflated_top - res - 1e-9
    checks["decoy_route_uses_lower_fork"] = bool(decoy_fork_y) and max(decoy_fork_y) <= inflated_bottom + res + 1e-9

    # The horizontal divider must make the east-side target/decoy branches distinct.
    checks["east_branches_disconnected"] = not connected_with_x_floor(gc, dc, free, xs, branch_x_min)

    # Count safe free cells in each east-side branch. This is a geometry-level
    # proxy for a larger potential information region, independent of selector scoring.
    target_cells = 0
    decoy_cells = 0
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            if x < branch_x_min or not free[j][i]:
                continue
            if target_y0 <= y <= target_y1:
                target_cells += 1
            if decoy_y0 <= y <= decoy_y1:
                decoy_cells += 1
    checks["decoy_free_area_larger"] = decoy_cells > target_cells

    result = {
        "schema": "S2_4_E_LITERAL_CORRIDOR_GEOMETRY_AUDIT_V1",
        "validator_contract": "S2_4_E_LAYERED_V5",
        "source_geometry": str(GEOMETRY.relative_to(ROOT)),
        "checks": checks,
        "metrics": {
            "inflation_m": inflation,
            "upper_fork_safe_width_m": top_fork_safe,
            "lower_fork_safe_width_m": bottom_fork_safe,
            "target_branch_safe_width_m": target_branch_safe,
            "decoy_branch_safe_width_m": decoy_branch_safe,
            "target_route_length_m": path_length(target_path, res) if target_path else None,
            "decoy_route_length_m": path_length(decoy_path, res) if decoy_path else None,
            "target_east_free_cells": target_cells,
            "decoy_east_free_cells": decoy_cells,
            "decoy_to_target_free_area_ratio": decoy_cells / target_cells if target_cells else None,
            "inflated_obstacles_xywh_m": inflated,
        },
        "pass": all(checks.values()),
        "scope": "static physical-geometry contract only; coupled MATLAB mission remains unvalidated",
    }

    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUT_JSON.write_text(json.dumps(result, indent=2) + "\n")
    m = result["metrics"]
    lines = [
        "S2.4-E literal competing-corridors geometry audit",
        f"inflation: {inflation:.3f} m",
        f"safe fork widths upper/lower: {m['upper_fork_safe_width_m']:.3f} / {m['lower_fork_safe_width_m']:.3f} m",
        f"safe branch widths target/decoy: {m['target_branch_safe_width_m']:.3f} / {m['decoy_branch_safe_width_m']:.3f} m",
        f"A* route lengths target/decoy: {fmt_metric(m['target_route_length_m'])} / {fmt_metric(m['decoy_route_length_m'])} m",
        f"east-side safe free cells target/decoy: {target_cells} / {decoy_cells}",
        f"decoy/target safe-area ratio: {fmt_metric(m['decoy_to_target_free_area_ratio'])}",
        "checks:",
    ]
    lines.extend(f"  {name}: {'PASS' if ok else 'FAIL'}" for name, ok in checks.items())
    lines.append(f"RESULT: {'PASS' if result['pass'] else 'FAIL'}")
    lines.append("NOTE: coupled MATLAB mission validation is still pending.")
    OUT_TXT.write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

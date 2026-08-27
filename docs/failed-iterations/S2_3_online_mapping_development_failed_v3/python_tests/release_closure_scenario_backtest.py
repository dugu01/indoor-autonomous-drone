#!/usr/bin/env python3
"""Release-closing truth-map feasibility checks for revised S2.3 scenarios.

This is a mechanism/backtest only. It does not claim coupled MATLAB validation.
"""
from __future__ import annotations
import heapq
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

ROOM = (6.0, 6.0)
RES = 0.10
INFLATION = 0.602
XS = np.arange(0.0, ROOM[0] + 0.5 * RES, RES)
YS = np.arange(0.0, ROOM[1] + 0.5 * RES, RES)
NX, NY = len(XS), len(YS)
NEIGHBORS = (
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)),
)

Rect = tuple[float, float, float, float, float]

@dataclass(frozen=True)
class Case:
    name: str
    start: tuple[float, float]
    goal: tuple[float, float]
    rects: tuple[Rect, ...]


def point_rect_distance(x: np.ndarray, y: np.ndarray, rect: Rect) -> np.ndarray:
    rx, ry, w, d, _ = rect
    dx = np.maximum.reduce((rx - x, np.zeros_like(x), x - (rx + w)))
    dy = np.maximum.reduce((ry - y, np.zeros_like(y), y - (ry + d)))
    return np.hypot(dx, dy)


def occupancy(rects: Iterable[Rect]) -> np.ndarray:
    xx, yy = np.meshgrid(XS, YS)
    occ = ((xx < INFLATION - 1e-12) | (xx > ROOM[0] - INFLATION + 1e-12) |
           (yy < INFLATION - 1e-12) | (yy > ROOM[1] - INFLATION + 1e-12))
    for rect in rects:
        occ |= point_rect_distance(xx, yy, rect) <= INFLATION + 1e-12
    return occ


def index(point: tuple[float, float]) -> tuple[int, int]:
    return round(point[0] / RES), round(point[1] / RES)


def astar(case: Case) -> np.ndarray | None:
    occ = occupancy(case.rects)
    sx, sy = index(case.start)
    gx, gy = index(case.goal)
    if not (0 <= gx < NX and 0 <= gy < NY) or occ[gy, gx]:
        return None
    if occ[sy, sx]:
        return None
    score = np.full((NY, NX), np.inf)
    score[sy, sx] = 0.0
    came: dict[tuple[int, int], tuple[int, int]] = {}
    queue = [(math.hypot(sx - gx, sy - gy), 0.0, sx, sy)]
    closed: set[tuple[int, int]] = set()
    while queue:
        _, g, x, y = heapq.heappop(queue)
        if (x, y) in closed:
            continue
        closed.add((x, y))
        if (x, y) == (gx, gy):
            cells = []
            cur = (x, y)
            while True:
                cells.append(cur)
                if cur == (sx, sy):
                    break
                cur = came[cur]
            cells.reverse()
            return np.asarray([(XS[ix], YS[iy]) for ix, iy in cells])
        for dx, dy, cost in NEIGHBORS:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < NX and 0 <= ny < NY):
                continue
            if occ[ny, nx] or (nx, ny) in closed:
                continue
            if dx and dy and (occ[y, nx] or occ[ny, x]):
                continue
            ng = g + cost
            if ng < score[ny, nx]:
                score[ny, nx] = ng
                came[(nx, ny)] = (x, y)
                h = math.hypot(nx - gx, ny - gy)
                heapq.heappush(queue, (ng + h, ng, nx, ny))
    return None


def min_clearance(path: np.ndarray, rects: Iterable[Rect]) -> float:
    result = math.inf
    for x, y in path:
        result = min(result, x, ROOM[0] - x, y, ROOM[1] - y)
        for rect in rects:
            result = min(result, float(point_rect_distance(np.asarray(x), np.asarray(y), rect)))
    return result


def intersects_inflated(path: np.ndarray, rect: Rect) -> bool:
    return any(float(point_rect_distance(np.asarray(x), np.asarray(y), rect)) <= INFLATION + 1e-12
               for x, y in path)


def main() -> int:
    base: tuple[Rect, ...] = (
        (1.00, 1.00, 0.50, 0.50, 1.80),
        (4.00, 3.50, 0.50, 0.50, 1.80),
    )
    inserted: Rect = (3.65, 2.10, 0.45, 0.80, 1.90)
    cases = (
        Case('occluded_obstacle', (3.0, 0.8), (5.3, 5.3), (
            (3.15, 1.65, 0.50, 0.65, 1.90),
            (3.40, 2.75, 0.55, 0.75, 1.90),
            (4.40, 4.10, 0.40, 0.50, 1.90),
        )),
        Case('unknown_narrow_passage', (3.0, 0.8), (3.0, 5.25), (
            (0.80, 2.45, 1.45, 0.45, 1.90),
            (3.75, 2.45, 1.45, 0.45, 1.90),
        )),
        Case('goal_requires_scan', (0.85, 0.85), (5.15, 5.15), (
            (2.50, 1.20, 0.45, 3.00, 1.90),
        )),
    )
    failures: list[str] = []
    print('S2.3 release-closing scenario backtest')
    print(f'grid={RES:.2f} m | physical inflation={INFLATION:.3f} m')
    for case in cases:
        path = astar(case)
        passed = path is not None
        if not passed:
            failures.append(f'{case.name}: no truth-map route')
            print(f'{case.name:28s}: FAIL — no truth-map route')
            continue
        clearance = min_clearance(path, case.rects)
        if clearance + 1e-12 < INFLATION:
            failures.append(f'{case.name}: clearance {clearance:.3f} < {INFLATION:.3f}')
            passed = False
        length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        print(f'{case.name:28s}: {"PASS" if passed else "FAIL"} | '
              f'nodes={len(path):3d} length={length:.3f} m min_clearance={clearance:.3f} m')

    before = Case('hidden_before', (3.0, 0.8), (5.3, 5.3), base)
    after = Case('hidden_after', (3.0, 0.8), (5.3, 5.3), base + (inserted,))
    path_before = astar(before)
    path_after = astar(after)
    hidden_ok = (path_before is not None and path_after is not None and
                 intersects_inflated(path_before, inserted))
    if not hidden_ok:
        failures.append('hidden_obstacle_replan: initial route/intersection/alternate-route contract failed')
    print(f'hidden_obstacle_replan       : {"PASS" if hidden_ok else "FAIL"} | '
          f'initial_route={path_before is not None} intersects_insert={path_before is not None and intersects_inflated(path_before, inserted)} '
          f'alternate_route={path_after is not None}')

    # The unreachable goal must remain unreachable at the same frozen radius.
    unreachable = Case('unreachable_goal', (3.0, 0.8), (4.9, 4.9), (
        (4.20, 4.20, 1.40, 0.30, 2.20),
        (4.20, 5.30, 1.40, 0.30, 2.20),
        (4.20, 4.20, 0.30, 1.40, 2.20),
        (5.30, 4.20, 0.30, 1.40, 2.20),
    ))
    unreachable_ok = astar(unreachable) is None
    if not unreachable_ok:
        failures.append('unreachable_goal: truth-map route unexpectedly exists')
    print(f'unreachable_goal            : {"PASS" if unreachable_ok else "FAIL"} | no truth-map route expected')

    if failures:
        print('\nFAILURES:')
        for failure in failures:
            print(' -', failure)
        return 1
    print('\nRESULT: PASS')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

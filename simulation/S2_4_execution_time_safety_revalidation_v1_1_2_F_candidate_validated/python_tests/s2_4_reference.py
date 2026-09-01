from __future__ import annotations

from dataclasses import dataclass, field
from collections import deque
import hashlib
import heapq
import math
from typing import Iterable, Sequence

import numpy as np
from scipy.ndimage import distance_transform_edt


@dataclass(frozen=True)
class S24Config:
    resolution: float = 0.10
    effective_inflation_m: float = 0.602
    lidar_range_m: float = 6.5
    depth_range_m: float = 4.5
    depth_hfov_rad: float = math.radians(87.0)
    stop_extra_margin_m: float = 0.08
    min_frontier_cells: int = 2
    max_frontier_extent_cells: int = 18
    target_corridor_radius_cells: int = 3
    min_visible_unknown: int = 1
    candidate_radii_m: tuple[float, ...] = (0.7, 1.0, 1.3)
    candidate_angles: int = 16
    stale_age_s: float = 8.0
    max_frontier_failures: int = 2
    blacklist_cooldown_updates: int = 3
    diagnostic_unknown_cost: float = 5.0
    w_information: float = 0.25
    w_target: float = 0.35
    w_travel: float = 0.10
    w_static: float = 0.08
    w_dynamic: float = 0.10
    w_uncertainty: float = 0.05
    w_yaw: float = 0.02
    w_return: float = 0.05


@dataclass
class GridBelief:
    known_free: np.ndarray
    occupied: np.ndarray
    unknown: np.ndarray
    entropy: np.ndarray | None = None
    stale_free: np.ndarray | None = None
    dynamic_risk: np.ndarray | None = None
    source_quality: np.ndarray | None = None
    navigation_blocked: np.ndarray | None = None
    timestamp: float = 0.0
    resolution: float = 0.10

    def __post_init__(self) -> None:
        shape = self.known_free.shape
        for name in ("occupied", "unknown"):
            if getattr(self, name).shape != shape:
                raise ValueError(f"{name} shape mismatch")
        if np.any(self.known_free & self.occupied):
            raise ValueError("known_free overlaps occupied")
        if np.any(self.known_free & self.unknown):
            raise ValueError("known_free overlaps unknown")
        if np.any(self.occupied & self.unknown):
            raise ValueError("occupied overlaps unknown")
        if self.entropy is None:
            self.entropy = np.where(self.unknown, 1.0, 0.15).astype(float)
        if self.stale_free is None:
            self.stale_free = np.zeros(shape, dtype=bool)
        if self.dynamic_risk is None:
            self.dynamic_risk = np.zeros(shape, dtype=float)
        if self.source_quality is None:
            self.source_quality = np.ones(shape, dtype=float)
        if self.navigation_blocked is None:
            self.navigation_blocked = self.occupied.copy()
        if self.navigation_blocked.shape != shape:
            raise ValueError("navigation_blocked shape mismatch")
        if np.any(self.known_free & self.occupied):
            raise ValueError("known_free overlaps hard occupied")

    @property
    def shape(self) -> tuple[int, int]:
        return self.known_free.shape

    def xy(self, cell: tuple[int, int]) -> np.ndarray:
        y, x = cell
        return np.array([x * self.resolution, y * self.resolution], dtype=float)

    def cell(self, xy: Sequence[float]) -> tuple[int, int]:
        x = int(math.floor(float(xy[0]) / self.resolution + 0.5))
        y = int(math.floor(float(xy[1]) / self.resolution + 0.5))
        return y, x

    def inside(self, cell: tuple[int, int]) -> bool:
        y, x = cell
        return 0 <= y < self.shape[0] and 0 <= x < self.shape[1]


@dataclass
class Frontier:
    track_id: int
    cells: tuple[tuple[int, int], ...]
    centroid: np.ndarray
    geometry_hash: str
    state: str = "ACTIVE"
    failures: int = 0
    blocked_until_update: int = -1
    target_relevance: float = 0.0


@dataclass
class Candidate:
    candidate_id: int
    frontier_track_id: int
    cell: tuple[int, int]
    yaw: float
    path: tuple[tuple[int, int], ...]
    accepted: bool
    rejection_reasons: tuple[str, ...] = ()
    visible_unknown: tuple[tuple[int, int], ...] = ()
    information_gain: float = 0.0
    target_relevance: float = 0.0
    travel_cost: float = 0.0
    static_risk: float = 0.0
    dynamic_risk: float = 0.0
    uncertainty_risk: float = 0.0
    yaw_cost: float = 0.0
    return_risk: float = 0.0
    tier: int = 3
    utility: float = -math.inf
    action: str = "NONE"


@dataclass
class FrontierManager:
    next_track_id: int = 1
    tracks: dict[int, Frontier] = field(default_factory=dict)
    update_index: int = 0

    def update(self, clusters: list[tuple[tuple[int, int], ...]]) -> list[Frontier]:
        self.update_index += 1
        new_frontiers: list[Frontier] = []
        unmatched = set(self.tracks)
        for cells in clusters:
            h = geometry_hash(cells)
            centroid = np.mean(np.asarray(cells, dtype=float), axis=0)
            best_id = None
            best_key = None
            new_set = set(cells)
            for tid in sorted(unmatched):
                old = self.tracks[tid]
                overlap = len(new_set.intersection(old.cells))
                dist = float(np.linalg.norm(centroid - old.centroid))
                key = (-overlap, dist, tid)
                if best_key is None or key < best_key:
                    best_key = key
                    best_id = tid
            if best_id is not None and (best_key[0] < 0 or best_key[1] <= 2.5):
                old = self.tracks[best_id]
                f = Frontier(best_id, cells, centroid, h, old.state, old.failures, old.blocked_until_update)
                unmatched.remove(best_id)
            else:
                f = Frontier(self.next_track_id, cells, centroid, h)
                self.next_track_id += 1
            self.tracks[f.track_id] = f
            new_frontiers.append(f)
        for tid in unmatched:
            self.tracks[tid].state = "RESOLVED"
        return sorted(new_frontiers, key=lambda f: f.track_id)

    def record_failure(self, track_id: int, cfg: S24Config) -> None:
        f = self.tracks[track_id]
        f.failures += 1
        if f.failures >= cfg.max_frontier_failures:
            f.state = "TEMPORARILY_BLOCKED"
            f.blocked_until_update = self.update_index + cfg.blacklist_cooldown_updates

    def eligible(self, f: Frontier) -> bool:
        if f.state == "TEMPORARILY_BLOCKED" and self.update_index < f.blocked_until_update:
            return False
        if f.state in {"RESOLVED", "EXHAUSTED", "BLACKLISTED"}:
            return False
        return True


def geometry_hash(cells: Iterable[tuple[int, int]]) -> str:
    payload = ";".join(f"{y},{x}" for y, x in sorted(cells)).encode()
    return hashlib.sha256(payload).hexdigest()[:16]


def neighbors4(cell: tuple[int, int]) -> tuple[tuple[int, int], ...]:
    y, x = cell
    return ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1))


def neighbors8(cell: tuple[int, int]) -> tuple[tuple[int, int], ...]:
    y, x = cell
    return tuple((y + dy, x + dx) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dx or dy)


def frontier_mask(grid: GridBelief) -> np.ndarray:
    out = np.zeros(grid.shape, dtype=bool)
    ys, xs = np.nonzero(grid.known_free)
    for y, x in zip(ys.tolist(), xs.tolist()):
        if any(grid.inside(n) and grid.unknown[n] for n in neighbors4((y, x))):
            out[y, x] = True
    return out


def _split_cluster_deterministic(
    cells: tuple[tuple[int, int], ...], max_extent_cells: int
) -> list[tuple[tuple[int, int], ...]]:
    """Split a long frontier deterministically along its principal axis.

    FUEL-style frontier information structures subdivide large frontier surfaces so
    that local viewpoints can be evaluated independently.  The sign of the
    principal axis and every tie are normalized to make replay hashes stable.
    """
    if len(cells) <= 1:
        return [cells]
    a = np.asarray(cells, dtype=float)
    extent = np.ptp(a, axis=0)
    if float(extent.max()) <= float(max_extent_cells):
        return [cells]
    centered = a - a.mean(axis=0, keepdims=True)
    cov = centered.T @ centered
    vals, vecs = np.linalg.eigh(cov)
    axis = vecs[:, int(np.argmax(vals))]
    # Normalize eigenvector sign deterministically.
    nz = np.flatnonzero(np.abs(axis) > 1e-12)
    if nz.size and axis[nz[0]] < 0:
        axis = -axis
    proj = centered @ axis
    order = sorted(range(len(cells)), key=lambda i: (float(proj[i]), cells[i][0], cells[i][1]))
    mid = len(order) // 2
    left = tuple(sorted(cells[i] for i in order[:mid]))
    right = tuple(sorted(cells[i] for i in order[mid:]))
    if not left or not right:
        return [cells]
    return _split_cluster_deterministic(left, max_extent_cells) + _split_cluster_deterministic(right, max_extent_cells)


def cluster_frontiers(
    mask: np.ndarray, min_cells: int = 1, max_extent_cells: int | None = None
) -> list[tuple[tuple[int, int], ...]]:
    seen = np.zeros(mask.shape, dtype=bool)
    raw: list[tuple[tuple[int, int], ...]] = []
    for y in range(mask.shape[0]):
        for x in range(mask.shape[1]):
            if not mask[y, x] or seen[y, x]:
                continue
            q = deque([(y, x)])
            seen[y, x] = True
            cells: list[tuple[int, int]] = []
            while q:
                c = q.popleft()
                cells.append(c)
                for n in neighbors8(c):
                    ny, nx = n
                    if 0 <= ny < mask.shape[0] and 0 <= nx < mask.shape[1] and mask[n] and not seen[n]:
                        seen[n] = True
                        q.append(n)
            if len(cells) >= min_cells:
                raw.append(tuple(sorted(cells)))
    clusters: list[tuple[tuple[int, int], ...]] = []
    for cells in raw:
        parts = [cells] if max_extent_cells is None else _split_cluster_deterministic(cells, max_extent_cells)
        clusters.extend(p for p in parts if len(p) >= min_cells)
    return sorted(clusters, key=lambda c: (c[0], len(c), geometry_hash(c)))


def astar(
    grid: GridBelief,
    start: tuple[int, int],
    goal: tuple[int, int],
    *,
    allow_unknown: bool = False,
    unknown_cost: float = 5.0,
) -> tuple[tuple[int, int], ...]:
    if not grid.inside(start) or not grid.inside(goal):
        return ()
    blocked = grid.occupied if allow_unknown else grid.navigation_blocked
    if blocked[start] or blocked[goal]:
        return ()
    if not allow_unknown and (not grid.known_free[start] or not grid.known_free[goal]):
        return ()
    pq: list[tuple[float, float, int, int]] = [(0.0, 0.0, start[0], start[1])]
    g = {start: 0.0}
    parent: dict[tuple[int, int], tuple[int, int]] = {}
    while pq:
        _, gc, y, x = heapq.heappop(pq)
        cur = (y, x)
        if gc != g.get(cur):
            continue
        if cur == goal:
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            return tuple(reversed(path))
        for n in neighbors8(cur):
            if not grid.inside(n) or blocked[n]:
                continue
            if not allow_unknown and not grid.known_free[n]:
                continue
            if allow_unknown and not (grid.known_free[n] or grid.unknown[n]):
                continue
            dy, dx = n[0] - y, n[1] - x
            step = math.sqrt(2.0) if dy and dx else 1.0
            if grid.unknown[n]:
                step *= unknown_cost
            ng = gc + step
            if ng + 1e-12 < g.get(n, math.inf):
                g[n] = ng
                parent[n] = cur
                h = math.hypot(goal[0] - n[0], goal[1] - n[1])
                heapq.heappush(pq, (ng + h, ng, n[0], n[1]))
    return ()


def line_cells(a: tuple[int, int], b: tuple[int, int]) -> tuple[tuple[int, int], ...]:
    y0, x0 = a
    y1, x1 = b
    n = max(abs(y1 - y0), abs(x1 - x0), 1)
    cells = []
    for k in range(n + 1):
        y = int(math.floor(y0 + (y1 - y0) * k / n + 0.5))
        x = int(math.floor(x0 + (x1 - x0) * k / n + 0.5))
        if not cells or cells[-1] != (y, x):
            cells.append((y, x))
    return tuple(cells)


def visible_unknown_cells(
    grid: GridBelief,
    origin: tuple[int, int],
    yaw: float,
    max_range_m: float,
    fov_rad: float,
    ray_count: int = 121,
) -> tuple[tuple[int, int], ...]:
    max_steps = int(math.floor(max_range_m / grid.resolution))
    if fov_rad >= 2 * math.pi - 1e-9:
        angles = np.linspace(-math.pi, math.pi, ray_count, endpoint=False)
    else:
        angles = np.linspace(yaw - fov_rad / 2, yaw + fov_rad / 2, ray_count)
    vis: set[tuple[int, int]] = set()
    oy, ox = origin
    for a in angles:
        for step in range(1, max_steps + 1):
            c = (
                int(math.floor(oy + step * math.sin(a) + 0.5)),
                int(math.floor(ox + step * math.cos(a) + 0.5)),
            )
            if not grid.inside(c):
                break
            if grid.occupied[c]:
                break
            if grid.unknown[c]:
                vis.add(c)
                break
    return tuple(sorted(vis))


def diagnostic_target_corridor(
    grid: GridBelief, start: tuple[int, int], target: tuple[int, int], cfg: S24Config
) -> tuple[tuple[int, int], ...]:
    p = astar(grid, start, target, allow_unknown=True, unknown_cost=cfg.diagnostic_unknown_cost)
    unknown_core = [c for c in p if grid.unknown[c]]
    tube: set[tuple[int, int]] = set()
    r = cfg.target_corridor_radius_cells
    for y, x in unknown_core:
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dy * dy + dx * dx <= r * r:
                    c = (y + dy, x + dx)
                    if grid.inside(c) and grid.unknown[c]:
                        tube.add(c)
    return tuple(sorted(tube))


def target_route_status(
    grid: GridBelief, start: tuple[int, int], target: tuple[int, int]
) -> tuple[str, tuple[tuple[int, int], ...]]:
    """Resolve target-first behavior before exploration is considered."""
    p = astar(grid, start, target)
    if not p:
        return "NO_KNOWN_FREE_ROUTE", ()
    if any(bool(grid.stale_free[c]) for c in p):
        return "HOLD_AND_RESCAN_STALE_ROUTE", p
    if path_dynamic_risk(grid, p) >= 0.80:
        return "WAIT_OR_REPLAN_DYNAMIC_ROUTE", p
    return "TARGET_ROUTE_AVAILABLE", p


def path_length(path: Sequence[tuple[int, int]], resolution: float) -> float:
    if len(path) < 2:
        return 0.0
    return resolution * sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path[:-1], path[1:]))


def path_static_risk(grid: GridBelief, path: Sequence[tuple[int, int]], cfg: S24Config) -> float:
    if not path:
        return 1.0
    clearance = distance_transform_edt(~grid.occupied) * grid.resolution
    vals = np.array([clearance[c] for c in path], dtype=float)
    # Occupancy already includes effective inflation. This measures residual margin.
    return float(np.clip(np.exp(-np.maximum(vals - grid.resolution, 0.0) / 0.35).max(), 0.0, 1.0))


def path_dynamic_risk(grid: GridBelief, path: Sequence[tuple[int, int]]) -> float:
    return float(max((grid.dynamic_risk[c] for c in path), default=0.0))


def path_uncertainty_risk(grid: GridBelief, path: Sequence[tuple[int, int]]) -> float:
    if not path:
        return 1.0
    vals = []
    for c in path:
        vals.append(0.55 * float(grid.entropy[c]) + 0.30 * float(grid.stale_free[c]) + 0.15 * (1 - float(grid.source_quality[c])))
    return float(np.clip(np.mean(vals), 0.0, 1.0))


def normalize(v: float, scale: float) -> float:
    return float(np.clip(v / max(scale, 1e-12), 0.0, 1.0))


def generate_candidates(
    grid: GridBelief,
    frontiers: Sequence[Frontier],
    start: tuple[int, int],
    target: tuple[int, int],
    cfg: S24Config,
    manager: FrontierManager | None = None,
    *,
    lidar_quality: float = 1.0,
    depth_quality: float = 1.0,
) -> list[Candidate]:
    corridor = set(diagnostic_target_corridor(grid, start, target, cfg))
    clearance = distance_transform_edt(~grid.occupied) * grid.resolution
    candidates: list[Candidate] = []
    cid = 1
    for frontier in sorted(frontiers, key=lambda f: f.track_id):
        if manager is not None and not manager.eligible(frontier):
            continue
        cy, cx = frontier.centroid
        proposed: set[tuple[int, int]] = set()
        for radius_m in cfg.candidate_radii_m:
            r = radius_m / grid.resolution
            for k in range(cfg.candidate_angles):
                a = 2 * math.pi * k / cfg.candidate_angles
                proposed.add((int(math.floor(cy + r * math.sin(a) + 0.5)), int(math.floor(cx + r * math.cos(a) + 0.5))))
        # Also evaluate representative frontier support cells, which is important
        # for narrow but valid doors where a ring sample may skip the safe cell.
        proposed.update(frontier.cells[:: max(1, len(frontier.cells) // 8)])
        for cell in sorted(proposed):
            reasons: list[str] = []
            if not grid.inside(cell):
                reasons.append("OUTSIDE_MAP")
            elif grid.navigation_blocked[cell]:
                reasons.append("POSITION_OCCUPIED_OR_UNKNOWN_INFLATED")
            elif grid.unknown[cell] or not grid.known_free[cell]:
                reasons.append("POSITION_UNKNOWN")
            if reasons:
                candidates.append(Candidate(cid, frontier.track_id, cell, 0.0, (), False, tuple(reasons)))
                cid += 1
                continue
            if clearance[cell] < grid.resolution * 1.5:
                reasons.append("INSUFFICIENT_STATIC_CLEARANCE")
            p = astar(grid, start, cell)
            if not p:
                reasons.append("UNREACHABLE_KNOWN_FREE")
            if p and any(grid.stale_free[c] for c in p):
                reasons.append("STALE_ROUTE_REQUIRES_RESCAN")
            # Target-facing yaw is deterministic.
            yaw = math.atan2(target[0] - cell[0], target[1] - cell[1])
            lidar_vis = visible_unknown_cells(grid, cell, yaw, cfg.lidar_range_m, 2 * math.pi) if lidar_quality > 0 else ()
            depth_vis = visible_unknown_cells(grid, cell, yaw, cfg.depth_range_m, cfg.depth_hfov_rad) if depth_quality > 0 else ()
            weighted_visible: dict[tuple[int, int], float] = {}
            for c in lidar_vis:
                weighted_visible[c] = max(weighted_visible.get(c, 0.0), lidar_quality)
            for c in depth_vis:
                weighted_visible[c] = max(weighted_visible.get(c, 0.0), depth_quality)
            visible = tuple(sorted(weighted_visible))
            if len(visible) < cfg.min_visible_unknown:
                reasons.append("INSUFFICIENT_VISIBLE_UNKNOWN")
            # Reverse of a known-free path is a valid retreat in shadow mode.
            if p and len(p) < 2:
                reasons.append("RETREAT_ROUTE_INVALID")
            dyn = path_dynamic_risk(grid, p)
            if dyn >= 0.80:
                reasons.append("DYNAMIC_ROUTE_CROSSING")
            accepted = not reasons
            info = float(sum(grid.entropy[c] * weighted_visible[c] for c in visible))
            rel = float(sum(grid.entropy[c] * weighted_visible.get(c, 0.0) for c in corridor))
            target_vec = np.asarray(target, dtype=float) - np.asarray(start, dtype=float)
            frontier_vec = np.asarray(frontier.centroid, dtype=float) - np.asarray(start, dtype=float)
            denom = float(np.linalg.norm(target_vec) * np.linalg.norm(frontier_vec))
            alignment = float(np.dot(target_vec, frontier_vec) / denom) if denom > 1e-12 else -1.0
            projected_progress = float(np.dot(frontier_vec, target_vec) / max(np.linalg.norm(target_vec), 1e-12))
            progress_certificate = alignment >= 0.80 and projected_progress >= 2.0
            tier = 1 if rel > 1e-9 else (2 if info > 0 and progress_certificate else 3)
            travel = path_length(p, grid.resolution)
            stat = path_static_risk(grid, p, cfg)
            unc = path_uncertainty_risk(grid, p)
            yaw_cost = abs(math.atan2(math.sin(yaw), math.cos(yaw))) / math.pi
            ret = 0.0 if p and len(p) >= 2 else 1.0
            max_info = max(1.0, float(np.count_nonzero(grid.unknown)))
            max_rel = max(1.0, float(len(corridor)))
            utility = (
                cfg.w_information * normalize(info, max_info)
                + cfg.w_target * normalize(rel, max_rel)
                - cfg.w_travel * normalize(travel, 10.0)
                - cfg.w_static * stat
                - cfg.w_dynamic * dyn
                - cfg.w_uncertainty * unc
                - cfg.w_yaw * yaw_cost
                - cfg.w_return * ret
            )
            if tier == 3:
                accepted = False
                reasons.append("IRRELEVANT_EXPLORATION")
            action = "FLY_AND_SCAN" if accepted else ("HOLD_AND_RESCAN" if "STALE_ROUTE_REQUIRES_RESCAN" in reasons else "REJECT")
            candidates.append(
                Candidate(
                    cid,
                    frontier.track_id,
                    cell,
                    yaw,
                    p,
                    accepted,
                    tuple(sorted(set(reasons))),
                    visible,
                    info,
                    rel,
                    travel,
                    stat,
                    dyn,
                    unc,
                    yaw_cost,
                    ret,
                    tier,
                    utility,
                    action,
                )
            )
            cid += 1
    return candidates


def select_candidate(candidates: Sequence[Candidate]) -> Candidate | None:
    valid = [c for c in candidates if c.accepted]
    if not valid:
        return None
    return min(
        valid,
        key=lambda c: (
            c.tier,
            -c.utility,
            -c.target_relevance,
            -c.information_gain,
            c.dynamic_risk,
            c.static_risk,
            c.travel_cost,
            c.frontier_track_id,
            c.candidate_id,
        ),
    )


def deterministic_digest(frontiers: Sequence[Frontier], candidates: Sequence[Candidate]) -> str:
    rows = []
    for f in frontiers:
        rows.append(("F", f.track_id, f.geometry_hash, f.state, f.failures, f.blocked_until_update))
    for c in candidates:
        rows.append(
            (
                "C",
                c.candidate_id,
                c.frontier_track_id,
                c.cell,
                round(c.yaw, 12),
                c.accepted,
                c.rejection_reasons,
                tuple(c.path),
                tuple(c.visible_unknown),
                round(c.information_gain, 12),
                round(c.target_relevance, 12),
                round(c.utility, 12),
                c.tier,
            )
        )
    return hashlib.sha256(repr(rows).encode()).hexdigest()


def predict_track(
    position: Sequence[float],
    velocity: Sequence[float],
    covariance: np.ndarray,
    times: Sequence[float],
    acceleration_noise: float = 0.35,
) -> list[tuple[np.ndarray, np.ndarray]]:
    p = np.asarray(position, dtype=float)
    v = np.asarray(velocity, dtype=float)
    cov = np.asarray(covariance, dtype=float)
    out = []
    for t in times:
        mean = p + v * t
        growth = (0.5 * acceleration_noise * t * t) ** 2
        out.append((mean, cov + np.eye(2) * growth))
    return out


def route_crossing_risk(
    path_xy: np.ndarray,
    path_times: np.ndarray,
    predictions: Sequence[tuple[np.ndarray, np.ndarray]],
    safety_radius: float,
) -> float:
    """Return a deterministic space-time route/track overlap proxy.

    ``path_xy[k]`` and ``predictions[k]`` must refer to the same arrival time
    ``path_times[k]``.  The times are part of the public contract even though
    the prediction sequence is already evaluated at those timestamps; checking
    them here prevents accidental space-only comparisons.
    """
    path_xy = np.asarray(path_xy, dtype=float)
    path_times = np.asarray(path_times, dtype=float).reshape(-1)
    if path_xy.ndim != 2 or path_xy.shape[1] != 2:
        raise ValueError("path_xy must have shape (N, 2)")
    if len(path_xy) != len(path_times) or len(path_xy) != len(predictions):
        raise ValueError("path, time, and prediction sequences must be aligned")
    if np.any(~np.isfinite(path_xy)) or np.any(~np.isfinite(path_times)):
        raise ValueError("path and arrival times must be finite")
    if np.any(np.diff(path_times) < -1e-12):
        raise ValueError("path arrival times must be nondecreasing")
    if safety_radius <= 0:
        raise ValueError("safety_radius must be positive")

    risk = 0.0
    for point, (mean, cov) in zip(path_xy, predictions):
        mean = np.asarray(mean, dtype=float).reshape(2)
        cov = np.asarray(cov, dtype=float).reshape(2, 2)
        sigma = math.sqrt(max(float(np.linalg.eigvalsh(cov).max()), 1e-12))
        d = float(np.linalg.norm(point - mean))
        z = (d - safety_radius) / sigma
        risk = max(risk, float(math.exp(-0.5 * max(z, 0.0) ** 2)))
    return float(np.clip(risk, 0.0, 1.0))


def incremental_frontier_update(
    previous_mask: np.ndarray | None,
    previous_grid: GridBelief | None,
    grid: GridBelief,
) -> tuple[np.ndarray, np.ndarray]:
    """Update only cells whose classification or neighbours changed.

    Returns the incremental mask and the dirty region.  Validation compares the
    result byte-for-byte with ``frontier_mask(grid)``.
    """
    full = frontier_mask(grid)
    if previous_mask is None or previous_grid is None:
        return full, np.ones(grid.shape, dtype=bool)
    changed = (
        (previous_grid.known_free != grid.known_free)
        | (previous_grid.occupied != grid.occupied)
        | (previous_grid.unknown != grid.unknown)
        | (previous_grid.navigation_blocked != grid.navigation_blocked)
    )
    # A frontier predicate depends on the cell and its 4-neighbours.
    dirty = changed.copy()
    for _ in range(1):
        dirty |= np.roll(changed, 1, axis=0)
        dirty |= np.roll(changed, -1, axis=0)
        dirty |= np.roll(changed, 1, axis=1)
        dirty |= np.roll(changed, -1, axis=1)
    dirty[0, :] = dirty[-1, :] = True
    dirty[:, 0] = dirty[:, -1] = True
    out = previous_mask.copy()
    out[dirty] = full[dirty]
    return out, dirty

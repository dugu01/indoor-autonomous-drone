from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import h5py
import numpy as np
from scipy.ndimage import binary_dilation

from s2_4_reference import (
    FrontierManager,
    GridBelief,
    S24Config,
    astar,
    cluster_frontiers,
    deterministic_digest,
    frontier_mask,
    generate_candidates,
    incremental_frontier_update,
    select_candidate,
    target_route_status,
)

CFG = S24Config()


def scalar(dataset) -> float:
    return float(np.asarray(dataset).reshape(-1)[0])


def matlab_char(dataset) -> str:
    return ''.join(chr(int(x)) for x in np.asarray(dataset).reshape(-1))


def metric_inflate(mask: np.ndarray, radius_m: float, resolution: float) -> np.ndarray:
    r = int(math.floor(radius_m / resolution + 1e-12))
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    footprint = np.hypot(xx * resolution, yy * resolution) <= radius_m + 1e-12
    return binary_dilation(mask, structure=footprint)


def digest_arrays(*arrays: np.ndarray) -> str:
    h = hashlib.sha256()
    for a in arrays:
        aa = np.ascontiguousarray(a)
        h.update(str(aa.shape).encode())
        h.update(str(aa.dtype).encode())
        h.update(aa.tobytes())
    return h.hexdigest()


def read_snapshot(file: h5py.File, ref, inflation: float, resolution: float) -> tuple[GridBelief, float, int]:
    grp = file[ref]
    known = np.asarray(grp['knownFree'], dtype=bool)
    unknown_raw = np.asarray(grp['unknown'], dtype=bool)
    static = np.asarray(grp['staticOccupied'], dtype=bool)
    dynamic = np.asarray(grp['dynamicOccupied'], dtype=bool)
    occupied = metric_inflate(static | dynamic, inflation, resolution)
    known = known & ~occupied
    unknown = unknown_raw & ~occupied & ~known
    navigation_blocked = occupied | metric_inflate(unknown, inflation, resolution)
    grid = GridBelief(known, occupied, unknown, navigation_blocked=navigation_blocked, resolution=resolution, timestamp=scalar(grp['time']))
    return grid, scalar(grp['time']), int(scalar(grp['version']))


def count_replay_sources(file: h5py.File) -> dict[str, int]:
    refs = np.asarray(file['maps/perceptionReplay']).reshape(-1)
    out = {
        'records': len(refs), 'accepted': 0, 'rejected': 0,
        'lidar_packets': 0, 'depth_packets': 0,
        'lidar_rays': 0, 'depth_rays': 0,
        'lidar_hits': 0, 'depth_hits': 0,
    }
    for ref in refs:
        grp = file[ref]
        accepted = bool(scalar(grp['update/accepted']))
        out['accepted' if accepted else 'rejected'] += 1
        if not accepted:
            continue
        packet = grp['packet']
        if bool(scalar(packet['hasLidarScan'])):
            out['lidar_packets'] += 1
            ranges = np.asarray(packet['lidarRanges']).reshape(-1)
            hits = np.asarray(packet['lidarHits'], dtype=bool).reshape(-1)
            out['lidar_rays'] += int(np.isfinite(ranges).sum())
            out['lidar_hits'] += int(hits.sum())
        if bool(scalar(packet['hasDepthRays'])):
            out['depth_packets'] += 1
            ranges = np.asarray(packet['depthRanges']).reshape(-1)
            hits = np.asarray(packet['depthHits'], dtype=bool).reshape(-1)
            out['depth_rays'] += int(np.isfinite(ranges).sum())
            out['depth_hits'] += int(hits.sum())
    return out


def compute_uncertainty(file: h5py.File) -> dict:
    pmap = file['maps/probabilisticMap']
    log_odds = np.asarray(pmap['logOdds'], dtype=np.float64)
    probability = 1.0 / (1.0 + np.exp(-log_odds))
    eps = np.finfo(np.float64).tiny
    entropy = -(probability * np.log2(np.clip(probability, eps, 1.0)) +
                (1.0 - probability) * np.log2(np.clip(1.0 - probability, eps, 1.0)))
    count = np.asarray(pmap['observationCount'], dtype=np.float64)
    last = np.asarray(pmap['lastObserved'], dtype=np.float64)
    t = scalar(pmap['lastUpdateTime'])
    age = np.where(np.isfinite(last), np.maximum(t - last, 0.0), np.inf)
    static = np.asarray(pmap['staticOccupied'], dtype=bool)
    dynamic_lo = np.asarray(pmap['dynamicLogOdds'], dtype=np.float64)
    dynamic_p = 1.0 / (1.0 + np.exp(-dynamic_lo))
    dynamic_hits = np.asarray(pmap['dynamicHitCount'], dtype=np.float64)
    static_conf = np.where(static, 1.0, (1.0 - entropy) * (1.0 - np.exp(-count / 4.0)) * np.exp(-np.minimum(age, 1e6) / 30.0))
    dynamic_conf = dynamic_p * (1.0 - np.exp(-dynamic_hits / 3.0))
    final_grid = file['maps/finalGrid']
    known_free = np.asarray(final_grid['knownFree'], dtype=bool)
    last_xy = np.asarray(final_grid['lastObservedXY'], dtype=np.float64)
    stale_free = known_free & ((t - last_xy) > CFG.stale_age_s)
    return {
        'entropy_min': float(entropy.min()),
        'entropy_max': float(entropy.max()),
        'finite_entropy': bool(np.isfinite(entropy).all()),
        'observation_total': int(count.astype(np.uint64).sum()),
        'stale_free_cells': int(stale_free.sum()),
        'static_confidence_range': [float(static_conf.min()), float(static_conf.max())],
        'dynamic_confidence_range': [float(dynamic_conf.min()), float(dynamic_conf.max())],
        'digest': digest_arrays(entropy, count, age, static_conf, dynamic_conf, stale_free),
    }


def replay_once(mat_path: Path) -> dict:
    with h5py.File(mat_path, 'r') as file:
        scenario = matlab_char(file['summary/scenario'])
        resolution = scalar(file['maps/finalGrid/resolution'])
        inflation = scalar(file['maps/finalGrid/inflationRadius'])
        if abs(resolution - 0.10) > 1e-12 or abs(inflation - 0.602) > 1e-9:
            raise AssertionError(f'unexpected geometry contract: {resolution=} {inflation=}')
        times = np.asarray(file['log/t']).reshape(-1)
        est = np.asarray(file['log/estP'])
        snap_times = np.asarray(file['log/mapSnapshotTimes']).reshape(-1)
        refs = np.asarray(file['log/mapSnapshots']).reshape(-1)
        goal_xy = np.asarray(file['scenario/goal']).reshape(-1)[:2]
        target = (int(math.floor(goal_xy[1] / resolution + 0.5)), int(math.floor(goal_xy[0] / resolution + 0.5)))

        manager = FrontierManager()
        previous_grid = None
        previous_frontier_mask = None
        replay_rows = []
        incremental_exact = True
        accepted_candidate_count = 0
        unsafe_candidate_count = 0
        max_active_frontiers = 0
        max_tracks_created = 0

        for index, (ref, snap_time) in enumerate(zip(refs, snap_times)):
            grid, map_time, version = read_snapshot(file, ref, inflation, resolution)
            nearest = int(np.argmin(np.abs(times - snap_time)))
            start_xy = est[:2, nearest]
            start = (int(math.floor(start_xy[1] / resolution + 0.5)), int(math.floor(start_xy[0] / resolution + 0.5)))
            if not grid.inside(start) or not grid.known_free[start]:
                free_cells = np.argwhere(grid.known_free)
                if free_cells.size == 0:
                    previous_grid, previous_frontier_mask = grid, frontier_mask(grid)
                    replay_rows.append((index, version, 0, 0, 'NO_KNOWN_FREE_START'))
                    continue
                j = int(np.argmin(np.sum((free_cells - np.asarray(start)) ** 2, axis=1)))
                start = tuple(int(x) for x in free_cells[j])

            full_mask = frontier_mask(grid)
            inc_mask, dirty = incremental_frontier_update(previous_frontier_mask, previous_grid, grid)
            incremental_exact &= bool(np.array_equal(full_mask, inc_mask))
            clusters = cluster_frontiers(inc_mask, CFG.min_frontier_cells, CFG.max_frontier_extent_cells)
            frontiers = manager.update(clusters)
            max_active_frontiers = max(max_active_frontiers, len(frontiers))
            max_tracks_created = max(max_tracks_created, manager.next_track_id - 1)

            status, target_path = target_route_status(grid, start, target)
            selected = None
            candidates = []
            # Candidate scoring is evaluated at every fourth snapshot and whenever
            # no target route exists.  Frontier extraction itself runs every snapshot.
            if status != 'TARGET_ROUTE_AVAILABLE' and (index % 4 == 0 or index == len(refs) - 1):
                candidates = generate_candidates(grid, frontiers, start, target, CFG, manager)
                selected = select_candidate(candidates)
                for c in candidates:
                    if c.accepted:
                        accepted_candidate_count += 1
                        safe = grid.inside(c.cell) and grid.known_free[c.cell] and not grid.navigation_blocked[c.cell]
                        safe &= bool(c.path) and all(grid.known_free[x] and not grid.navigation_blocked[x] for x in c.path)
                        if not safe:
                            unsafe_candidate_count += 1
            digest = deterministic_digest(frontiers, candidates)
            replay_rows.append((index, version, int(dirty.sum()), len(frontiers), status,
                                0 if selected is None else selected.frontier_track_id, digest))
            previous_grid, previous_frontier_mask = grid, inc_mask

        uncertainty = compute_uncertainty(file)
        sources = count_replay_sources(file)
        summary = {
            'schema': 'S2_4_AD_RECORDED_SHADOW_REPLAY_V1',
            'mat_file': mat_path.name,
            'scenario': scenario,
            'snapshots': len(refs),
            'map_resolution_m': resolution,
            'effective_inflation_m': inflation,
            'incremental_frontier_equals_full': incremental_exact,
            'accepted_candidates': accepted_candidate_count,
            'unsafe_accepted_candidates': unsafe_candidate_count,
            'max_active_frontiers': max_active_frontiers,
            'tracks_created': max_tracks_created,
            'source_evidence': sources,
            'uncertainty': uncertainty,
            'replay_digest': hashlib.sha256(repr(replay_rows).encode()).hexdigest(),
        }
        summary['pass'] = bool(
            incremental_exact
            and unsafe_candidate_count == 0
            and uncertainty['finite_entropy']
            and 0.0 <= uncertainty['entropy_min'] <= uncertainty['entropy_max'] <= 1.0 + 1e-12
            and sources['accepted'] == sources['records']
            and sources['rejected'] == 0
            and max_tracks_created < 500
        )
        return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('mat_file', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    first = replay_once(args.mat_file)
    second = replay_once(args.mat_file)
    first['deterministic_repeat_match'] = (
        first['replay_digest'] == second['replay_digest']
        and first['uncertainty']['digest'] == second['uncertainty']['digest']
    )
    first['pass'] = bool(first['pass'] and first['deterministic_repeat_match'])
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / 'S2_4_AD_RECORDED_SHADOW_REPLAY.json').write_text(json.dumps(first, indent=2))
    lines = [
        'S2.4 A-D recorded S2.3 shadow replay',
        f"scenario: {first['scenario']}",
        f"snapshots: {first['snapshots']}",
        f"incremental frontier == full: {int(first['incremental_frontier_equals_full'])}",
        f"deterministic repeat: {int(first['deterministic_repeat_match'])}",
        f"unsafe accepted candidates: {first['unsafe_accepted_candidates']}",
        f"tracks created: {first['tracks_created']}",
        f"replay digest: {first['replay_digest']}",
        f"uncertainty digest: {first['uncertainty']['digest']}",
        f"RESULT: {'PASS' if first['pass'] else 'FAIL'}",
    ]
    (args.output_dir / 'S2_4_AD_RECORDED_SHADOW_REPLAY.txt').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    return 0 if first['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

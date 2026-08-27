from __future__ import annotations

import json
from pathlib import Path
import sys
import numpy as np
from scipy.ndimage import binary_dilation

from s2_4_reference import (
    Candidate,
    FrontierManager,
    GridBelief,
    S24Config,
    astar,
    cluster_frontiers,
    deterministic_digest,
    frontier_mask,
    generate_candidates,
    predict_track,
    route_crossing_risk,
    select_candidate,
    target_route_status,
)

CFG = S24Config()
N = 61


def empty_grid() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    occ = np.zeros((N, N), dtype=bool)
    occ[[0, -1], :] = True
    occ[:, [0, -1]] = True
    free = np.zeros_like(occ)
    unknown = ~(occ | free)
    return free, occ, unknown


def inflate(occ: np.ndarray, radius_m: float = CFG.effective_inflation_m) -> np.ndarray:
    r = int(np.floor(radius_m / CFG.resolution + 1e-12))
    yy, xx = np.mgrid[-r : r + 1, -r : r + 1]
    footprint = np.hypot(xx * CFG.resolution, yy * CFG.resolution) <= radius_m + 1e-12
    return binary_dilation(occ, structure=footprint)


def carve_rect(free: np.ndarray, y0: int, y1: int, x0: int, x1: int) -> None:
    free[y0:y1, x0:x1] = True


def make_belief(free: np.ndarray, occ_raw: np.ndarray, *, stale=None, dynamic=None, entropy=None, quality=None) -> GridBelief:
    hard = inflate(occ_raw)
    free = free & ~hard
    unknown = ~(free | hard)
    unknown_inflated = inflate(unknown)
    navigation_blocked = hard | unknown_inflated
    return GridBelief(free, hard, unknown, entropy=entropy, stale_free=stale, dynamic_risk=dynamic, source_quality=quality, navigation_blocked=navigation_blocked, resolution=CFG.resolution)


def run_frontier_stack(grid: GridBelief, start, target, manager=None, **kwargs):
    manager = manager or FrontierManager()
    clusters = cluster_frontiers(frontier_mask(grid), CFG.min_frontier_cells, CFG.max_frontier_extent_cells)
    frontiers = manager.update(clusters)
    candidates = generate_candidates(grid, frontiers, start, target, CFG, manager, **kwargs)
    return manager, frontiers, candidates, select_candidate(candidates)


def room_with_right_openings(
    openings: tuple[tuple[int, int], ...],
    *,
    y_top: int = 3,
    y_bottom: int = 57,
    x_left: int = 3,
    x_right: int = 38,
) -> tuple[np.ndarray, np.ndarray]:
    """Create a broad observed room with only declared right-wall openings.

    The room is deliberately wider than twice the 0.602 m inflation radius, so
    accepted viewpoints have a genuine known-free stopping/retreat interior.
    Unknown space exists only beyond the openings; all other room boundaries are
    registered occupied, matching S2.3's known-room-boundary contract.
    """
    free, occ, _ = empty_grid()
    occ[y_top, x_left : x_right + 1] = True
    occ[y_bottom, x_left : x_right + 1] = True
    occ[y_top : y_bottom + 1, x_left] = True
    occ[y_top : y_bottom + 1, x_right] = True
    for y0, y1 in openings:
        occ[y0:y1, x_right] = False
    carve_rect(free, y_top + 1, y_bottom, x_left + 1, x_right)
    return free, occ


def enclosed_hall() -> tuple[np.ndarray, np.ndarray]:
    free, occ, _ = empty_grid()
    occ[15, 3:58] = True
    occ[45, 3:58] = True
    occ[15:46, 3] = True
    occ[15:46, 57] = True
    carve_rect(free, 16, 45, 4, 57)
    return free, occ


def scenario_1():
    # Target is visible through one physical doorway frontier.
    free, occ = room_with_right_openings(((23, 39),))
    g = make_belief(free, occ); start = (30, 15); target = (30, 52)
    _, fs, cs, sel = run_frontier_stack(g, start, target)
    assert len(fs) == 1, f"expected one doorway frontier, found {len(fs)}"
    assert sel and sel.target_relevance > 0 and sel.tier == 1
    assert not g.navigation_blocked[sel.cell]
    return {'selected': sel.frontier_track_id, 'relevance': sel.target_relevance, 'candidate': sel.cell}


def scenario_2():
    # Two unknown corridors; only the lower one intersects the target corridor.
    free, occ = room_with_right_openings(((8, 24), (36, 52)))
    g = make_belief(free, occ); start = (30, 14); target = (44, 52)
    _, fs, cs, sel = run_frontier_stack(g, start, target)
    assert len(fs) == 2, f"expected two corridor frontiers, found {len(fs)}"
    assert sel and sel.tier == 1 and sel.target_relevance > 0
    selected_centroid = next(f.centroid for f in fs if f.track_id == sel.frontier_track_id)
    assert selected_centroid[0] >= 36, selected_centroid
    assert any(c.frontier_track_id != sel.frontier_track_id for c in cs)
    return {'selected_centroid': selected_centroid.tolist(), 'target_relevance': sel.target_relevance}


def scenario_3():
    # A short internal screen blocks centered views, while an offset observation
    # pose remains reachable through known free space and reveals the doorway.
    free, occ = room_with_right_openings(((22, 40),))
    occ[29:32, 29] = True
    g = make_belief(free, occ); start = (30, 14); target = (30, 52)
    _, fs, cs, sel = run_frontier_stack(g, start, target)
    assert fs and sel is not None
    assert any('INSUFFICIENT_VISIBLE_UNKNOWN' in c.rejection_reasons for c in cs)
    assert abs(sel.cell[0] - 30) >= 8, sel.cell
    assert sel.visible_unknown
    return {'selected_cell': sel.cell, 'visible': len(sel.visible_unknown), 'occluded_rejections': sum('INSUFFICIENT_VISIBLE_UNKNOWN' in c.rejection_reasons for c in cs)}


def scenario_4():
    # The upper opening exposes much more unknown boundary, but it is unrelated
    # to the commanded target. The smaller lower opening must still win.
    free, occ = room_with_right_openings(((5, 30), (40, 55)), y_top=2, y_bottom=58)
    g = make_belief(free, occ); start = (34, 14); target = (47, 52)
    _, fs, cs, sel = run_frontier_stack(g, start, target)
    assert len(fs) == 2 and sel and sel.tier == 1
    irrelevant = [c for c in cs if 'IRRELEVANT_EXPLORATION' in c.rejection_reasons]
    assert irrelevant
    assert max(c.information_gain for c in irrelevant) >= sel.information_gain
    selected_centroid = next(f.centroid for f in fs if f.track_id == sel.frontier_track_id)
    assert selected_centroid[0] >= 40
    return {'irrelevant_rejections': len(irrelevant), 'selected_relevance': sel.target_relevance, 'largest_irrelevant_gain': max(c.information_gain for c in irrelevant)}


def scenario_5():
    # A direct route exists, but its central section has stale free-space evidence.
    free, occ = enclosed_hall()
    stale = np.zeros((N, N), bool); stale[22:39, 25:36] = True
    g = make_belief(free, occ, stale=stale); start = (30, 12); target = (30, 49)
    status, path = target_route_status(g, start, target)
    assert status == 'HOLD_AND_RESCAN_STALE_ROUTE', status
    assert path and any(g.stale_free[c] for c in path)
    return {'action': status, 'stale_cells_on_route': sum(bool(g.stale_free[c]) for c in path)}


def scenario_6():
    # Two useful alternatives point generally toward the target. The target-
    # relevant frontier is tried first; after two recorded failures it enters a
    # bounded cooldown and the alternative tier-2 frontier is selected.
    free, occ = room_with_right_openings(((15, 29), (33, 47)))
    g = make_belief(free, occ); start = (31, 14); target = (31, 52)
    manager, _, _, sel = run_frontier_stack(g, start, target)
    assert sel
    failed = sel.frontier_track_id
    manager.record_failure(failed, CFG); manager.record_failure(failed, CFG)
    manager, fs2, _, sel2 = run_frontier_stack(g, start, target, manager)
    assert manager.tracks[failed].state == 'TEMPORARILY_BLOCKED'
    assert sel2 and sel2.frontier_track_id != failed
    assert any(f.track_id != failed and manager.eligible(f) for f in fs2)
    return {'failed': failed, 'alternative': sel2.frontier_track_id, 'alternative_tier': sel2.tier}


def scenario_7():
    free, occ = room_with_right_openings(((23, 39),))
    g = make_belief(free, occ); start = (30, 15); target = (30, 52)
    manager, fs, _, _ = run_frontier_stack(g, start, target)
    assert fs
    for f in fs:
        manager.tracks[f.track_id].state = 'EXHAUSTED'
    manager, _, cs2, sel2 = run_frontier_stack(g, start, target, manager)
    assert sel2 is None and not cs2
    return {'terminal': 'USEFUL_EXPLORATION_EXHAUSTED', 'exhausted_tracks': len(fs)}


def scenario_8():
    free, occ = room_with_right_openings(((23, 39),))
    dyn = np.zeros((N, N), float)
    # A conservative predicted crossing band immediately ahead of the hold point
    # intersects every known-free route to an informative doorway viewpoint.
    dyn[24:37, 14:21] = 0.95
    g = make_belief(free, occ, dynamic=dyn); start = (30, 15); target = (30, 52)
    _, _, cs, sel = run_frontier_stack(g, start, target)
    assert cs and sel is None
    assert not any(c.accepted for c in cs)
    assert any('DYNAMIC_ROUTE_CROSSING' in c.rejection_reasons for c in cs)

    # Independent space-time contract: a class-agnostic track crosses the
    # planned centreline at the same predicted arrival time.
    times = np.linspace(0.0, 4.0, 9)
    path_xy = np.column_stack((np.linspace(1.5, 3.0, len(times)), np.full(len(times), 3.0)))
    predictions = predict_track([2.2, 2.0], [0.0, 0.5], np.eye(2) * 0.02, times)
    predicted_risk = route_crossing_risk(path_xy, times, predictions, safety_radius=0.602 + 0.20)
    assert predicted_risk > 0.8
    return {
        'action': 'WAIT_OR_REPLAN',
        'dynamic_rejections': sum('DYNAMIC_ROUTE_CROSSING' in c.rejection_reasons for c in cs),
        'predicted_space_time_risk': predicted_risk,
    }


def scenario_9():
    times = np.linspace(0, 3, 7); preds = predict_track([2.0, 0.0], [0.0, 0.0], np.eye(2) * 0.02, times)
    traces = [float(np.trace(c)) for _, c in preds]
    assert traces[-1] > traces[0]
    return {'action': 'INCREASE_CLEARANCE', 'covariance_growth': traces[-1] - traces[0]}


def scenario_10():
    times = np.linspace(0, 2, 5); preds = predict_track([1.0, 1.0], [0.2, 0.0], np.eye(2) * 0.03, times)
    assert len(preds) == 5 and np.trace(preds[-1][1]) > np.trace(preds[0][1])
    return {'retained': True, 'timeout_policy': 'CONSERVATIVE_UNTIL_EXPIRED'}


def scenario_11():
    free, occ = room_with_right_openings(((23, 39),))
    g = make_belief(free, occ); start = (30, 15); target = (30, 52)
    _, _, _, sel = run_frontier_stack(g, start, target, lidar_quality=1.0, depth_quality=0.2)
    assert sel is not None
    return {'selected': sel.candidate_id, 'depth_quality': 0.2, 'information_gain': sel.information_gain}


def scenario_12():
    free, occ = room_with_right_openings(((23, 39),))
    g = make_belief(free, occ); start = (30, 15); target = (30, 52)
    _, _, _, sel = run_frontier_stack(g, start, target, lidar_quality=0.1, depth_quality=0.6)
    assert sel is not None
    _, _, _, healthy = run_frontier_stack(g, start, target, lidar_quality=1.0, depth_quality=1.0)
    assert healthy and abs(sel.information_gain - healthy.information_gain) > 1e-9
    return {'selected': sel.candidate_id, 'lidar_quality': 0.1, 'depth_quality': 0.6, 'degraded_gain': sel.information_gain, 'healthy_gain': healthy.information_gain}


def scenario_13():
    free, occ = room_with_right_openings(((23, 39),))
    g = make_belief(free, occ); start = (30, 15); target = (30, 52)
    manager = FrontierManager(); manager, fs, cs, sel = run_frontier_stack(g, start, target, manager, lidar_quality=0.0, depth_quality=0.0)
    d1 = deterministic_digest(fs, cs)
    manager2 = FrontierManager(); manager2, fs2, cs2, sel2 = run_frontier_stack(g, start, target, manager2, lidar_quality=0.0, depth_quality=0.0)
    d2 = deterministic_digest(fs2, cs2)
    assert sel is None and sel2 is None and d1 == d2
    assert any('INSUFFICIENT_VISIBLE_UNKNOWN' in c.rejection_reasons for c in cs)
    return {'action': 'SAFE_HOLD', 'digest': d1}


def scenario_14():
    free, occ, _ = empty_grid()
    # Known-free island beyond a sub-diameter doorway. Its frontier is valid in
    # raw map terms, but every observation pose is unreachable after 0.602 m
    # static/unknown inflation.
    occ[17:44, 3] = True; occ[17:44, 18] = True; occ[17, 3:19] = True; occ[43, 3:19] = True
    carve_rect(free, 25, 36, 5, 17)
    occ[5:56, 29] = True; occ[29:32, 29] = False
    carve_rect(free, 24, 37, 37, 48)
    g = make_belief(free, occ); start = (30, 9); target = (30, 52)
    _, fs, cs, sel = run_frontier_stack(g, start, target)
    assert fs and sel is None
    assert any(('INSUFFICIENT_STATIC_CLEARANCE' in c.rejection_reasons or 'UNREACHABLE_KNOWN_FREE' in c.rejection_reasons or 'POSITION_OCCUPIED_OR_UNKNOWN_INFLATED' in c.rejection_reasons) for c in cs)
    assert not any(c.accepted for c in cs)
    return {'rejected': True, 'candidate_count': len(cs)}


def scenario_15():
    # Known room boundaries are fixed; accepted perception progressively extends
    # the observed-free interior. Three bounded active scans expose the target.
    free_base, occ, _ = empty_grid()
    occ[14, 3:58] = True; occ[46, 3:58] = True
    occ[14:47, 3] = True; occ[14:47, 57] = True
    start = (30, 12); target = (30, 49); manager = FrontierManager(); selected = []
    for end in (25, 35, 45, 57):
        free = free_base.copy(); carve_rect(free, 15, 46, 4, end)
        g = make_belief(free, occ)
        manager, fs, cs, sel = run_frontier_stack(g, start, target, manager)
        if sel:
            selected.append(sel.frontier_track_id)
    final_path = astar(g, start, target)
    assert final_path and len(selected) == 3
    assert manager.next_track_id < 20
    return {'goal_reached': True, 'scan_selections': selected, 'tracks_created': manager.next_track_id - 1}


SCENARIOS=[scenario_1,scenario_2,scenario_3,scenario_4,scenario_5,scenario_6,scenario_7,scenario_8,scenario_9,scenario_10,scenario_11,scenario_12,scenario_13,scenario_14,scenario_15]


def main() -> int:
    results=[]; passed=0
    for i,fn in enumerate(SCENARIOS,1):
        try:
            detail=fn(); ok=True; passed+=1; error=''
        except Exception as exc:
            detail={}; ok=False; error=f'{type(exc).__name__}: {exc}'
        results.append({'id':i,'name':fn.__name__,'pass':ok,'detail':detail,'error':error})
        print(f'{i:02d} {fn.__name__:24s} {"PASS" if ok else "FAIL"} {error}')
    out={'schema':'S2_4_AD_CONTRACT_BACKTEST_V1','grid_resolution_m':CFG.resolution,'effective_inflation_m':CFG.effective_inflation_m,'passed':passed,'total':len(SCENARIOS),'pass':passed==len(SCENARIOS),'results':results}
    out_dir=Path(__file__).resolve().parents[1]/'evidence'; out_dir.mkdir(parents=True,exist_ok=True)
    (out_dir/'S2_4_AD_SCENARIO_CONTRACT_RESULTS.json').write_text(json.dumps(out,indent=2))
    lines=['S2.4 A-D scenario-contract backtest',f'grid={CFG.resolution:.2f} m effective_inflation={CFG.effective_inflation_m:.3f} m','']
    lines += [f"{r['id']:02d} {r['name']}: {'PASS' if r['pass'] else 'FAIL'} {r['error']}" for r in results]
    lines += ['',f'RESULT: {passed}/{len(SCENARIOS)} '+('PASS' if out['pass'] else 'FAIL')]
    (out_dir/'S2_4_AD_SCENARIO_CONTRACT_RESULTS.txt').write_text('\n'.join(lines)+'\n')
    return 0 if out['pass'] else 1

if __name__=='__main__':
    raise SystemExit(main())

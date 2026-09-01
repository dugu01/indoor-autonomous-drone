#!/usr/bin/env python3
"""Legacy recorded-map selector regression retained from v0.3.0.

This does not validate the new literal-corridor physical world.
"""
from __future__ import annotations
import json
import math
import pathlib
import sys

import h5py
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python_tests"))

from s2_4_recorded_shadow_replay import read_snapshot, scalar  # noqa: E402
from s2_4_reference import (  # noqa: E402
    FrontierManager,
    S24Config,
    cluster_frontiers,
    frontier_mask,
    generate_candidates,
    incremental_frontier_update,
    select_candidate,
    target_route_status,
)

CFG = S24Config()
MAT = ROOT / "frozen_parent" / "results" / "S2_3_online_mapping" / \
    "v1_0_0_candidate" / "goal_requires_scan" / "seed_000" / \
    "S2_3_v1_0_0_candidate_trial_data.mat"
OUT_JSON = ROOT / "coupled" / "evidence" / \
    "S2_4_E_COMPETING_CORRIDORS_PREFLIGHT.json"
OUT_TXT = ROOT / "coupled" / "evidence" / \
    "S2_4_E_COMPETING_CORRIDORS_PREFLIGHT.txt"


def analyse() -> dict:
    if not MAT.is_file():
        raise FileNotFoundError(MAT)
    with h5py.File(MAT, "r") as file:
        resolution = scalar(file["maps/finalGrid/resolution"])
        inflation = scalar(file["maps/finalGrid/inflationRadius"])
        times = np.asarray(file["log/t"]).reshape(-1)
        estimates = np.asarray(file["log/estP"])
        snapshot_times = np.asarray(file["log/mapSnapshotTimes"]).reshape(-1)
        refs = np.asarray(file["log/mapSnapshots"]).reshape(-1)
        goal_xy = np.asarray(file["scenario/goal"]).reshape(-1)[:2]
        target = (
            int(math.floor(goal_xy[1] / resolution + 0.5)),
            int(math.floor(goal_xy[0] / resolution + 0.5)),
        )

        manager = FrontierManager()
        previous_grid = None
        previous_mask = None
        qualifying: list[dict] = []

        for index, (ref, snapshot_time) in enumerate(zip(refs, snapshot_times)):
            grid, _, version = read_snapshot(file, ref, inflation, resolution)
            nearest = int(np.argmin(np.abs(times - snapshot_time)))
            start_xy = estimates[:2, nearest]
            start = (
                int(math.floor(start_xy[1] / resolution + 0.5)),
                int(math.floor(start_xy[0] / resolution + 0.5)),
            )
            if not grid.inside(start) or not grid.known_free[start]:
                free = np.argwhere(grid.known_free)
                if free.size == 0:
                    previous_grid, previous_mask = grid, frontier_mask(grid)
                    continue
                nearest_free = int(np.argmin(np.sum((free - np.asarray(start)) ** 2, axis=1)))
                start = tuple(int(v) for v in free[nearest_free])

            full = frontier_mask(grid)
            incremental, _ = incremental_frontier_update(
                previous_mask, previous_grid, grid
            )
            if not np.array_equal(full, incremental):
                raise AssertionError("incremental frontier mismatch")
            frontiers = manager.update(
                cluster_frontiers(
                    incremental,
                    CFG.min_frontier_cells,
                    CFG.max_frontier_extent_cells,
                )
            )
            status, _ = target_route_status(grid, start, target)
            if status != "TARGET_ROUTE_AVAILABLE":
                candidates = generate_candidates(
                    grid, frontiers, start, target, CFG, manager
                )
                selected = select_candidate(candidates)
                if selected is not None:
                    target_ids = {
                        c.frontier_track_id for c in candidates
                        if c.target_relevance > 0
                    }
                    irrelevant = [
                        c for c in candidates
                        if c.information_gain > 0 and c.target_relevance <= 0
                    ]
                    irrelevant_ids = {c.frontier_track_id for c in irrelevant}
                    distinct_ids = irrelevant_ids - target_ids
                    distinct = [
                        c for c in irrelevant
                        if c.frontier_track_id in distinct_ids
                    ]
                    if (
                        selected.tier == 1
                        and selected.target_relevance > 0
                        and distinct
                    ):
                        best_decoy = max(distinct, key=lambda c: (
                            c.information_gain, c.utility,
                            -c.frontier_track_id, -c.candidate_id
                        ))
                        qualifying.append({
                            "snapshot_index": index,
                            "snapshot_time_s": float(snapshot_time),
                            "map_version": int(version),
                            "active_frontiers": len(frontiers),
                            "candidate_count": len(candidates),
                            "target_frontier_count": len(target_ids),
                            "distinct_irrelevant_frontier_count": len(distinct_ids),
                            "selected_frontier_track_id": int(selected.frontier_track_id),
                            "selected_candidate_id": int(selected.candidate_id),
                            "selected_tier": int(selected.tier),
                            "selected_target_relevance": float(selected.target_relevance),
                            "selected_information_gain": float(selected.information_gain),
                            "best_decoy_frontier_track_id": int(best_decoy.frontier_track_id),
                            "best_decoy_information_gain": float(best_decoy.information_gain),
                            "larger_decoy_information_gain": bool(
                                best_decoy.information_gain >= selected.information_gain
                            ),
                        })

            previous_grid, previous_mask = grid, incremental

    result = {
        "schema": "S2_4_E_COMPETING_CORRIDORS_PREFLIGHT_V1",
        "source_mat": str(MAT.relative_to(ROOT)),
        "map_resolution_m": resolution,
        "effective_inflation_m": inflation,
        "qualifying_snapshot_count": len(qualifying),
        "first_qualifying_snapshot": qualifying[0] if qualifying else None,
        "scope": "legacy_goal_requires_scan_selector_regression_not_literal_world",
        "pass": bool(
            qualifying
            and qualifying[0]["active_frontiers"] >= 2
            and qualifying[0]["distinct_irrelevant_frontier_count"] >= 1
            and qualifying[0]["selected_tier"] == 1
            and qualifying[0]["selected_target_relevance"] > 0
            and qualifying[0]["larger_decoy_information_gain"]
        ),
    }
    return result


def main() -> int:
    result = analyse()
    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUT_JSON.write_text(json.dumps(result, indent=2) + "\n")
    first = result["first_qualifying_snapshot"] or {}
    lines = [
        "S2.4-E legacy v0.3.0 recorded-map selector regression",
        "scope: NOT the literal-corridor physical-world validation",
        f"qualifying snapshots: {result['qualifying_snapshot_count']}",
        f"first snapshot/time: {first.get('snapshot_index')} / {first.get('snapshot_time_s')}",
        f"active frontiers: {first.get('active_frontiers')}",
        f"distinct irrelevant frontiers: {first.get('distinct_irrelevant_frontier_count')}",
        f"selected tier/relevance: {first.get('selected_tier')} / {first.get('selected_target_relevance')}",
        f"selected/decoy information: {first.get('selected_information_gain')} / {first.get('best_decoy_information_gain')}",
        f"RESULT: {'PASS' if result['pass'] else 'FAIL'}",
    ]
    OUT_TXT.write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

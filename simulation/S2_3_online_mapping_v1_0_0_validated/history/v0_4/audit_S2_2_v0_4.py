#!/usr/bin/env python3
"""Static package audit for Stage S2.2 v0.4.

This audit checks packaging and interface consistency. It does not replace
MATLAB runtime validation.
"""
from __future__ import annotations
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent
REQUIRED = [
    'run_S2_2_mission_replanning.m','validate_S2_2.m','mission_manager_S2_2.m',
    'init_S2_2_config.m','scenario_S2_2.m','plot_S2_2_dashboard.m',
    'animate_S2_2_flight.m','multi_lane_eskf_S2_2.m',
    'geometric_controller_S2_2.m','quadrotor_dynamics_S2_2.m',
    'simulate_sensor_packet_S2_2.m','uncertainty_inflation_S2_2.m',
    'dstar_lite_S2_2.m','astar_grid_S2_2.m',
    'generate_min_snap_trajectory_S2_2.m','segment_occupied_grid_S2_2.m',
]
SCENARIOS = [
    'nominal_6dof','incremental_static_estimated','dynamic_crossing_6dof',
    'dynamic_blocker_becomes_static_6dof','obstacle_sensor_dropout_recover_6dof',
    'primary_imu_fault_vio_outage','xy_aid_loss_failsafe',
]

def main() -> None:
    errors: list[str] = []
    for name in REQUIRED:
        if not (ROOT / name).is_file():
            errors.append(f'missing required file: {name}')

    run = (ROOT / 'run_S2_2_mission_replanning.m').read_text()
    validate = (ROOT / 'validate_S2_2.m').read_text()
    plot = (ROOT / 'plot_S2_2_dashboard.m').read_text()
    manager = (ROOT / 'mission_manager_S2_2.m').read_text()
    config = (ROOT / 'init_S2_2_config.m').read_text()

    if not re.search(r'^function\s+results\s*=\s*run_S2_2_mission_replanning\s*\(seed,scenarioName,makePlots,makeAnimation\)', run, re.M):
        errors.append('main four-argument interface changed')
    if "cfg.version='v0.4'" not in config:
        errors.append('configuration version is not v0.4')
    if "'v0_4'" in run:
        errors.append('hard-coded version folder found; expected cfg.version-derived folder')
    for scenario in SCENARIOS:
        if scenario not in validate:
            errors.append(f'validator missing scenario: {scenario}')
    for stale in ('log.p(', 'log.v,', 'log.a,', 'trackingValidationError'):
        if stale in plot:
            errors.append(f'plot contains stale v0.3 field: {stale}')
    for required in ('log.truthP','log.estP','log.pRef','log.laneScores','log.inflationRadius'):
        if required not in plot:
            errors.append(f'plot missing v0.4 field: {required}')
    for required in ('maxEstimatorPositionError_m','maxEstimatorAttitudeError_deg','laneSwitches','maxInflationRadius_m'):
        if required not in manager:
            errors.append(f'summary missing integration metric: {required}')

    all_text='\n'.join(p.read_text(errors='ignore') for p in ROOT.glob('*'))
    if '/Users/' in all_text or 'C:\\Users\\' in all_text:
        # README protocol intentionally contains the user command path.
        sources='\n'.join(p.read_text(errors='ignore') for p in ROOT.glob('*.m'))
        if '/Users/' in sources or 'C:\\Users\\' in sources:
            errors.append('absolute local path embedded in source code')

    if errors:
        print('STATIC AUDIT: FAIL')
        for e in errors:
            print(f' - {e}')
        raise SystemExit(1)
    print(f'STATIC AUDIT: PASS ({len(REQUIRED)} required files, {len(SCENARIOS)} scenarios)')
    print('NOTE: MATLAB runtime validation is still required.')

if __name__ == '__main__':
    main()

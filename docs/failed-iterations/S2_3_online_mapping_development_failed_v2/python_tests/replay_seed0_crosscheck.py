#!/usr/bin/env python3
"""Cross-check the recorded UNKNOWN_ROOM_NOMINAL seed-0 MATLAB trace.

This script diagnoses the supplied failing run and verifies that the cumulative
candidate source contains the corresponding mechanism corrections. It does not
execute MATLAB and cannot establish coupled 6-DOF acceptance.
"""
from __future__ import annotations
import argparse, json
from pathlib import Path
import h5py
import numpy as np


def scalar(group, name):
    return np.asarray(group[name])[()].reshape(-1)[0].item()


def main() -> int:
    ap=argparse.ArgumentParser()
    ap.add_argument('mat_file', type=Path)
    ap.add_argument('--source-root', type=Path, default=Path(__file__).resolve().parents[1])
    ap.add_argument('--output', type=Path)
    args=ap.parse_args()
    with h5py.File(args.mat_file,'r') as f:
        t=np.asarray(f['log/t']).reshape(-1)
        state=np.asarray(f['log/stateId']).reshape(-1)
        truth_p=np.asarray(f['log/truthP']).T
        dt=float(np.median(np.diff(t)))
        idx_un=np.flatnonzero(state==23)
        idx_land=np.flatnonzero(state==12)
        un_t=float(t[idx_un[0]]) if idx_un.size else float('nan')
        before=t<=un_t if idx_un.size else np.ones_like(t,dtype=bool)
        p0=truth_p[0,:2]
        max_disp=float(np.max(np.linalg.norm(truth_p[before,:2]-p0,axis=1)))
        summary=f['summary']; cfg=f['cfg']
        rec={
            'goal_unreachable_time_s':un_t,
            'map_extension_count_reported':int(round(scalar(summary,'mapExtensionCount'))),
            'max_displacement_before_unreachable_m':max_disp,
            'track_outbound_time_before_unreachable_s':float(np.count_nonzero((state==7)&before)*dt),
            'scan_hold_time_before_unreachable_s':float(np.count_nonzero((state==21)&before)*dt),
            'replan_count':int(round(scalar(summary,'replanCount'))),
            'replan_brake_count':int(round(scalar(summary,'replanBrakeCount'))),
            'trajectory_generation_count':int(round(scalar(summary,'trajectoryGenerationCount'))),
            'landing_descent_start_s':float(t[idx_land[0]]) if idx_land.size else float('nan'),
            'landing_duration_s':float(scalar(cfg,'landingDuration_s')),
            'simulation_horizon_s':float(scalar(cfg,'maxLifecycleTime_s')),
            'map_false_free_rate':float(scalar(summary,'mapFalseFreeRate')),
            'map_occupied_recall':float(scalar(summary,'mapOccupiedRecall')),
            'promotion_count':int(round(scalar(summary,'mapPromotionCount'))),
            'max_reference_speed_mps':float(scalar(summary,'maxReferenceSpeed_mps')),
            'max_estimator_attitude_error_deg':float(scalar(summary,'maxEstimatorAttitudeError_deg')),
        }
    root=args.source_root
    mgr=(root/'mission_lifecycle_manager_S2_3.m').read_text(errors='ignore')
    mapper=(root/'update_probabilistic_map_S2_3.m').read_text(errors='ignore')
    cfg_text=(root/'init_S2_3_config.m').read_text(errors='ignore')
    source={
        'route_aware_repair': 'changed_cells_affect_route_S2_3' in mgr and 'if routeAffected' in mgr,
        'extension_plan_completion_split': 'mapExtensionPlanCount=mapExtensionPlanCount+1' in mgr and 'mapExtensionCount=mapExtensionCount+1' in mgr,
        'scan_progress_not_map_version': 'A changing map version alone is not physical progress' in mgr,
        'continuous_scan_yaw': 'scanStartYaw+cfg.mapScanYawRate_radps' in mgr,
        'endpoint_voxel_excluded': 'if hitIndex>0&&idx==hitIndex,continue;end' in mapper,
        'unique_static_promotion': 'map.promotedStatic' in mapper and 'loPromote' in mapper,
        'strict_trajectory_adapter': 'generate_strict_trajectory_S2_3' in mgr,
        'scan_rate_35_deg_s': 'cfg.mapScanYawRate_radps=deg2rad(35)' in cfg_text,
        'horizon_not_increased_after_failure': 'cfg.maxLifecycleTime_s=220.0' in cfg_text,
    }
    diagnosis={
        'false_extension_exhaustion_supported': rec['map_extension_count_reported']==12 and rec['max_displacement_before_unreachable_m']<0.10,
        'replan_storm_supported': rec['replan_count']>1000,
        'landing_horizon_exhaustion_supported': rec['landing_descent_start_s']+rec['landing_duration_s']>rec['simulation_horizon_s'],
        'map_quality_failure_supported': rec['map_false_free_rate']>0.005 and rec['map_occupied_recall']<0.95,
        'hard_speed_contract_mismatch_supported': rec['max_reference_speed_mps']>0.32,
        'scan_attitude_issue_supported': rec['max_estimator_attitude_error_deg']>2.0,
    }
    report={'mat_file':str(args.mat_file),'recorded_trace':rec,'diagnosis_checks':diagnosis,
            'candidate_source_corrections':source,
            'pass':all(diagnosis.values()) and all(source.values()),
            'note':'PASS means the failing trace is reproduced and candidate mechanisms are present; it is not a MATLAB validation result.'}
    text=json.dumps(report,indent=2)
    print(text)
    if args.output:
        args.output.write_text(text+'\n')
    return 0 if report['pass'] else 1

if __name__=='__main__':
    raise SystemExit(main())

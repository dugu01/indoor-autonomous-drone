#!/usr/bin/env python3
from __future__ import annotations
from pathlib import Path
import hashlib,re,sys,json
root=Path(__file__).resolve().parent
parent=root.parent/'S2_2_src'/'S2_2_mission_replanning_v1_0_0_validated'
checks={}

# Frozen inherited source files copied into the development folder must match.
changed=[];missing=[]
if parent.exists():
    for p in parent.rglob('*'):
        if not p.is_file() or p.name=='SHA256SUMS.txt': continue
        rel=p.relative_to(parent); q=root/rel
        if not q.exists(): missing.append(str(rel)); continue
        if hashlib.sha256(p.read_bytes()).digest()!=hashlib.sha256(q.read_bytes()).digest(): changed.append(str(rel))
if parent.exists():
    checks['inherited_files_present']=not missing
    checks['inherited_files_unchanged']=not changed
else:
    immutability_file=root/'INHERITED_S2_2_IMMUTABILITY_CHECK.json'
    if immutability_file.exists():
        immutable=json.loads(immutability_file.read_text())
        checks['inherited_files_present']=bool(immutable.get('pass')) and immutable.get('checked_file_count',0)>0 and not immutable.get('missing')
        checks['inherited_files_unchanged']=bool(immutable.get('pass')) and immutable.get('checked_file_count',0)>0 and not immutable.get('changed')
    else:
        checks['inherited_files_present']=False
        checks['inherited_files_unchanged']=False

mfiles=list(root.glob('*.m'))
all_text='\n'.join(p.read_text(errors='ignore') for p in mfiles)
# Public S2.3 entry and separation.
checks['s23_entry_point']=(root/'run_S2_3_online_mapping.m').exists()
checks['s23_lifecycle_separate']=(root/'mission_lifecycle_manager_S2_3.m').exists()
checks['frozen_s22_entry_retained']=(root/'run_S2_2_mission_replanning.m').exists()
checks['tabbed_single_dashboard']='uitabgroup(fig)' in (root/'plot_S2_3_dashboard.m').read_text()
checks['raw_lidar_stream']='lidarRanges' in (root/'simulate_perception_packet_S2_3.m').read_text()
checks['raw_depth_stream']='depthRanges' in (root/'simulate_perception_packet_S2_3.m').read_text()
checks['unknown_fail_closed']='mapUnknownIsOccupied=true' in (root/'init_S2_3_config.m').read_text()
checks['map_version_contract']='trajectory.mapVersion' in all_text or "'mapVersion'" in all_text
checks['timestamp_pose_interpolation']='interpolate_pose_buffer_S2_3' in (root/'mission_lifecycle_manager_S2_3.m').read_text()
checks['duplicate_packet_rejection']='duplicate_packet' in (root/'update_probabilistic_map_S2_3.m').read_text()
checks['free_changes_increment_version']='oldFreeClass' in (root/'update_probabilistic_map_S2_3.m').read_text()
checks['safe_stop_gate']='validate_known_free_stop_S2_3' in (root/'plan_unknown_segment_S2_3.m').read_text()
checks['no_route_scan_hold']="state='SCAN_HOLD'" in (root/'mission_lifecycle_manager_S2_3.m').read_text()
checks['truth_audit_script']=(root/'audit_truth_isolation_S2_3.py').exists()
checks['mechanism_tests']=(root/'python_tests'/'test_s23_mechanisms.py').exists()
preflight_text=(root/'preflight_check_S2_3.m').read_text()
update_text=(root/'update_probabilistic_map_S2_3.m').read_text()
validator_text=(root/'validate_map_against_truth_S2_3.m').read_text()
mgr=(root/'mission_lifecycle_manager_S2_3.m').read_text()
checks['preflight_uses_map_freshness']='lastPerceptionTime' in preflight_text and 'mapPreflightMinAcceptedPackets' in preflight_text
checks['idle_packets_not_rejected']='no_sensor_event' in update_text and 'noDataPackets' in update_text
checks['ground_layer_validation_contract']='Floor contact is validated' in validator_text

checks['route_aware_map_repair']='changed_cells_affect_route_S2_3' in mgr and 'if routeAffected' in mgr
checks['extension_plan_completion_split']='mapExtensionPlanCount=mapExtensionPlanCount+1' in mgr and 'mapExtensionCount=mapExtensionCount+1' in mgr
checks['map_version_not_progress_proxy']='A changing map version alone is not physical progress' in mgr
checks['continuous_scan_yaw']='scanStartYaw+cfg.mapScanYawRate_radps' in mgr
checks['strict_trajectory_adapter']=(root/'generate_strict_trajectory_S2_3.m').exists() and 'generate_strict_trajectory_S2_3' in mgr
checks['endpoint_voxel_exclusion']='if hitIndex>0&&idx==hitIndex,continue;end' in update_text
checks['height_aware_dynamic_layer']='zeros(ny,nx,nz' in (root/'init_probabilistic_map_S2_3.m').read_text()
checks['unique_static_promotion']='promotedStatic' in update_text and 'loPromote' in update_text
checks['recorded_trace_crosscheck']=(root/'RECORDED_TRACE_CROSSCHECK_S2_3_SEED0.json').exists()
checks['metric_radius_inflation']='inflate_binary_metric' in (root/'project_map_to_planner_S2_3.m').read_text() and \
    'ceil(inflationRadius/map.resolutionXY)' not in (root/'project_map_to_planner_S2_3.m').read_text()
checks['persistent_static_latch']='map.staticOccupied=false' in (root/'init_probabilistic_map_S2_3.m').read_text() and \
    'if ~map.staticOccupied(idx)' in update_text and 'map.staticOccupied(idx)=true' in update_text
checks['latest_recorded_trace_crosscheck']=(root/'RECORDED_TRACE_CROSSCHECK_S2_3_SEED0_THIRD_RUN.json').exists()

# New S2.3 call references have a matching file or local definition.
defs=set()
for p in mfiles:
    s=p.read_text(errors='ignore')
    defs |= set(re.findall(r'^\s*function(?:\s+\[[^\]]*\]|\s+\w+)?\s*=\s*([A-Za-z]\w*)|^\s*function\s+([A-Za-z]\w*)',s,re.M))
flatdefs={x for tup in defs for x in (tup if isinstance(tup,tuple) else [tup]) if x}
files={p.stem for p in mfiles}
calls=set(re.findall(r'\b([A-Za-z]\w*_S2_3)\s*\(',all_text))
missing_calls=sorted(calls-(files|flatdefs))
checks['all_s23_calls_resolved']=not missing_calls

# Duplicate literal summary fields (most likely to cause MATLAB struct errors).
start=mgr.find('summary=struct('); end=mgr.find('\n\nlog=struct',start)
fields=re.findall(r"'([A-Za-z]\w*)'\s*,",mgr[start:end]) if start>=0 and end>start else []
dups=sorted({f for f in fields if fields.count(f)>1})
checks['summary_fields_unique']=not dups

report={'checks':checks,'pass':all(checks.values()),'missing_inherited':missing,'changed_inherited':changed,
        'missing_s23_calls':missing_calls,'duplicate_summary_fields':dups,
        'matlab_file_count':len(mfiles)}
print('S2.3 cumulative candidate static audit')
for k,v in checks.items(): print(f'{k:38s}: {"PASS" if v else "FAIL"}')
if missing: print('Missing inherited:',missing)
if changed: print('Changed inherited:',changed)
if missing_calls: print('Missing S2.3 calls:',missing_calls)
if dups: print('Duplicate summary fields:',dups)
(root/'STATIC_AUDIT_S2_3_CANDIDATE.json').write_text(json.dumps(report,indent=2))
print('S2.3 STATIC AUDIT:', 'PASS' if report['pass'] else 'FAIL')
sys.exit(0 if report['pass'] else 1)

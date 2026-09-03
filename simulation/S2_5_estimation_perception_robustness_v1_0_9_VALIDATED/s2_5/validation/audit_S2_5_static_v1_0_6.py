#!/usr/bin/env python3
from pathlib import Path
import hashlib,re,subprocess,sys
ROOT=Path(__file__).resolve().parents[2]; S=ROOT/'s2_5'
checks=[]
def ck(n,c,d=''): checks.append((n,bool(c),d))
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()

# Frozen parent must remain byte-identical.
r=subprocess.run([sys.executable,str(S/'validation/audit_S2_4_G_parent_immutability.py')],cwd=ROOT,capture_output=True,text=True)
print(r.stdout,end=''); ck('parent_byte_identity',r.returncode==0)

req=[S/'mission/init_S2_5_config.m',S/'mission/run_S2_5_coupled.m',S/'mission/mission_lifecycle_manager_S2_5.m',
     S/'mission/plan_recovery_viewpoint_S2_5.m',S/'perception/sanitize_perception_packet_S2_5.m',
     S/'validation/backtest_S2_5_v1_0_6_root_cause.py']
ck('required_v106_files',all(p.exists() for p in req))
config=(S/'mission/init_S2_5_config.m').read_text(); manager=(S/'mission/mission_lifecycle_manager_S2_5.m').read_text()
runner=(S/'mission/run_S2_5_coupled.m').read_text(); san=(S/'perception/sanitize_perception_packet_S2_5.m').read_text()
helper=(S/'mission/plan_recovery_viewpoint_S2_5.m').read_text()

ck('candidate_version_v1_0_6','v1.0.6-estimation-perception-robustness-candidate' in config)
ck('inherits_validated_F_config','cfg=init_S2_4_F_config()' in config)
ck('no_estimator_threshold_relaxation',not re.search(r'cfg\.maxEstimator(PositionError_m|AttitudeError_deg)\s*=',config))
ck('no_map_safety_threshold_relaxation',not re.search(r'cfg\.map(MaxFalseFreeRate|MinOccupiedRecall|PoseCovarianceReject_m|MaxPacketAge_s)\s*=',config))
ck('no_progress_scan_threshold_not_changed','cfg.mapMaxNoProgressScans=' not in config)

# Runtime components unrelated to the root-cause overlay remain exact r1 bytes.
exact={
 'mission/plan_recovery_viewpoint_S2_5.m':'2783befe2b2efa6954e0f13598a636f1c25d20642d3e2c73541572fbd17460c9',
 'scenarios/scenario_S2_5.m':'39837a9a1fa4a57d141172171dac0c59f801b7c1dd7c48f1ece232584d3337e0',
 'sensors/simulate_sensor_packet_S2_5.m':'2fc21bea1b228821dc106325f5b46495c8b84dcb496caef6752d4aba1d709d02',
 'perception/simulate_perception_packet_S2_5.m':'a7cad1065a52a08a7a3de84e1157a472f468973b49bb412c54a53db560908b36',
 'validation/run_S2_5_qualification_case.m':'f40c7c7eb5a98ca34125efe56cb675d55c336701b9829cb676ff40a168d287c7',
 'validation/start_S2_5_parallel_pool.m':'82e7003a45e9c3e170e024be260f69c274a1a095e2ba9112e240faf5b43ae20e'}
for rel,h in exact.items(): ck('r1_exact_'+rel.replace('/','_').replace('.','_'),sha(S/rel)==h)
mapper=ROOT/'frozen_parent/S2_3_online_mapping_v1_0_0_validated/update_probabilistic_map_S2_3.m'
ck('frozen_mapper_exact',sha(mapper)=='fc28de39ea7a341ee8a8d0ae199b28a4488ef61fec68ced82d3d4338a90a030e')

# Perception integrity overlay is fault-agnostic and cannot access truth/scenario labels.
code='\n'.join(line.split('%',1)[0] for line in san.splitlines()).lower()
ck('sanitizer_fault_agnostic',all(x not in code for x in ['s25faultname','scenario','truth_world','faultstart','range_spike']))
ck('sanitizer_packet_voxel_dedup',all(x in san for x in ['seen=false','if seen(endpointIndex)','seen(idx)=true']))
ck('sanitizer_static_occlusion_gate','map.staticOccupied(idx)' in san and 'occlusionRejectedHitRayCount' in san)
ck('sanitizer_rejects_not_free_converts','packet.lidarHits(keep)' in san and 'packet.depthHits(keep)' in san)
ck('mapper_uses_sanitized_packet','sanitize_perception_packet_S2_5' in manager and re.search(r'update_probabilistic_map_S2_3\(\s*\.\.\.\s*\n\s*cfg,mapState,mapPacket',manager) is not None)
ck('original_packet_kept_for_fault_semantics','perceptionPacket' in manager and 'mapPacket' in manager)

# Preserve inherited mapping gate; add separate S2.5 recovery-aware accounting.
ck('inherited_scan_gate_exact','scanHoldPass=scanHoldCount>=minScanHolds&&scanHoldCount<=maxScanHolds;' in manager)
ck('recovery_allowance_derived_not_tuned','recoveryScanHoldAllowance=(cfg.mapMaxNoProgressScans+1)*informativeRecoveryRelocationCount;' in manager)
ck('separate_s25_scan_gate','s25RecoveryScanHoldPass=scanHoldCount>=minScanHolds&&' in manager)
ck('inherited_mapping_composite_uses_inherited_scan','mapSafetyReplanPass&&scanHoldPass&&perceptionHoldPass' in manager)
ck('s25_mapping_composite_separate','mapSafetyReplanPass&&s25RecoveryScanHoldPass&&perceptionHoldPass' in manager and "'s25MappingCompositePass',s25MappingCompositePass" in manager)
ck('runner_uses_s25_composite_only_for_s25_acceptance',"field_or_local(s,'s25MappingCompositePass',s.mappingCompositePass)" in runner)
ck('inherited_mapping_composite_still_exposed',"'mappingCompositePass',mappingPass" in manager)

# Existing CSE/SIE and hard safety rules remain present and unchanged in helper.
ck('recovery_cse_sie_still_present',all(x in helper for x in ['continuous_start_egress_local','best_information_anchor_local','validate_known_free_stop_S2_3']))
ck('unknown_still_continuous_hazard','logical(g.staticOccupiedRaw)|logical(g.dynamicOccupiedRaw)|logical(g.unknown)' in helper)
ck('no_global_occupancy_clear','grid.occ(start(1),start(2))=false;' not in helper)

for n,ok,d in checks: print(f'{n:66s} {"PASS" if ok else "FAIL"} {d}')
passed=all(ok for _,ok,_ in checks)
print('S2.5 v1.0.6 STATIC / ISOLATION AUDIT:', 'PASS' if passed else 'FAIL')
sys.exit(0 if passed else 1)

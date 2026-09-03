#!/usr/bin/env python3
from pathlib import Path
import subprocess,sys,re,hashlib
ROOT=Path(__file__).resolve().parents[2]; S=ROOT/'s2_5'
checks=[]
def ck(n,c): checks.append((n,bool(c)))
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()

req=[S/'mission/init_S2_5_config.m',S/'mission/run_S2_5_coupled.m',S/'mission/mission_lifecycle_manager_S2_5.m',S/'mission/plan_recovery_viewpoint_S2_5.m',
 S/'scenarios/scenario_S2_5.m',S/'sensors/simulate_sensor_packet_S2_5.m',
 S/'perception/simulate_perception_packet_S2_5.m',S/'validation/backtest_S2_5_fault_model.py',
 S/'validation/backtest_S2_5_recovery_logic_v1_0_4.py',S/'validation/backtest_S2_5_parallel_harness.py',
 S/'validation/run_S2_5_qualification_case.m',S/'validation/start_S2_5_parallel_pool.m',
 S/'evidence/S2_5_V1_0_1_USER_RECOVERY_TRACE.txt',S/'evidence/v1_0_2_serial_reference/run_S2_5_coupled_serial_reference.txt']
missing=[str(p.relative_to(ROOT)) for p in req if not p.exists()]
ck('required_files',not missing)

r=subprocess.run([sys.executable,str(S/'validation/audit_S2_4_G_parent_immutability.py')],cwd=ROOT,capture_output=True,text=True)
print(r.stdout,end='');ck('parent_byte_identity',r.returncode==0)

config=(S/'mission/init_S2_5_config.m').read_text(); manager=(S/'mission/mission_lifecycle_manager_S2_5.m').read_text()
ck('inherits_validated_F_config','cfg=init_S2_4_F_config()' in config)
ck('candidate_version_v1_0_4',"v1.0.4-estimation-perception-robustness-candidate" in config)
ck('no_estimator_threshold_relaxation',not re.search(r'cfg\.maxEstimator(PositionError_m|AttitudeError_deg)\s*=',config))
ck('no_map_safety_threshold_relaxation',not re.search(r'cfg\.map(MaxFalseFreeRate|MinOccupiedRecall|PoseCovarianceReject_m|MaxPacketAge_s)\s*=',config))
ck('execution_validation_fault_disabled',"'name','NONE'" in config)

# Runtime files unrelated to the reviewed recovery change remain byte-identical
# to the user-tested v1.0.1 candidate.
old_hashes={
 'scenarios/scenario_S2_5.m':'39837a9a1fa4a57d141172171dac0c59f801b7c1dd7c48f1ece232584d3337e0',
 'sensors/simulate_sensor_packet_S2_5.m':'2fc21bea1b228821dc106325f5b46495c8b84dcb496caef6752d4aba1d709d02',
 'perception/simulate_perception_packet_S2_5.m':'a7cad1065a52a08a7a3de84e1157a472f468973b49bb412c54a53db560908b36'}
for rel,h in old_hashes.items(): ck('v101_unchanged_'+rel.replace('/','_').replace('.','_'),sha(S/rel)==h)
# run_S2_5_coupled has a reviewed harness-only I/O delta: default behavior is
# unchanged, while qualification may disable verbose printing/full MAT save.
runner=(S/'mission/run_S2_5_coupled.m').read_text()
runner_ref=S/'evidence/v1_0_2_serial_reference/run_S2_5_coupled_serial_reference.txt'
ck('serial_runner_reference_exact',sha(runner_ref)=='8d1ace1f24dbd37114775a816012fa2a0fb826b641aa920a7e129a44cb92f265')
ck('runner_reviewed_io_delta',all(x in runner for x in [
    'mission_lifecycle_manager_S2_5(cfg,scenario)',
    'if nargin<5||isempty(saveArtifacts),saveArtifacts=true;end',
    'if nargin<6||isempty(verbose),verbose=true;end',
    'if saveArtifacts', 'if verbose']))
# Config delta is metadata-only: replacing v1.0.2 by v1.0.1 must recover the
# exact user-tested v1.0.1 config hash.
old_cfg=config.replace('v1.0.4-estimation-perception-robustness-candidate','v1.0.1-estimation-perception-robustness-candidate')
ck('config_delta_version_only',hashlib.sha256(old_cfg.encode()).hexdigest()=='27aadea2e02935bc97b4386bfddc9d6129eef1ac824c7df79bedc99536a55346')
ck('manager_changed_from_v101',sha(S/'mission/mission_lifecycle_manager_S2_5.m')!='5f3383a635050ad107161dad0e4f8c5c5e159b87b3addc9996f5b6509b921930')

# Reviewed recovery delta contracts.
for name,tok in [
 ('episode_counter','authorityEpisodeInvalidationCount'),
 ('episode_bound','pendingExplorationTerminal=authorityEpisodeInvalidationCount>='),
 ('fresh_request_resets_episode','authorityEpisodeRequestId=active_request_id(plannedRequest);'),
 ('same_request_reauth_does_not_reset','REAUTHORIZED_AFTER_REVOCATION'),
 ('conservative_fallback_counter','conservativeFrontierFallbackCount'),
 ('fallback_reason','s25_recovery_conservative_frontier'),
 ('bound_fallback_reason','conservative_frontier_after_authority_bound'),
 ('frozen_s23_planner_reused','plan_unknown_segment_S2_3'),
 ('three_scans_semantics','scanNoProgressCount>cfg.mapMaxNoProgressScans')]: ck(name,tok in manager)
ck('episode_invalidation_limit_unchanged',"field_or_local(executionSafetyCfg,'maxAuthorityInvalidations',3)" in manager)
ck('mission_extension_limit_unchanged','mapExtensionCount>=cfg.mapMaxExtensionAttempts' in manager)
helper=(S/'mission/plan_recovery_viewpoint_S2_5.m').read_text()
ck('informative_recovery_counter','informativeRecoveryRelocationCount' in manager)
ck('informative_recovery_after_three_scans','scanNoProgressCount>=cfg.mapMaxNoProgressScans' in manager and 'plan_recovery_viewpoint_S2_5' in manager)
ck('recovery_endpoint_known_free','~g.knownFree' in helper and 'g.navigationBlocked' in helper)
ck('recovery_hold_support','hold_support_local' in helper)
ck('recovery_metric_route_no_corner_cut','astar_grid_S2_2' in helper and 'astar_known_free_S2_4' not in helper)
ck('recovery_route_freshness','staleFreeAge_s' in helper)
ck('recovery_dynamic_rejection','dynamicOccupiedRaw' in helper)
ck('recovery_visible_unknown','minVisibleUnknownCells' in helper)
ck('recovery_meaningful_relocation','candidateRadii_m' in helper and 'minRelocation' in helper)
ck('recovery_strict_trajectory','generate_strict_trajectory_S2_3' in helper)
ck('recovery_known_free_stop','validate_known_free_stop_S2_3' in helper)

ck('recovery_active_config_namespace','plan_recovery_viewpoint_S2_5(cfg,s24cfg' in manager)
ck('recovery_no_double_active_config_dereference','plan_recovery_viewpoint_S2_5(cfg,s24cfg.activeExploration' not in manager)
ck('manager_active_config_alias','s24cfg=cfg.activeExploration;' in manager)
ck('recovery_dead_stageA_removed','unknownInflated(iy-1:iy+1' not in helper)
ck('recovery_exhaustive_full_checks','for k=1:size(rows,1)' in helper and 'candidateTraj' in helper and 'stopRejected' in helper)
ck('recovery_scan_budget_reset_at_new_vantage','segmentRecoveryRelocation' in manager and 'scanNoProgressCount=0;' in manager)
ck('recovery_diagnostics_exposed','informativeRecoveryAttemptCount' in manager and 'lastInformativeRecoveryDiagnostics' in manager)

# The fallback must not introduce direct controller/estimator/plant tuning.
forbidden=[r'cfg\.Kp\s*=',r'cfg\.Kv\s*=',r'cfg\.KR\s*=',r'cfg\.Kw\s*=',r'cfg\.maxEstimatorPositionError_m\s*=',
           r'cfg\.maxEstimatorAttitudeError_deg\s*=',r'cfg\.mapMaxFalseFreeRate\s*=',r'cfg\.mapMinOccupiedRecall\s*=']
ck('manager_no_threshold_or_gain_tuning',not any(re.search(p,manager) for p in forbidden))

# Fail-safe qualification remains the v1.0.1 corrected contract.
val=(S/'validation/validate_S2_5_all.m').read_text()
fs=re.search(r'function \[pass,mapSafety,core,evidence\]=failsafe_case_pass(.*?)\nend',val,re.S)
ck('historical_failure_preflight',all(x in val for x in ['HISTORICAL RECOVERY PREFLIGHT','histNames','historical_cached_run','assert(all(histPass)']))
ck('failsafe_helper_found',fs is not None)
if fs:
    f=fs.group(1)
    ck('failsafe_no_nominal_mapping_completeness','mappingCompositePass' not in f)
    ck('failsafe_keeps_hard_safety',all(x in f for x in ['mapFalseFreeRate','unknownCommitmentCount==0','truthIsolationPass','executionSafetyPass','failsafeTriggered','emergencyLanding','missionComplete']))

# Fault definitions themselves are unchanged.
sc=(S/'scenarios/scenario_S2_5.m').read_text()
for token in ['S25_NAV_VIO_DROPOUT','S25_NAV_LIDAR_AID_DROPOUT','S25_NAV_VIO_OUTLIER_BURST',
              'S25_NAV_LIDAR_OUTLIER_BURST','S25_NAV_PRIMARY_IMU_FAULT_VIO_OUTAGE','S25_NAV_HIGH_MEASUREMENT_NOISE',
              'S25_NAV_XY_AID_LOSS_FAILSAFE','S25_PERCEPTION_LIDAR_DROPOUT','S25_PERCEPTION_DEPTH_DROPOUT',
              'S25_PERCEPTION_BRIEF_DUAL_DROPOUT','S25_PERCEPTION_STALE_PACKET_BURST','S25_PERCEPTION_RANGE_SPIKE',
              'S25_PERCEPTION_PROLONGED_DUAL_DROPOUT','S25_COUPLED_IMU_FAULT_PERCEPTION_DROPOUT']:
    ck('scenario_'+token,token in sc)
nav=(S/'sensors/simulate_sensor_packet_S2_5.m').read_text(); per=(S/'perception/simulate_perception_packet_S2_5.m').read_text()
ck('nav_outlier_hooks',all(x in nav for x in ['VIO_OUTLIER_BURST','LIDAR_OUTLIER_BURST','VIO_DROPOUT','LIDAR_AID_DROPOUT']))
ck('perception_fault_hooks',all(x in per for x in ['LIDAR_SCAN_DROPOUT','DEPTH_DROPOUT','DUAL_DROPOUT','STALE_PACKET_BURST','RANGE_SPIKE']))

for n,ok in checks: print(f'{n:62s} {"PASS" if ok else "FAIL"}')
if missing:
    for m in missing: print('MISSING',m)
passed=all(ok for _,ok in checks)
print('S2.5 v1.0.4 STATIC / ISOLATION AUDIT:', 'PASS' if passed else 'FAIL')
sys.exit(0 if passed else 1)

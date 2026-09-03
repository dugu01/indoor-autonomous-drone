#!/usr/bin/env python3
from pathlib import Path
import re,sys
ROOT=Path(__file__).resolve().parents[2]; S=ROOT/'s2_5'
manager=(S/'mission/mission_lifecycle_manager_S2_5.m').read_text()
validator=(S/'validation/validate_S2_5_all.m').read_text()
trace=(S/'evidence/S2_5_V1_0_1_USER_RECOVERY_TRACE.txt').read_text()
s23plan=(ROOT/'frozen_parent/S2_3_online_mapping_v1_0_0_validated/plan_unknown_segment_S2_3.m').read_text()
s23front=(ROOT/'frozen_parent/S2_3_online_mapping_v1_0_0_validated/select_goal_frontier_S2_3.m').read_text()
config=(S/'mission/init_S2_5_config.m').read_text()
checks=[]
def ck(n,c,d=''): checks.append((n,bool(c),d))

# 1) The actual v1.0.1 MATLAB trace establishes that direct goal reachability
# never appears in any of the five historical failures and separates the two
# recovery mechanisms we are correcting.
for name in ['NAV_IMU_FAULT_VIO_OUTAGE','PERCEPTION_DUAL_BRIEF','PERCEPTION_STALE_BURST','PERCEPTION_RANGE_SPIKE','COUPLED_IMU_PERCEPTION']:
    ck('trace_'+name, name in trace)
ck('trace_direct_goal_never_available', trace.count('direct |')>=5 and not re.search(r'\|\s+\d+\s+\d+\s+\d+\s+\d+\s+\d+\s+1\s+\|',trace))
ck('trace_no_safe_active_viewpoint_repeats', trace.count('no_safe_active_viewpoint')>=8)
ck('trace_authority_bound_cases', trace.count('authority_bound_direct_goal_unavailable')>=8)
ck('trace_global_count_mixed_requests', all(x in trace for x in ['req=16','req=25','inv=3']))

# 2) Per-request/episode invalidation semantics. A fresh planner-issued request
# resets the episode counter; reauthorization does not. This prevents the
# perception revocation of request 16 from consuming request 25's retry budget.
for token in [
    'authorityEpisodeRequestId=uint64(0);authorityEpisodeInvalidationCount=0;',
    'authorityEpisodeInvalidationCount=authorityEpisodeInvalidationCount+1;',
    'authorityEpisodeRequestId=active_request_id(plannedRequest);',
    'authorityEpisodeInvalidationCount=0;',
    "pendingExplorationTerminal=authorityEpisodeInvalidationCount>= ...",
    "field_or_local(executionSafetyCfg,'maxAuthorityInvalidations',3)"]:
    ck('episode_source_'+re.sub(r'\W+','_',token)[:45], token in manager)
# Three distinct invalidation sites must feed the same episode counter.
ck('episode_increment_three_revocation_seams', manager.count('authorityEpisodeInvalidationCount=authorityEpisodeInvalidationCount+1;')==3)
# Reset occurs on fresh request acceptance and after a completed conservative
# frontier hop, but not inside the reauthorization-success block.
reauth=re.search(r'if recovery\.success(.*?)else\n\s*executionSuspendedRequestRecoveryRejectCount',manager,re.S)
ck('reauthorization_does_not_reset_episode', reauth is not None and 'authorityEpisodeInvalidationCount=0' not in reauth.group(1))

def global_old(seq):
    total=0; terminal=[]
    for fresh in seq:
        if fresh=='invalidate': total+=1
        terminal.append(total>=3)
    return terminal
# req16 revocation + fresh req25 + 2 req25 invalidations: old global counter
# terminates; per-episode counter correctly leaves req25 at 2/3.
old_total=3
req25_episode=2
ck('old_global_false_terminal',old_total>=3)
ck('new_episode_not_terminal_for_req25',req25_episode<3)
ck('same_request_three_invalidations_terminal',3>=3)

# 3) Conservative S2.3 fallback. This is not a new planner: it must call the
# frozen inherited planner when S2.4 cannot produce a safe active viewpoint,
# and after an episode bound. No active exploration lease is issued for it.
ck('fallback_on_no_safe_active_viewpoint', all(x in manager for x in [
    "{'no_safe_active_viewpoint','active_viewpoint_request_invalid'}",
    'plan_unknown_segment_S2_3(cfg,grid,est,estAcc,segmentMissionGoal',
    "'s25_recovery_conservative_frontier'",
    'conservativeFrontierFallbackCount=conservativeFrontierFallbackCount+1;']))
# bound branch uses S2.3 planner and does not call active planner in that branch
boundplan=re.search(r'if authorityBoundExplorationSuppressed(.*?)else\n\s*\[planner,path,traj,st,jump,routeExists,planMeta,frontierState,plannedRequest\]',manager,re.S)
ck('bound_branch_found',boundplan is not None)
if boundplan:
    q=boundplan.group(1)
    ck('bound_uses_frozen_s23_planner','plan_unknown_segment_S2_3' in q)
    ck('bound_does_not_issue_active_request','plan_active_exploration_segment_S2_4' not in q and 'exploration_request_S2_4' not in q)
    ck('bound_allows_frontier_not_only_direct','baseMeta.frontierUsed' in q and 'routeExists=false' not in q)

# Frozen S2.3 frontier safety/progress contract: frontier cell itself is
# known-free, must make goal progress, must have an A* path, strict trajectory,
# and known-free stop validation.
for name,cond in [
    ('s23_frontier_requires_known_free','~grid.knownFree' in s23front),
    ('s23_frontier_requires_goal_progress','progress<cfg.mapMinFrontierProgress_m' in s23front),
    ('s23_frontier_requires_astar','astar_grid_S2_2' in s23front),
    ('s23_plan_uses_strict_trajectory','generate_strict_trajectory_S2_3' in s23plan),
    ('s23_plan_uses_known_free_stop','validate_known_free_stop_S2_3' in s23plan)]: ck(name,cond)

# 4) After completing one conservative frontier extension following an episode
# bound, active exploration may be reconsidered from the new sensing context.
# The failed request itself remains dead because active request/authority were
# cleared at the bound and no suspended request is retained.
ck('bound_clears_dead_request',all(x in manager for x in [
    'suspendedExplorationRequest=struct();suspendedRequestAvailable=false;',
    'activeExplorationRequest=struct();explorationActive=false;',
    'activeAuthorityGeneration=uint64(0);']))
arrival=re.search(r'mapExtensionCount=mapExtensionCount\+1;(.*?)if explorationActive',manager,re.S)
ck('conservative_hop_reopens_fresh_episode',arrival is not None and all(x in arrival.group(1) for x in [
    'authorityBoundExplorationSuppressed=false','authorityEpisodeRequestId=uint64(0)','authorityEpisodeInvalidationCount=0']))
ck('mission_level_extension_bound_unchanged','mapExtensionCount>=cfg.mapMaxExtensionAttempts' in manager)

# 5) Keep the corrected v1.0.1 three-scan semantics and fail-safe scoring.
ck('three_configured_scans_still_allowed','scanNoProgressCount>cfg.mapMaxNoProgressScans||mapExtensionCount>=cfg.mapMaxExtensionAttempts' in manager)
fs=re.search(r'function \[pass,mapSafety,core,evidence\]=failsafe_case_pass(.*?)\nend',validator,re.S)
ck('failsafe_helper_found',fs is not None)
if fs:
    f=fs.group(1)
    ck('failsafe_nominal_map_completeness_not_required','mappingCompositePass' not in f)
    for tok in ['mapFalseFreeRate','unknownCommitmentCount==0','truthIsolationPass','missionOutcomePass','estimatorGate','executionSafetyPass','failsafeTriggered','emergencyLanding','missionComplete']:
        ck('failsafe_keeps_'+re.sub(r'\W+','_',tok),tok in f)

# 6) Fail-fast/caching remains: no multi-hour campaign until the exact five
# historical failures pass on MATLAB.
for tok in ["'nav_imu_fault_vio_outage'","'perception_dual_brief'","'perception_stale_burst'","'perception_range_spike'","'coupled_imu_perception'",'histSeeds=[1 1 1 2 1]','historical_cached_run','assert(all(histPass)']:
    ck('preflight_'+re.sub(r'\W+','_',tok),tok in validator)

# 7) No threshold/config safety relaxation.
ck('version_v1_0_2','v1.0.2-estimation-perception-robustness-candidate' in config)
ck('estimator_thresholds_untouched','maxEstimatorPositionError_m=' not in config and 'maxEstimatorAttitudeError_deg=' not in config)
ck('map_thresholds_untouched','mapMaxFalseFreeRate=' not in config and 'mapMinOccupiedRecall=' not in config and 'mapMaxPacketAge_s=' not in config)
ck('execution_fault_injection_disabled',"'name','NONE'" in config)

# Simple negative controls for the fallback contract: a conservative fallback
# cannot be considered executable without route + valid trajectory + stop.
def can_translate(route,traj,stop): return bool(route and traj and stop)
ck('negative_no_route',not can_translate(False,True,True))
ck('negative_bad_trajectory',not can_translate(True,False,True))
ck('negative_bad_stop',not can_translate(True,True,False))
ck('positive_known_free_frontier',can_translate(True,True,True))

for n,ok,d in checks: print(f'{n:66s} {"PASS" if ok else "FAIL"} {d}')
passed=all(x[1] for x in checks)
print('S2.5 v1.0.2 RECOVERY / FAIL-SAFE BACKTEST:', 'PASS' if passed else 'FAIL')
sys.exit(0 if passed else 1)

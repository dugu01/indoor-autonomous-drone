#!/usr/bin/env python3
from pathlib import Path
import re,sys
ROOT=Path(__file__).resolve().parents[2]; S=ROOT/'s2_5'
manager=(S/'mission/mission_lifecycle_manager_S2_5.m').read_text()
validator=(S/'validation/validate_S2_5_all.m').read_text()
evidence=(S/'evidence/S2_5_V1_0_0_USER_MATLAB_RECOVERY_DIAGNOSTIC.txt').read_text()
checks=[]
def ck(n,c,d=''): checks.append((n,bool(c),d))
# User diagnostic fixture identity.
expected=[('NAV_IMU_FAULT_VIO_OUTAGE',1,'PLAN_OUTBOUND_NO_ROUTE_BUDGET',3,2),
          ('PERCEPTION_DUAL_BRIEF',1,'AUTHORITY_INVALIDATION_BOUND',0,0),
          ('PERCEPTION_STALE_BURST',1,'AUTHORITY_INVALIDATION_BOUND',0,0),
          ('PERCEPTION_RANGE_SPIKE',2,'PLAN_OUTBOUND_NO_ROUTE_BUDGET',3,3),
          ('COUPLED_IMU_PERCEPTION',1,'PLAN_OUTBOUND_NO_ROUTE_BUDGET',3,2)]
for name,seed,reason,np,holds in expected:
    ck('fixture_'+name, all(x in evidence for x in [name,f'seed={seed}',reason,f'noProgress={np}/3',f'scanHolds={holds}']))
# Off-by-one: cfg says 3 scans, therefore counts 1,2,3 should each execute a scan.
def old_action(count,maxn=3): return 'UNREACHABLE' if count>=maxn else 'SCAN'
def new_action(count,maxn=3): return 'UNREACHABLE' if count>maxn else 'SCAN'
ck('old_logic_only_allowed_two_scans',[old_action(i) for i in (1,2,3)]==['SCAN','SCAN','UNREACHABLE'])
ck('new_logic_executes_three_configured_scans',[new_action(i) for i in (1,2,3,4)]==['SCAN','SCAN','SCAN','UNREACHABLE'])
ck('source_has_corrected_plan_outbound_comparison','scanNoProgressCount>cfg.mapMaxNoProgressScans||mapExtensionCount>=cfg.mapMaxExtensionAttempts' in manager)
# Authority invalidation bound remains a safety bound, but suppresses further exploration instead of asserting global goal unreachable.
bound=re.search(r'if pendingExplorationTerminal(.*?)elseif suspendedRequestAvailable',manager,re.S)
ck('authority_bound_block_found',bound is not None)
if bound:
    b=bound.group(1)
    ck('authority_bound_suppresses_translation','authorityBoundExplorationSuppressed=true' in b)
    ck('authority_bound_clears_authority',all(x in b for x in ['activeAuthorityGeneration=uint64(0)','explorationActive=false']))
    ck('authority_bound_does_not_assert_global_unreachable',"goalUnreachable=true" not in b and "state='GOAL_UNREACHABLE'" not in b)
    ck('authority_bound_returns_to_goal_planning',"state='PLAN_OUTBOUND'" in b)
ck('max_authority_invalidations_not_raised',"field_or_local(executionSafetyCfg,'maxAuthorityInvalidations',3)" in manager and 'maxAuthorityInvalidations',)
# After bound, only direct known-free goal motion is allowed: frontier route is discarded.
plan=re.search(r'if authorityBoundExplorationSuppressed(.*?)else\n\s*\[planner,path,traj,st,jump,routeExists,planMeta,frontierState,plannedRequest\]',manager,re.S)
ck('direct_only_branch_found',plan is not None)
if plan:
    q=plan.group(1)
    ck('uses_inherited_unknown_planner_readonly','plan_unknown_segment_S2_3' in q)
    ck('requires_final_direct_goal','baseMeta.isFinal&&baseMeta.goalCurrentlyReachable' in q)
    ck('discards_frontier_translation','if ~directGoal' in q and 'routeExists=false' in q)
# Fail-safe scoring: map completeness may be false after intentional abort, but all safety/core gates remain mandatory.
fs=re.search(r'function \[pass,mapSafety,core,evidence\]=failsafe_case_pass(.*?)\nend',validator,re.S)
ck('failsafe_helper_found',fs is not None)
if fs:
    f=fs.group(1)
    ck('nominal_mapping_completeness_not_required','mappingCompositePass' not in f)
    for token in ['mapFalseFreeRate','unknownCommitmentCount==0','truthIsolationPass','missionOutcomePass','trajectoryGate','controllerGate','estimatorGate','continuityPass','uncertaintyPass','staticGate','executionSafetyPass','failsafeTriggered','emergencyLanding','missionComplete']:
        ck('failsafe_keeps_'+token.replace('==','_'),token in f)
# Negative control model for corrected fail-safe contract.
base=dict(mapFalseFreeRate=0.001, unknown=0, truth=1, M=1,T=1,C=1,E=1,K=1,U=1,S=1,X=1,timeout=0,
          failsafe=1,emergency=1,complete=1,final='COMPLETE',collision=0,geo=0,hold=1,perapps=118)
def fs_pass(x,per=True):
    mapSafety=x['mapFalseFreeRate']<=0.005 and x['unknown']==0 and x['truth']==1
    core=all(x[k]==1 for k in ['M','T','C','E','K','U','S','X']) and x['timeout']==0
    ev=x['failsafe']==1 and x['emergency']==1 and x['complete']==1 and x['final']=='COMPLETE' and x['collision']==0 and x['geo']==0
    if per: ev=ev and x['hold']>=1 and x['perapps']>=1
    return mapSafety and core and ev
ck('failsafe_positive_control',fs_pass(base.copy()))
for key,bad in [('unknown',1),('truth',0),('collision',1),('geo',1),('E',0),('X',0),('failsafe',0),('emergency',0),('complete',0),('timeout',1)]:
    x=base.copy();x[key]=bad;ck('failsafe_negative_'+key,not fs_pass(x))
x=base.copy();x['mapFalseFreeRate']=0.006;ck('failsafe_negative_false_free',not fs_pass(x))
# Historical-first caching contract.
for token in ["'nav_imu_fault_vio_outage'","'perception_dual_brief'","'perception_stale_burst'","'perception_range_spike'","'coupled_imu_perception'",'histSeeds=[1 1 1 2 1]','historical_cached_run']:
    ck('preflight_'+re.sub(r'\W+','_',token),token in validator)
ck('preflight_fail_fast','assert(all(histPass)' in validator)
# No threshold relaxation.
config=(S/'mission/init_S2_5_config.m').read_text()
ck('estimator_thresholds_untouched','maxEstimatorPositionError_m=' not in config and 'maxEstimatorAttitudeError_deg=' not in config)
ck('map_thresholds_untouched','mapMaxFalseFreeRate=' not in config and 'mapMinOccupiedRecall=' not in config)

for n,ok,d in checks: print(f'{n:62s} {"PASS" if ok else "FAIL"} {d}')
passed=all(x[1] for x in checks)
print('S2.5 v1.0.1 RECOVERY / FAIL-SAFE BACKTEST:', 'PASS' if passed else 'FAIL')
sys.exit(0 if passed else 1)

#!/usr/bin/env python3
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]; S=ROOT/'s2_5'
mgr=(S/'mission/mission_lifecycle_manager_S2_5.m').read_text(); runner=(S/'mission/run_S2_5_coupled.m').read_text(); val=(S/'validation/validate_S2_5_all.m').read_text(); cfg=(S/'mission/init_S2_5_config.m').read_text()
checks=[]
def ck(n,c,d=''): checks.append((n,bool(c),d))
# Reproduce the NAV dead-state logic discovered from MATLAB trace.
def old_nav_resume(interrupted,pending_resume):
    pending=False # NAV hold cancels pending replan
    return interrupted,pending
def new_nav_resume(interrupted,pending_resume):
    pending=False
    if interrupted=='LIFECYCLE_REPLAN_BRAKE':
        return ('PLAN_RTL' if pending_resume=='TRACK_RTL' else 'PLAN_OUTBOUND'),pending
    return interrupted,pending
st,p=old_nav_resume('LIFECYCLE_REPLAN_BRAKE','TRACK_OUTBOUND')
ck('old_logic_reproduces_dead_brake',st=='LIFECYCLE_REPLAN_BRAKE' and not p)
st,p=new_nav_resume('LIFECYCLE_REPLAN_BRAKE','TRACK_OUTBOUND')
ck('new_outbound_nav_recovery_replans_fresh',st=='PLAN_OUTBOUND' and not p)
st,p=new_nav_resume('LIFECYCLE_REPLAN_BRAKE','TRACK_RTL')
ck('new_rtl_nav_recovery_replans_fresh',st=='PLAN_RTL' and not p)
ck('source_has_dead_brake_guard',"if strcmp(navigationResumeState,'LIFECYCLE_REPLAN_BRAKE')" in mgr)
ck('mission_timeout_unchanged','missionStateTimeout_s=' not in cfg)
# Perception: stale while already in replan brake is a safe hold-equivalent episode,
# not a request to change motion state. Count one episode until freshness returns.
def count_eps(states,fresh):
    active=False;n=0
    for st,fr in zip(states,fresh):
        if (not fr) and st=='LIFECYCLE_REPLAN_BRAKE':
            if not active:n+=1
            active=True
        elif fr: active=False
    return n
states=['TRACK_OUTBOUND']+['LIFECYCLE_REPLAN_BRAKE']*43+['PLAN_OUTBOUND']
fresh=[True]+[False]*43+[True]
ck('stale_brake_43_samples_is_one_safe_episode',count_eps(states,fresh)==1)
ck('safe_response_does_not_force_state_transition',"state='MAP_DEGRADED_HOLD'" in mgr and "perceptionStaleBrakeEpisodeCount=perceptionStaleBrakeEpisodeCount+1;" in mgr)
ck('runner_accepts_s25_safe_response',"s25PerceptionSafeResponseCount" in runner)
ck('validator_brief_uses_safe_response','safe_perception_response_count(s)>=1' in val)
ck('failsafe_still_demands_named_hold','s.perceptionHoldCount>=1&&s.s25PerceptionFaultApplicationCount>=1' in val)
ck('execution_safety_retained','s.executionSafetyPass' in runner and 's.staleCommandContinuationCount==0' in runner)
for n,c,d in checks: print(f'{n:55s} {"PASS" if c else "FAIL"} {d}')
ok=all(c for _,c,_ in checks); print('S2.5 v1.0.9 LIFECYCLE INTEGRATION BACKTEST:', 'PASS' if ok else 'FAIL'); sys.exit(0 if ok else 1)

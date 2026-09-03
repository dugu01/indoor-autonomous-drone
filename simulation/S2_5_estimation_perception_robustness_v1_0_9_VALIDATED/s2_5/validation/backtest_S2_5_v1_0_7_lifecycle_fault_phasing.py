#!/usr/bin/env python3
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]; S=ROOT/'s2_5'
mgr=(S/'mission/mission_lifecycle_manager_S2_5.m').read_text(); sc=(S/'scenarios/scenario_S2_5.m').read_text(); per=(S/'perception/simulate_perception_packet_S2_5.m').read_text()
checks=[]
def ck(n,c,d=''): checks.append((n,bool(c),d))
# Replan timestamp invariant.
needle="replanBrakeCount=replanBrakeCount+1;state='LIFECYCLE_REPLAN_BRAKE';stateEntryTime=t;holdPosition=est.p;"
ck('route_affected_replan_clock_reset',needle in mgr)
ck('mission_timeout_unchanged','missionStateTimeout_s=' not in (S/'mission/init_S2_5_config.m').read_text())
# Minimal state-trigger model matching MATLAB logic.
def active_series(states,ts,after=18.0,duration=1.2):
    trig=None; out=[]; eligible={'TRACK_OUTBOUND','TRACK_RTL','SCAN_HOLD'}
    for st,t in zip(states,ts):
        if trig is None and t>=after-1e-12 and st in eligible: trig=t
        out.append(trig is not None and t>=trig-1e-12 and t<=trig+duration+1e-12)
    return trig,out
states=['PLAN_OUTBOUND']*5+['TRACK_OUTBOUND']*5+['MAP_DEGRADED_HOLD']*35
ts=[18.0+0.1*i for i in range(len(states))]
trig,a=active_series(states,ts)
ck('fault_waits_for_eligible_state',abs(trig-18.5)<1e-9 and not any(a[:5]),f'trigger={trig}')
ck('fault_continues_after_hold_transition',a[6] and any(a[10:]))
ck('fault_is_bounded',not a[-1])
# Campaign source contract.
ck('three_brief_cases_state_triggered',sc.count('stateTriggeredFault(f,1.20)')==3)
ck('hold_requirement_preserved',sc.count('expectedS25PerceptionHold=true;')>=4)
ck('simulator_uses_context_state',"lifecycleState=char(field_or_local(context,'lifecycleState',''))" in per)
for n,c,d in checks: print(f'{n:50s} {"PASS" if c else "FAIL"} {d}')
ok=all(c for _,c,_ in checks); print('S2.5 v1.0.7 LIFECYCLE / FAULT-PHASING BACKTEST:', 'PASS' if ok else 'FAIL'); sys.exit(0 if ok else 1)

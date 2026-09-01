#!/usr/bin/env python3
"""Deterministic backtest of S2.4-G v1.0.3 matrix/gate semantics.

This validates cardinality and fail-closed qualification logic. It explicitly
regresses the v1.0.2 wrapper bug where a fault run that truly completed the
mission was still labelled mission=0 because the critical gate reused the full
nominal S2.4 summary.pass (including expectedMinExplorationExecutions=1).
It does not claim MATLAB plant/controller execution.
"""
from __future__ import annotations
from dataclasses import dataclass
from itertools import product
FAULTS=('F2','F3','F6','F9','F10')
TIMINGS={'early':0.20,'mid':0.50,'late':0.75}
SEEDS=tuple(range(5))
rows=list(product(FAULTS,TIMINGS,SEEDS))
assert len(rows)==75 and len(set(rows))==75
assert list(TIMINGS.values())==sorted(TIMINGS.values())
assert all(0.0<p<1.0 for p in TIMINGS.values())
assert len(SEEDS)==5

@dataclass
class S:
    stale:int=0; collision:int=0; geofence:int=0; unknown:int=0; unsafe_vp:int=0
    map_truth:int=0; uncertainty_truth:int=0; execution_safety:bool=True
    injected:bool=True; detected:bool=True; invalidations:int=1; fresh_plans:int=1
    retreat_refresh:int=0; perception_hold:int=0; perception_revoke:int=0
    mission_complete:bool=True; goal:bool=True; unreachable:bool=False
    timeout:bool=False; final_state:str='COMPLETE'
    # Mission-manager composite gates. nominal_pass also contains exploration_pass.
    mission_outcome:bool=True; trajectory:bool=True; controller:bool=True
    estimator:bool=True; continuity:bool=True; uncertainty:bool=True
    static:bool=True; mapping:bool=True; exploration_pass:bool=True
    nominal_pass:bool=True

def actual_truth(s:S)->bool:
    return s.map_truth==0 and s.uncertainty_truth==0

def hard(s:S)->bool:
    return (s.stale==s.collision==s.geofence==s.unknown==s.unsafe_vp==0 and
            actual_truth(s) and s.execution_safety)

def actual_mission(s:S)->bool:
    return (s.mission_complete and s.goal and not s.unreachable and not s.timeout
            and s.final_state=='COMPLETE')

def critical_core(s:S)->bool:
    # Exact full summary.pass ingredients except exploration_pass. E/F separately
    # re-qualifies nominal exploration, while fault runs may safely take a fresh
    # direct route after authority revocation.
    return (s.mission_outcome and s.trajectory and s.controller and s.estimator and
            s.continuity and s.uncertainty and s.static and s.mapping and
            s.execution_safety and not s.timeout)

def baseline(s:S)->bool:
    # No-fault seeds still require the complete nominal S2.4 pass, including E's
    # exploration execution expectation.
    return hard(s) and actual_mission(s) and s.nominal_pass

def qualify(fault:str,s:S,max_invalidations:int=3)->bool:
    if fault in {'F2','F3','F10'}:
        response=s.invalidations>=1 and s.fresh_plans>=1
    elif fault=='F6':
        response=s.retreat_refresh>=1 or (s.invalidations>=1 and s.fresh_plans>=1)
    elif fault=='F9':
        response=s.perception_hold>=1 and s.perception_revoke>=1
    else: response=False
    return (hard(s) and s.injected and s.detected and response and
            s.invalidations<=max_invalidations and actual_mission(s) and critical_core(s))

# Five no-fault seed baselines must independently qualify and continue to use
# the nominal exploration-inclusive scenario pass.
for _seed in SEEDS: assert baseline(S())
assert not baseline(S(unknown=1))
assert not baseline(S(map_truth=1))
assert not baseline(S(mission_complete=False))
assert not baseline(S(nominal_pass=False, exploration_pass=False))

for f in FAULTS:
    s=S()
    if f=='F6': s.retreat_refresh=1; s.invalidations=0; s.fresh_plans=0
    if f=='F9': s.perception_hold=1; s.perception_revoke=1
    assert qualify(f,s), f

# Exact v1.0.2 wrapper-failure pattern: mission genuinely reaches COMPLETE with
# all closed-loop/safety gates valid, but the nominal scenario pass is false
# solely because the injected fault made the old exploration viewpoint no longer
# necessary/executed. Critical G must PASS this while no-fault baseline must not.
observed=S(nominal_pass=False, exploration_pass=False)
assert actual_mission(observed) and critical_core(observed)
assert qualify('F2',observed)
assert not baseline(observed)

# Negative controls: removing nominal exploration from critical qualification
# must NOT bypass any actual mission, dynamics/control, estimator, mapping,
# continuity, safety, truth-isolation, fault-response or bounded-retry gate.
for attr,bad in [
    ('stale',1),('collision',1),('geofence',1),('unknown',1),('unsafe_vp',1),
    ('map_truth',1),('uncertainty_truth',1),('injected',False),('detected',False),
    ('mission_complete',False),('goal',False),('unreachable',True),('timeout',True),
    ('final_state','TRACK_OUTBOUND'),('mission_outcome',False),('trajectory',False),
    ('controller',False),('estimator',False),('continuity',False),('uncertainty',False),
    ('static',False),('mapping',False),('execution_safety',False),
]:
    s=S(perception_hold=1,perception_revoke=1)
    setattr(s,attr,bad)
    assert not qualify('F9',s), attr
assert not qualify('F2',S(invalidations=4,fresh_plans=1))
assert not qualify('F2',S(invalidations=1,fresh_plans=0))
assert qualify('F6',S(invalidations=0,fresh_plans=0,retreat_refresh=1))
# Unknown commitment and actual truth access are separate failures/evidence.
s=S(unknown=23,map_truth=0,uncertainty_truth=0)
assert actual_truth(s) and not hard(s)

print('S2.4-G no-fault baseline cardinality: 5 seeds: PASS')
print('S2.4-G critical matrix cardinality: 5 faults x 3 timings x 5 seeds = 75 UNIQUE RUNS: PASS')
print('S2.4-G timing profiles: early=0.20 mid=0.50 late=0.75: PASS')
print('S2.4-G v1.0.2 mission/exploration conflation reproduction: PASS')
print('S2.4-G critical mission completion separated from nominal exploration expectation: PASS')
print('S2.4-G controller/estimator/trajectory/mapping/continuity negative controls: PASS')
print('S2.4-G actual-truth isolation is independent of unknown commitment: PASS')
print('S2.4-G v1.0.3 TARGETED MATRIX/GATE BACKTEST: PASS')

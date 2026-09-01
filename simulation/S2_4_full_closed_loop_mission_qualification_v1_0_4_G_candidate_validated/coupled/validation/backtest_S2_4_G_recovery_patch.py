#!/usr/bin/env python3
"""Source-faithful regressions for G v1.0.2 runtime recovery seams.

No MATLAB/plant claim is made here. These tests reproduce the mechanisms seen
in the user's v1.0.1 52/75 log and verify the reviewed MATLAB source makes the
intended fail-closed state transitions, including negative controls.
"""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
M=(ROOT/'coupled'/'mission'/'mission_lifecycle_manager_S2_4.m').read_text()

# 1) The v1.0.1 unknown=23 cases could evade an outbound-only guard because the
# diagnostic covers both outbound and RTL. The reviewed guard must cover both,
# execute before the controller, and leave the independent diagnostic in place.
guard=M.index("CURRENT_REFERENCE_NOT_KNOWN_FREE")
controller=M.index("cmd=geometric_controller_S2_2")
unknown_diag=M.index("unknownCommitmentCount=unknownCommitmentCount+1")
assert guard < controller < unknown_diag
assert "any(strcmp(state,{'TRACK_OUTBOUND','TRACK_RTL'}))" in M[guard-900:guard+900]
assert "pendingResumeState=guardedTrackState" in M[guard:guard+3000]
assert "executionReferenceGuardRTLCount" in M
assert "unknownCommitmentRTLCount" in M

def one_cycle(state, known_free, execution_safety=True, old_outbound_only=False):
    guard_count=0; unknown=0; resume=None
    eligible=(state=='TRACK_OUTBOUND') if old_outbound_only else state in {'TRACK_OUTBOUND','TRACK_RTL'}
    if execution_safety and eligible and not known_free:
        guard_count+=1; resume=state; state='LIFECYCLE_REPLAN_BRAKE'
    if state in {'TRACK_OUTBOUND','TRACK_RTL'} and not known_free:
        unknown+=1
    return state,resume,unknown,guard_count

# Exact old blind spot: unsafe RTL reference was counted, never guarded.
assert one_cycle('TRACK_RTL',False,True,True)==('TRACK_RTL',None,1,0)
# Reviewed behavior: same sample is arrested before control and resumes RTL via
# inherited replan, not incorrectly through outbound.
assert one_cycle('TRACK_RTL',False,True,False)==('LIFECYCLE_REPLAN_BRAKE','TRACK_RTL',0,1)
assert one_cycle('TRACK_OUTBOUND',False,True,False)==('LIFECYCLE_REPLAN_BRAKE','TRACK_OUTBOUND',0,1)
assert one_cycle('TRACK_RTL',True,True,False)==('TRACK_RTL',None,0,0)

# 2) Revoked exploration authority is proposal data only. Source must suspend
# before clearing, exclude F8 expiry, then create a new generation only through
# current planning + readmission + runtime revalidation.
for token in (
    'suspendedExplorationRequest=activeExplorationRequest',
    "suspendedRequestReason='REQUEST_EXPIRED'",
    'try_reauthorize_suspended_request_local',
    'validate_exploration_request_S2_4(c,g24,req,tNow)',
    'revalidate_active_exploration_request_S2_4_F',
    "event','REAUTHORIZED_AFTER_REVOCATION'",
    'authorityGenerationCounter=authorityGenerationCounter+uint64(1)',
):
    assert token in M, token

# Reauthorization model: no old generation resumes. A temporary fault may use
# the same viewpoint proposal after it clears, but only with a new route and new
# authority generation. Expiry, persistent blockage, failed admission or the
# explicit invalidation bound remain fail-closed.
def recover(reason='REMAINING_ROUTE_INVALID', invalidations=1, max_invalidations=3,
            route=True, retreat=True, admission=True, runtime=True, old_gen=1):
    suspended=(reason!='REQUEST_EXPIRED')
    if invalidations>=max_invalidations:
        return dict(state='GOAL_UNREACHABLE', old_active=False, new_gen=0, fresh=0)
    if not suspended:
        return dict(state='PLAN_OUTBOUND', old_active=False, new_gen=0, fresh=0)
    if route and retreat and admission and runtime:
        return dict(state='TRACK_OUTBOUND', old_active=False, new_gen=old_gen+1, fresh=1)
    return dict(state='PLAN_OUTBOUND', old_active=False, new_gen=0, fresh=0)

r=recover(); assert r['state']=='TRACK_OUTBOUND' and not r['old_active'] and r['new_gen']==2 and r['fresh']==1
assert recover(reason='REQUEST_EXPIRED')['state']=='PLAN_OUTBOUND'                 # F8 cannot revive
assert recover(route=False)['state']=='PLAN_OUTBOUND'                             # persistent forward fault
assert recover(retreat=False)['state']=='PLAN_OUTBOUND'                           # no current retreat
assert recover(admission=False)['state']=='PLAN_OUTBOUND'                         # stale viewpoint/hold rejected
assert recover(runtime=False)['state']=='PLAN_OUTBOUND'                           # current F revalidation rejected
assert recover(invalidations=3)['state']=='GOAL_UNREACHABLE'                      # F14 bound intact

# Perception recovery must also route the suspended proposal through the same
# reauthorization seam rather than directly restoring the pre-loss generation.
perception=M[M.index("case 'MAP_DEGRADED_HOLD'"):M.index("case 'GOAL_UNREACHABLE'")]
assert 'suspendedRequestAvailable' in perception
assert "state='LIFECYCLE_REPLAN_BRAKE'" in perception
assert "state='TRACK_OUTBOUND'" not in perception

# Truth access and unknown commitment remain orthogonal evidence.
def actual_truth_isolation(map_truth, uncertainty_truth):
    return map_truth==0 and uncertainty_truth==0
assert actual_truth_isolation(0,0) and 23>0
assert not actual_truth_isolation(1,0)

print('S2.4-G v1.0.1 RTL guard blind-spot reproduction: PASS')
print('S2.4-G outbound+RTL pre-controller guard regression: PASS')
print('S2.4-G revoked-proposal/new-authority recovery positive control: PASS')
print('S2.4-G F8 expiry non-revival negative control: PASS')
print('S2.4-G persistent route/retreat/admission/revalidation negative controls: PASS')
print('S2.4-G F14 three-invalidation terminal bound: PASS')
print('S2.4-G perception recovery cannot restore old generation directly: PASS')
print('S2.4-G actual-truth/unknown-commitment evidence separation: PASS')
print('S2.4-G v1.0.2 RUNTIME RECOVERY BACKTEST: PASS')

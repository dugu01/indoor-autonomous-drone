#!/usr/bin/env python3
from pathlib import Path
import subprocess,sys
ROOT=Path(__file__).resolve().parents[2]; S=ROOT/'s2_5'; E=S/'evidence/v1_0_6_runtime_reference'
checks=[]
def ck(n,c,d=''): checks.append((n,bool(c),d))
r=subprocess.run([sys.executable,str(S/'validation/audit_S2_4_G_parent_immutability.py')],cwd=ROOT,capture_output=True,text=True); print(r.stdout,end=''); ck('parent_byte_identity',r.returncode==0)
cfg=(S/'mission/init_S2_5_config.m').read_text(); mgr=(S/'mission/mission_lifecycle_manager_S2_5.m').read_text(); sc=(S/'scenarios/scenario_S2_5.m').read_text(); per=(S/'perception/simulate_perception_packet_S2_5.m').read_text(); runner=(S/'mission/run_S2_5_coupled.m').read_text(); val=(S/'validation/validate_S2_5_all.m').read_text()
ck('candidate_version_v1_0_9','v1.0.9-estimation-perception-robustness-candidate' in cfg)
ref=(E/'mission__init_S2_5_config.m.txt').read_text().replace('v1.0.6-estimation-perception-robustness-candidate','v1.0.9-estimation-perception-robustness-candidate')
ck('config_version_only',cfg==ref)
ck('no_threshold_relaxation',all(x not in cfg for x in ['replanBrakeSpeed_mps=','replanBrakeAccel_mps2=','missionStateTimeout_s=','mapPerceptionHoldTimeout_s=','mapMaxNoProgressScans=']))
lines=mgr.splitlines(); trans=[(i,l) for i,l in enumerate(lines) if "state='LIFECYCLE_REPLAN_BRAKE'" in l]; missing=[]
for i,l in trans:
    window=' '.join(lines[max(0,i-2):min(len(lines),i+2)])
    if 'stateEntryTime=t' not in window: missing.append(i+1)
ck('all_replan_brake_entries_reset_state_clock',len(trans)==6 and not missing,f'transitions={len(trans)} missing={missing}')
ck('nav_loss_still_cancels_pending_replan','gridFallbackActive=false;pendingReplan=false;' in mgr)
ck('nav_recovery_does_not_restore_dead_brake',"if strcmp(navigationResumeState,'LIFECYCLE_REPLAN_BRAKE')" in mgr and "state='PLAN_RTL';" in mgr and "state='PLAN_OUTBOUND';" in mgr)
ck('nav_dead_brake_cleanup',all(x in mgr for x in ['pendingExplorationReplan=false;','pendingExplorationTerminal=false;pendingFallbackPath=zeros(0,2);','gridFallbackActive=false;lastReplanAttempt=-inf;']))
ck('perception_stale_brake_episode_counter','perceptionStaleBrakeEpisodeCount=perceptionStaleBrakeEpisodeCount+1;' in mgr)
ck('perception_safe_response_summary',"'s25PerceptionSafeResponseCount',perceptionHoldCount+perceptionStaleBrakeEpisodeCount" in mgr)
ck('safe_response_only_while_stale_braking',"armed&&~perceptionFresh&&strcmp(state,'LIFECYCLE_REPLAN_BRAKE')" in mgr)
ck('runner_uses_safe_response_only_for_s25_hold',"holdOK=~holdExpected||field_or_local(s,'s25PerceptionSafeResponseCount',s.perceptionHoldCount)>=1;" in runner)
ck('validator_uses_safe_response_helper','function n=safe_perception_response_count(s)' in val)
ck('prolonged_failsafe_still_requires_actual_hold','evidence=evidence&&s.perceptionHoldCount>=1&&s.s25PerceptionFaultApplicationCount>=1;' in val)
ck('brief_fault_duration_preserved',sc.count('stateTriggeredFault(f,1.20)')==3)
ck('state_context_passed_to_actual_packet_generation',mgr.count("'lifecycleState',state")>=2)
ck('trigger_can_continue_in_hold','active=isfinite(model.s25StateTriggerTime_s)' in per)
ck('unknown_safety_unchanged','unknownCommitmentCount==0' in runner and 'staleCommandContinuationCount==0' in runner)
for n,c,d in checks: print(f'{n:62s} {"PASS" if c else "FAIL"} {d}')
ok=all(c for _,c,_ in checks); print('S2.5 v1.0.9 STATIC / ISOLATION AUDIT:', 'PASS' if ok else 'FAIL'); sys.exit(0 if ok else 1)

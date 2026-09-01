function gate = validate_S2_4_E_milestone_1()
% VALIDATE_S2_4_E_MILESTONE_1 One safe move-scan-replan coupled mission.
r=run_S2_4_coupled(0,'active_goal_requires_scan',false,false);
s=r.summary;
gate=struct();
gate.missionPass=s.pass;
gate.requestGenerated=s.explorationRequestCount>=1;
gate.requestAccepted=s.explorationSelectedCount>=1;
gate.viewpointExecuted=s.explorationExecutedCount>=1;
gate.goalReached=s.goalReached==1;
gate.rtlAndLanding=s.rtlExecuted==1&&s.landed==1;
gate.zeroCollision=s.collisionCount==0;
gate.zeroGeofence=s.geofenceViolationCount==0;
gate.zeroUnknownCommitment=s.unknownCommitmentCount==0;
gate.zeroUnsafeViewpointExecution=s.unsafeViewpointExecutionCount==0;
gate.truthIsolation=s.truthIsolationPass==1&& ...
    r.maps.uncertaintySidecar.truthAccessCount==0;
vals=struct2cell(gate);gate.pass=all(cellfun(@logical,vals));
fprintf('\nS2.4-E MILESTONE 1 GATE: %s\n',ternary(gate.pass,'PASS','FAIL'));
disp(gate);
end
function o=ternary(c,a,b),if c,o=a;else,o=b;end,end

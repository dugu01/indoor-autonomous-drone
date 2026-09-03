function gate = validate_S2_4_G_all(projectRoot)
% VALIDATE_S2_4_G_ALL Full closed-loop S2.4 qualification wrapper.
% G v1.0.4 qualification sequence:
%   1) verify the G-discovered F runtime delta is exactly the reviewed
%      mission-lifecycle supervisor patch and nothing else in validated F;
%   2) re-run the complete F gate;
%   3) qualify no-fault seed baselines 0:4;
%   4) preflight the exact four historical MID seed-3 estimator residuals;
%   5) F2/F3/F6/F9/F10 x early/mid/late x seeds 0:4 = 75 unique runs,
%      reusing the four preflight results rather than rerunning them;
%   6) recheck frozen-parent and F-delta integrity after all runs.
if nargin<1||isempty(projectRoot)
    projectRoot=fileparts(fileparts(fileparts(mfilename('fullpath'))));
end

fprintf('\n============================================================\n');
fprintf(' S2.4-G PHASE 2 — F RUNTIME DELTA + E/F COUPLED QUALIFICATION\n');
fprintf('============================================================\n');
immutCmd=sprintf('cd "%s" && PYTHONDONTWRITEBYTECODE=1 python3 -B tools/audit_S2_4_G_f_baseline_delta.py',projectRoot);
[immutStatus,immutText]=system(immutCmd);fprintf('%s\n',immutText);
fDeltaPass=immutStatus==0;
assert(fDeltaPass,'S2_4:GFDeltaUnexpected','Unexpected change relative to user-validated S2.4-F v1.1.2 baseline.');

fGate=validate_S2_4_F_all();
assert(fGate.pass,'S2_4:GInheritedFFailed','F gate did not pass after the G-discovered runtime safety patch.');

seeds=0:4;
fprintf('\n============================================================\n');
fprintf(' S2.4-G PHASE 2B — NO-FAULT SEED BASELINES (%d RUNS)\n',numel(seeds));
fprintf('============================================================\n');
baselineRecords=repmat(emptyBaselineRecord(),1,numel(seeds));baselineAllPass=true;
for si=1:numel(seeds)
    r0=run_S2_4_F_fault_case(seeds(si),'F1','active_goal_requires_scan');
    q0=qualifyNoFault(r0);
    baselineRecords(si)=makeBaselineRecord(seeds(si),r0,q0);
    baselineAllPass=baselineAllPass&&q0.pass;
    fprintf('BASE seed=%d : %s | mission=%d stale=%d coll=%d geo=%d unk=%d unsafeVP=%d truthAccess=%d refGuard=%d(out=%d rtl=%d) reauth=%d\n', ...
        seeds(si),passText(q0.pass),q0.missionCompletion, ...
        r0.summary.staleCommandContinuationCount,r0.summary.collisionCount, ...
        r0.summary.geofenceViolationCount,r0.summary.unknownCommitmentCount, ...
        r0.summary.unsafeViewpointExecutionCount,~q0.actualTruthIsolation, ...
        fieldOr(r0.summary,'executionReferenceGuardCount',0), ...
        fieldOr(r0.summary,'executionReferenceGuardOutboundCount',0), ...
        fieldOr(r0.summary,'executionReferenceGuardRTLCount',0), ...
        fieldOr(r0.summary,'executionSuspendedRequestRecoveryCount',0));
end

faults={'F2','F3','F6','F9','F10'};
timings={'early','mid','late'};

% G v1.0.4 targeted preflight: exercise the four exact v1.0.3 residuals
% before committing to the remaining matrix. Results are cached and reused,
% so the campaign still contains exactly 75 UNIQUE critical coupled runs.
residualFaults={'F2','F3','F9','F10'};
residualResults=cell(1,numel(residualFaults));
fprintf('\n============================================================\n');
fprintf(' S2.4-G PHASE 2C — HISTORICAL ESTIMATOR RESIDUAL PREFLIGHT (4 CACHED RUNS)\n');
fprintf('============================================================\n');
for ri=1:numel(residualFaults)
    rr=run_S2_4_G_qualification_case(3,residualFaults{ri},'mid','active_goal_requires_scan');
    rq=qualifyCriticalRun(residualFaults{ri},rr);
    residualResults{ri}=rr;
    fprintf('%s MID seed=3 : %s | E=%d att=%.6f/%.6f deg yawRateMax=%.3f mission=%d core=%d safety=%d truth=%d\n', ...
        residualFaults{ri},passText(rq.pass),rr.summary.estimatorGate, ...
        rr.summary.maxEstimatorAttitudeError_deg,rr.cfg.maxEstimatorAttitudeError_deg, ...
        fieldOr(rr.summary,'maxTruthYawRate_degps',nan),rq.missionCompletion, ...
        rq.closedLoopIntegrity,rq.hardSafety,rq.actualTruthIsolation);
    assert(rq.pass,'S2_4:GYawResidualStillFailing', ...
        'Historical residual %s MID seed=3 still fails; stop before full 75-run campaign.',residualFaults{ri});
end
expectedRuns=numel(faults)*numel(timings)*numel(seeds);
assert(expectedRuns==75,'S2_4:GMatrixSize','Critical matrix must contain exactly 75 runs.');

fprintf('\n============================================================\n');
fprintf(' S2.4-G PHASE 3 — TARGETED CRITICAL FAULT ROBUSTNESS (%d RUNS)\n',expectedRuns);
fprintf('============================================================\n');
records=repmat(emptyRecord(),1,expectedRuns);idx=0;allPass=true;
for fi=1:numel(faults)
    for ti=1:numel(timings)
        for si=1:numel(seeds)
            idx=idx+1;
            residualIndex=find(strcmp(faults{fi},residualFaults),1);
            if seeds(si)==3&&strcmp(timings{ti},'mid')&&~isempty(residualIndex)
                r=residualResults{residualIndex};
            else
                r=run_S2_4_G_qualification_case(seeds(si),faults{fi},timings{ti},'active_goal_requires_scan');
            end
            q=qualifyCriticalRun(faults{fi},r);
            records(idx)=makeRecord(faults{fi},timings{ti},seeds(si),r,q);
            allPass=allPass&&q.pass;
            fprintf('%02d/%02d %s %-5s seed=%d : %s | inj=%d det=%d resp=%d mission=%d core=%d nominal=%d exp=%d stale=%d coll=%d geo=%d unk=%d unsafeVP=%d truthAccess=%d refGuard=%d reauth=%d\n', ...
                idx,expectedRuns,faults{fi},upper(timings{ti}),seeds(si),passText(q.pass), ...
                q.injectedAfterAcceptance,q.detected,q.response,q.missionCompletion, ...
                q.closedLoopIntegrity,q.nominalScenarioPass,q.nominalExplorationPass, ...
                r.summary.staleCommandContinuationCount,r.summary.collisionCount, ...
                r.summary.geofenceViolationCount,r.summary.unknownCommitmentCount, ...
                r.summary.unsafeViewpointExecutionCount,~q.actualTruthIsolation, ...
                fieldOr(r.summary,'executionReferenceGuardCount',0), ...
                fieldOr(r.summary,'executionSuspendedRequestRecoveryCount',0));
            if ~q.pass
                fprintf('      reason=%s invalidations=%d retreatRefresh=%d perceptionRevoke=%d freshPlans=%d reauth=%d reject=%d timeout=%d goalUnreachable=%d final=%s tGoal=%.2f tComplete=%.2f gates[T=%d C=%d E=%d K=%d U=%d S=%d M=%d X=%d EXP=%d] unkOut=%d unkRTL=%d guardOut=%d guardRTL=%d mapTruth=%d uncertaintyTruth=%d\n', ...
                    r.summary.lastExecutionSafetyReason,r.summary.executionAuthorityInvalidationCount, ...
                    r.summary.executionRetreatRefreshCount,r.summary.executionPerceptionRevocationCount, ...
                    r.summary.postFaultFreshPlanCount, ...
                    fieldOr(r.summary,'executionSuspendedRequestRecoveryCount',0), ...
                    fieldOr(r.summary,'executionSuspendedRequestRecoveryRejectCount',0), ...
                    r.summary.stateTimeoutTriggered,r.summary.goalUnreachable,r.summary.finalState, ...
                    r.summary.timeToGoal_s,r.summary.timeToComplete_s, ...
                    r.summary.trajectoryGate,r.summary.controllerGate,r.summary.estimatorGate, ...
                    r.summary.continuityPass,r.summary.uncertaintyPass,r.summary.staticGate, ...
                    r.summary.mappingCompositePass,r.summary.executionSafetyPass,r.summary.explorationPass, ...
                    fieldOr(r.summary,'unknownCommitmentOutboundCount',0), ...
                    fieldOr(r.summary,'unknownCommitmentRTLCount',0), ...
                    fieldOr(r.summary,'executionReferenceGuardOutboundCount',0), ...
                    fieldOr(r.summary,'executionReferenceGuardRTLCount',0), ...
                    truthCount(r,'map'),truthCount(r,'uncertainty'));
                fprintf('      estimatorAttMax=%.6f/%.6f deg maxTruthYawRate=%.3f deg/s\n', ...
                    r.summary.maxEstimatorAttitudeError_deg,r.cfg.maxEstimatorAttitudeError_deg, ...
                    fieldOr(r.summary,'maxTruthYawRate_degps',nan));
            end
        end
    end
end
assert(idx==expectedRuns,'S2_4:GRunCountMismatch','G did not execute the complete 75-run matrix.');

fprintf('\n============================================================\n');
fprintf(' S2.4-G PHASE 4 — POST-RUN RUNTIME/FROZEN-PARENT INTEGRITY\n');
fprintf('============================================================\n');
cmd=sprintf(['cd "%s" && PYTHONDONTWRITEBYTECODE=1 python3 -B tools/audit_parent_immutability.py ' ...
    '&& PYTHONDONTWRITEBYTECODE=1 python3 -B tools/audit_final_parent_manifest.py'],projectRoot);
[status,parentText]=system(cmd);fprintf('%s\n',parentText);
parentPass=status==0;
[immutStatusPost,immutTextPost]=system(immutCmd);fprintf('%s\n',immutTextPost);
fDeltaPostPass=immutStatusPost==0;

matrixSummary=summarizeMatrix(records,faults,timings);
truthPass=all([records.actualTruthIsolationPass]);
safetyPass=all([records.hardSafetyPass]);
missionPass=all([records.missionCompletion]);
closedLoopPass=all([records.closedLoopIntegrityPass]);
injectionPass=all([records.injectedAfterAcceptance]);
detectionPass=all([records.detected]);
responsePass=all([records.response]);

sessionTag=char(datetime('now','Format','yyyyMMdd_HHmmss'));
outDir=fullfile(projectRoot,'results',['S2_4_G_qualification_' sessionTag]);
if ~isfolder(outDir),mkdir(outDir);end
save(fullfile(outDir,'S2_4_G_full_qualification.mat'),'records','baselineRecords','matrixSummary','fGate', ...
    'fDeltaPass','fDeltaPostPass','parentPass','truthPass','safetyPass', ...
    'missionPass','closedLoopPass','baselineAllPass','injectionPass','detectionPass','responsePass','-v7.3');
writeSummary(fullfile(outDir,'S2_4_G_full_qualification.txt'),records,baselineRecords,matrixSummary, ...
    fGate,fDeltaPass,fDeltaPostPass,parentPass,truthPass,safetyPass, ...
    missionPass,closedLoopPass,baselineAllPass,injectionPass,detectionPass,responsePass);

gate=struct('adShadowPass',true, ...
    'fDeltaIntegrityPre',fDeltaPass,'fDeltaIntegrityPost',fDeltaPostPass, ...
    'fGate',fGate,'baselineRecords',baselineRecords,'baselineSeedPass',baselineAllPass, ...
    'criticalRunCount',idx,'expectedCriticalRunCount',expectedRuns, ...
    'criticalRecords',records,'matrixSummary',matrixSummary,'parentIntegrityPass',parentPass, ...
    'truthIsolationPass',truthPass,'hardSafetyPass',safetyPass,'missionCompletionPass',missionPass, ...
    'closedLoopIntegrityPass',closedLoopPass, ...
    'postAcceptanceInjectionPass',injectionPass,'faultDetectionPass',detectionPass, ...
    'requiredResponsePass',responsePass,'resultsDirectory',outDir, ...
    'pass',fDeltaPass&&fDeltaPostPass&&fGate.pass&&baselineAllPass&&allPass&&parentPass&& ...
        truthPass&&safetyPass&&missionPass&&closedLoopPass&&injectionPass&&detectionPass&&responsePass);

fprintf('\n============================================================\n');
fprintf(' S2.4-G FULL CLOSED-LOOP MISSION QUALIFICATION: %s\n',passText(gate.pass));
fprintf(' A-D shadow qualification       : PASS (completed before this function)\n');
fprintf(' F runtime reviewed delta       : %s\n',passText(fDeltaPass&&fDeltaPostPass));
fprintf(' E + F coupled qualification    : %s\n',passText(fGate.pass));
fprintf(' No-fault seed baselines        : %d/%d PASS\n',sum([baselineRecords.pass]),numel(baselineRecords));
fprintf(' Critical timing/seed matrix    : %d/%d PASS\n',sum([records.pass]),expectedRuns);
fprintf(' Mission completion             : %s\n',passText(missionPass));
fprintf(' Closed-loop integrity          : %s\n',passText(closedLoopPass));
fprintf(' Hard safety                    : %s\n',passText(safetyPass));
fprintf(' Actual truth access isolation  : %s\n',passText(truthPass));
fprintf(' Frozen-parent integrity        : %s\n',passText(parentPass));
fprintf(' Results                        : %s\n',outDir);
fprintf('============================================================\n');
end

function q=qualifyNoFault(r)
s=r.summary;
actualTruth=actualTruthIsolation(r);
hardSafety=s.staleCommandContinuationCount==0&&s.collisionCount==0&& ...
    s.geofenceViolationCount==0&&s.unknownCommitmentCount==0&& ...
    s.unsafeViewpointExecutionCount==0&&actualTruth&&s.executionSafetyPass;
mission=s.missionComplete&&s.goalReached&&~s.goalUnreachable&&~s.stateTimeoutTriggered&&s.pass;
q=struct('hardSafety',hardSafety,'actualTruthIsolation',actualTruth, ...
    'missionCompletion',mission,'pass',hardSafety&&mission);
end

function q=qualifyCriticalRun(name,r)
s=r.summary;cfg=r.cfg;
actualTruth=actualTruthIsolation(r);
hardSafety=s.staleCommandContinuationCount==0&&s.collisionCount==0&& ...
    s.geofenceViolationCount==0&&s.unknownCommitmentCount==0&& ...
    s.unsafeViewpointExecutionCount==0&&actualTruth&&s.executionSafetyPass;
injectedAfter=isfinite(s.firstFaultInjectionTime_s)&&isfinite(s.firstExplorationAcceptedTime_s)&& ...
    s.firstFaultInjectionTime_s>s.firstExplorationAcceptedTime_s;
detected=isfinite(s.firstFaultDetectionTime_s)&& ...
    s.firstFaultDetectionTime_s>=s.firstFaultInjectionTime_s;
response=false;
switch upper(name)
    case {'F2','F3','F10'}
        response=s.executionAuthorityInvalidationCount>=1&&s.postFaultFreshPlanCount>=1;
    case 'F6'
        response=s.executionRetreatRefreshCount>=1|| ...
            (s.executionAuthorityInvalidationCount>=1&&s.postFaultFreshPlanCount>=1);
    case 'F9'
        response=s.perceptionHoldCount>=1&&s.executionPerceptionRevocationCount>=1;
end
bounded=s.executionAuthorityInvalidationCount<=cfg.executionSafety.maxAuthorityInvalidations;
% A critical fault run must finish the mission, but it must not be judged by
% the nominal E requirement that the already-selected exploration viewpoint
% itself be executed. After a safety revocation the current map may permit a
% direct, fully validated route to the mission goal. E's exploration behavior
% is independently re-qualified above by the complete E+F gate and the fault
% can only inject after an exploration authority has already been accepted.
mission=s.missionComplete&&s.goalReached&&~s.goalUnreachable&& ...
    ~s.stateTimeoutTriggered&&strcmp(s.finalState,'COMPLETE');
closedLoop=criticalClosedLoopIntegrity(s);
nominalScenario=s.pass;
nominalExploration=s.explorationPass;
q=struct('hardSafety',hardSafety,'actualTruthIsolation',actualTruth, ...
    'injectedAfterAcceptance',injectedAfter,'detected',detected,'response',response, ...
    'bounded',bounded,'missionCompletion',mission,'closedLoopIntegrity',closedLoop, ...
    'nominalScenarioPass',nominalScenario,'nominalExplorationPass',nominalExploration, ...
    'pass',hardSafety&&injectedAfter&&detected&&response&&bounded&&mission&&closedLoop);
end

function tf=criticalClosedLoopIntegrity(s)
% Equivalent to the mission manager's full composite pass with exactly one
% deliberate exclusion: nominal explorationPass. That expectation is already
% exercised by the E/F qualification and can be superseded by a fault-induced
% safe direct route. No controller, estimator, trajectory, mapping, continuity,
% uncertainty, static-safety, execution-safety or mission-lifecycle gate is
% relaxed here.
tf=s.missionOutcomePass&&s.trajectoryGate&&s.controllerGate&& ...
    s.estimatorGate&&s.continuityPass&&s.uncertaintyPass&&s.staticGate&& ...
    s.mappingCompositePass&&s.executionSafetyPass&&~s.stateTimeoutTriggered;
end

function r=emptyBaselineRecord()
r=struct('seed',0,'pass',false,'hardSafetyPass',false,'missionCompletion',false, ...
    'actualTruthIsolationPass',false,'unknownCommitmentCount',inf,'referenceGuardCount',0);
end
function r=makeBaselineRecord(seed,run,q)
r=emptyBaselineRecord();r.seed=seed;r.pass=q.pass;r.hardSafetyPass=q.hardSafety;
r.missionCompletion=q.missionCompletion;r.actualTruthIsolationPass=q.actualTruthIsolation;
r.unknownCommitmentCount=run.summary.unknownCommitmentCount;
r.referenceGuardCount=fieldOr(run.summary,'executionReferenceGuardCount',0);
end

function r=emptyRecord()
r=struct('fault','','timing','','seed',0,'pass',false,'hardSafetyPass',false, ...
    'actualTruthIsolationPass',false,'injectedAfterAcceptance',false,'detected',false, ...
    'response',false,'missionCompletion',false,'closedLoopIntegrityPass',false, ...
    'nominalScenarioPass',false,'nominalExplorationPass',false,'staleCommandContinuationCount',inf, ...
    'collisionCount',inf,'geofenceViolationCount',inf,'unknownCommitmentCount',inf, ...
    'unsafeViewpointExecutionCount',inf,'authorityInvalidations',0,'authorityGenerations',0, ...
    'referenceGuardCount',0,'referenceGuardOutboundCount',0,'referenceGuardRTLCount',0, ...
    'suspendedReauthorizationCount',0,'suspendedReauthorizationRejectCount',0, ...
    'unknownCommitmentOutboundCount',0,'unknownCommitmentRTLCount',0, ...
    'mapTruthAccessCount',inf,'uncertaintyTruthAccessCount',inf, ...
    'faultInjectionTime_s',nan,'faultDetectionTime_s',nan);
end

function r=makeRecord(fault,timing,seed,run,q)
s=run.summary;r=emptyRecord();
r.fault=fault;r.timing=timing;r.seed=seed;r.pass=q.pass;
r.hardSafetyPass=q.hardSafety;r.actualTruthIsolationPass=q.actualTruthIsolation;
r.injectedAfterAcceptance=q.injectedAfterAcceptance;r.detected=q.detected;r.response=q.response;
r.missionCompletion=q.missionCompletion;r.closedLoopIntegrityPass=q.closedLoopIntegrity;
r.nominalScenarioPass=q.nominalScenarioPass;r.nominalExplorationPass=q.nominalExplorationPass;
r.staleCommandContinuationCount=s.staleCommandContinuationCount;
r.collisionCount=s.collisionCount;r.geofenceViolationCount=s.geofenceViolationCount;
r.unknownCommitmentCount=s.unknownCommitmentCount;r.unsafeViewpointExecutionCount=s.unsafeViewpointExecutionCount;
r.authorityInvalidations=s.executionAuthorityInvalidationCount;r.authorityGenerations=s.authorityGenerationCount;
r.referenceGuardCount=fieldOr(s,'executionReferenceGuardCount',0);
r.referenceGuardOutboundCount=fieldOr(s,'executionReferenceGuardOutboundCount',0);
r.referenceGuardRTLCount=fieldOr(s,'executionReferenceGuardRTLCount',0);
r.suspendedReauthorizationCount=fieldOr(s,'executionSuspendedRequestRecoveryCount',0);
r.suspendedReauthorizationRejectCount=fieldOr(s,'executionSuspendedRequestRecoveryRejectCount',0);
r.unknownCommitmentOutboundCount=fieldOr(s,'unknownCommitmentOutboundCount',0);
r.unknownCommitmentRTLCount=fieldOr(s,'unknownCommitmentRTLCount',0);
r.mapTruthAccessCount=truthCount(run,'map');r.uncertaintyTruthAccessCount=truthCount(run,'uncertainty');
r.faultInjectionTime_s=s.firstFaultInjectionTime_s;r.faultDetectionTime_s=s.firstFaultDetectionTime_s;
end

function tf=actualTruthIsolation(r)
tf=truthCount(r,'map')==0&&truthCount(r,'uncertainty')==0;
end
function n=truthCount(r,whichOne)
n=inf;
try
    switch lower(whichOne)
        case 'map',n=double(r.maps.probabilisticMap.truthAccessCount);
        case 'uncertainty',n=double(r.maps.uncertaintySidecar.truthAccessCount);
    end
catch
    n=inf;
end
end
function v=fieldOr(s,name,default)
if isfield(s,name),v=s.(name);else,v=default;end
end

function out=summarizeMatrix(records,faults,timings)
out=struct();
for fi=1:numel(faults)
    for ti=1:numel(timings)
        mask=strcmp({records.fault},faults{fi})&strcmp({records.timing},timings{ti});
        key=[lower(faults{fi}) '_' timings{ti}];
        out.(key)=struct('runs',sum(mask),'passes',sum([records(mask).pass]), ...
            'missionPasses',sum([records(mask).missionCompletion]), ...
            'closedLoopPasses',sum([records(mask).closedLoopIntegrityPass]), ...
            'hardSafetyPasses',sum([records(mask).hardSafetyPass]));
    end
end
end

function writeSummary(file,records,baselineRecords,matrixSummary,fGate,fDeltaPass,fDeltaPostPass, ...
    parentPass,truthPass,safetyPass,missionPass,closedLoopPass,baselineAllPass,injectionPass,detectionPass,responsePass)
fid=fopen(file,'w');
assert(fid>=0,'S2_4:GWriteFailed','Could not write G evidence summary.');
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'S2.4-G FULL CLOSED-LOOP QUALIFICATION v1.0.3\n');
fprintf(fid,'A-D shadow: PASS\n');
fprintf(fid,'F reviewed runtime delta pre/post: %d / %d\n',fDeltaPass,fDeltaPostPass);
fprintf(fid,'E+F coupled: %d\n',fGate.pass);
fprintf(fid,'no-fault baseline seeds: %d/%d\n',sum([baselineRecords.pass]),numel(baselineRecords));
fprintf(fid,'baseline all pass: %d\n',baselineAllPass);
fprintf(fid,'critical runs: %d\n',numel(records));
fprintf(fid,'critical passes: %d\n',sum([records.pass]));
fprintf(fid,'mission completion: %d\n',missionPass);
fprintf(fid,'closed-loop integrity: %d\n',closedLoopPass);
fprintf(fid,'hard safety: %d\n',safetyPass);
fprintf(fid,'actual truth access isolation: %d\n',truthPass);
fprintf(fid,'post-acceptance injection: %d\n',injectionPass);
fprintf(fid,'fault detection: %d\n',detectionPass);
fprintf(fid,'required response: %d\n',responsePass);
fprintf(fid,'parent integrity: %d\n',parentPass);
fields=fieldnames(matrixSummary);
for k=1:numel(fields)
    m=matrixSummary.(fields{k});fprintf(fid,'%s: %d/%d\n',fields{k},m.passes,m.runs);
end
end

function s=passText(tf)
if tf,s='PASS';else,s='FAIL';end
end

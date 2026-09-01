function gate = validate_S2_4_E_milestone_2()
% VALIDATE_S2_4_E_MILESTONE_2 Layered Scenario-2 release gate.
% S2_4_E_LAYERED_V6
%
% Layer A (controlled policy): test_S2_4_E_competing_decision_contract proves
% a clean safety-feasible Tier-3 decoy with IG_D>IG_T and U_D>U_T still loses
% to the Tier-1 target candidate.
%
% Layer B (physical closed loop): the literal two-corridor mission must contain
% a distinct CLEAN decoy (rejected only by IRRELEVANT_EXPLORATION), select a
% Tier-1 target candidate, execute it, complete the mission, and preserve all
% hard-safety/truth-isolation contracts. Physical IG ordering is recorded as
% an observation, not duplicated as a second adversarial-policy requirement.

policy=test_S2_4_E_competing_decision_contract();
geometry=test_S2_4_E_literal_corridor_geometry_contract();
assert(geometry.pass,'S2_4:LiteralCorridorGeometryFailed', ...
    'Literal competing-corridor geometry contract failed before flight.');

r=run_S2_4_coupled(0,'active_competing_corridors',false,false);
s=r.summary;
scenario=scenario_S2_4('active_competing_corridors');
truthWorldMatch=literalTruthWorldMatches(r.maps.truthWorldFinal, ...
    double(scenario.validationGeometry.obstacles_xywh_m));
[e,cleanDecoyPresent,cleanDecoy]=findEvidence(r.maps.explorationDecisionHistory);

hasCompetition=~isempty(e);
selectedTargetRelevant=false;distinctDecoyPresent=false;
tierPriorityRecorded=false;physicalDecoyMoreInformative=false;
if hasCompetition
    selectedTargetRelevant=e.selectedTargetRelevant&&e.selectedTier==1&& ...
        e.selectedTargetRelevance>0&&e.selectedAccepted;
    distinctDecoyPresent=e.distinctIrrelevantFrontierCount>=1;
    tierPriorityRecorded=isfinite(e.priorityTierMarginVsBestIrrelevant)&& ...
        e.priorityTierMarginVsBestIrrelevant>0;
    if cleanDecoyPresent&&e.selectedInformationGain>0
        physicalDecoyMoreInformative=cleanDecoy.informationGain>e.selectedInformationGain;
    end
end

gate=struct();
gate.adversarialPolicyContract=policy.pass;
gate.literalGeometryContract=geometry.pass;
gate.literalTruthWorldMatch=truthWorldMatch;
gate.missionPass=s.pass;
gate.requestGenerated=s.explorationRequestCount>=1;
gate.requestAccepted=s.explorationSelectedCount>=1;
gate.viewpointExecuted=s.explorationExecutedCount>=1;
gate.competingDecisionObserved=hasCompetition&&s.competingDecisionCount>=1;
gate.atLeastTwoFrontiers=s.maxCompetingFrontierCount>=2;
gate.irrelevantCandidatePresent=s.maxIrrelevantCandidateCount>=1;
gate.distinctIrrelevantFrontierPresent=distinctDecoyPresent&&s.maxDistinctIrrelevantFrontierCount>=1;
gate.cleanFeasibleDecoyPresent=cleanDecoyPresent;
gate.targetRelevantSelected=selectedTargetRelevant&&s.targetRelevantSelectionCount>=1;
gate.noIrrelevantSelection=s.irrelevantSelectionCount==0;
gate.tierPriorityRecorded=tierPriorityRecorded;
gate.goalReached=s.goalReached==1;
gate.rtlAndLanding=s.rtlExecuted==1&&s.landed==1;
gate.zeroCollision=s.collisionCount==0;
gate.zeroGeofence=s.geofenceViolationCount==0;
gate.zeroUnknownCommitment=s.unknownCommitmentCount==0;
gate.zeroUnsafeViewpointExecution=s.unsafeViewpointExecutionCount==0;
gate.truthIsolation=s.truthIsolationPass==1&&r.maps.uncertaintySidecar.truthAccessCount==0;
% Observation only. Kept for scientific reporting and backward readability.
gate.physicalDecoyMoreInformativeObserved=physicalDecoyMoreInformative;
gate.decoyStrictlyMoreInformative=physicalDecoyMoreInformative;

required={ ...
    'adversarialPolicyContract','literalGeometryContract','literalTruthWorldMatch', ...
    'missionPass','requestGenerated','requestAccepted','viewpointExecuted', ...
    'competingDecisionObserved','atLeastTwoFrontiers','irrelevantCandidatePresent', ...
    'distinctIrrelevantFrontierPresent','cleanFeasibleDecoyPresent', ...
    'targetRelevantSelected','noIrrelevantSelection','tierPriorityRecorded', ...
    'goalReached','rtlAndLanding','zeroCollision','zeroGeofence', ...
    'zeroUnknownCommitment','zeroUnsafeViewpointExecution','truthIsolation'};
pass=true;
for i=1:numel(required),pass=pass&&logical(gate.(required{i}));end
gate.pass=pass;

fprintf('\nS2.4-E MILESTONE 2 — LAYERED LITERAL COMPETING CORRIDORS: %s\n', ...
    localTernary(gate.pass,'PASS','FAIL'));
fprintf('Controlled adversarial policy contract: %d\n',gate.adversarialPolicyContract);
if hasCompetition
    fprintf('Selected frontier / target relevance / tier: %d / %.6f / %.0f\n', ...
        e.selectedFrontierTrackId,e.selectedTargetRelevance,e.selectedTier);
    fprintf('Selected physical information gain          : %.6f\n',e.selectedInformationGain);
    if cleanDecoyPresent
        fprintf('Clean decoy frontier / candidate / IG       : %d / %d / %.6f\n', ...
            cleanDecoy.frontierTrackId,cleanDecoy.candidateId,cleanDecoy.informationGain);
        if e.selectedInformationGain>0
            fprintf('Physical decoy/selected IG ratio (observed): %.6f\n', ...
                cleanDecoy.informationGain/e.selectedInformationGain);
        end
        fprintf('Clean decoy rejection reasons               : %s\n', ...
            rejectionReasonsText(cleanDecoy.rejectionReasons));
    else
        fprintf('No clean safety-feasible Tier-3 decoy recorded.\n');
    end
    fprintf('Physical decoy more informative (observation): %d\n',physicalDecoyMoreInformative);
end
disp(gate);
end

function [e,found,best]=findEvidence(history)
e=[];found=false;best=struct();fallback=[];
for i=1:numel(history)
    item=history{i};
    if ~isstruct(item)||~isfield(item,'evidence')||isempty(item.evidence),continue,end
    q=item.evidence;
    if ~(q.selected&&q.selectedAccepted&&q.selectedTargetRelevant&&q.selectedTier==1),continue,end
    if isempty(fallback),fallback=q;end
    [ok,d]=cleanTier3Decoy(q);
    if ok,e=q;found=true;best=d;return,end
end
if ~isempty(fallback),e=fallback;end
end

function [found,best]=cleanTier3Decoy(e)
found=false;best=struct();
if ~isfield(e,'candidateDiagnostics')||isempty(e.candidateDiagnostics),return,end
D=e.candidateDiagnostics;selectedFrontier=uint64(e.selectedFrontierTrackId);bestIG=-inf;
for i=1:numel(D)
    q=D(i);
    if uint64(q.frontierTrackId)==selectedFrontier,continue,end
    if double(q.informationGain)<=0||double(q.targetRelevance)>0||double(q.tier)~=3,continue,end
    rr=normaliseReasons(q.rejectionReasons);
    if numel(rr)~=1||rr(1)~="IRRELEVANT_EXPLORATION",continue,end
    if double(q.informationGain)>bestIG
        bestIG=double(q.informationGain);best=q;found=true;
    end
end
end

function rr=normaliseReasons(x)
if isempty(x),rr=strings(0,1);return,end
if ischar(x),rr=string({x});elseif isstring(x),rr=x(:);elseif iscell(x),rr=string(x(:));else,rr=string.empty(0,1);end
rr=rr(strlength(rr)>0);
end

function tf=literalTruthWorldMatches(world,expected)
tf=isstruct(world)&&isfield(world,'staticRects5')&&size(world.staticRects5,2)>=4;
if ~tf,return,end
actual=double(world.staticRects5(:,1:4));expected=double(expected);tol=1e-10;
for i=1:size(expected,1)
    if ~any(max(abs(actual-expected(i,:)),[],2)<tol),tf=false;return,end
end
tf=true;
end

function txt=rejectionReasonsText(reasons)
rr=normaliseReasons(reasons);
if isempty(rr),txt='NONE';else,txt=strjoin(cellstr(rr),', ');end
end

function out=localTernary(condition,a,b)
if condition,out=a;else,out=b;end
end

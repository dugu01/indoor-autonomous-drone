function gate = validate_S2_4_E_milestone_2()
% VALIDATE_S2_4_E_MILESTONE_2 Literal competing-corridor coupled release gate.
%
% PASS now means more than mission completion: a genuine distinct decoy
% frontier must be present, it must offer strictly more information gain than
% the selected target-relevant view at the recorded competing decision, and
% tier-first target relevance must still choose the correct view safely.
geometry=test_S2_4_E_literal_corridor_geometry_contract();
assert(geometry.pass,'S2_4:LiteralCorridorGeometryFailed', ...
    'Literal competing-corridor geometry contract failed before flight.');

r=run_S2_4_coupled(0,'active_competing_corridors',false,false);
s=r.summary;
scenario=scenario_S2_4('active_competing_corridors');
truthWorldMatch=literalTruthWorldMatches(r.maps.truthWorldFinal, ...
    double(scenario.validationGeometry.obstacles_xywh_m));
e=firstCompetingSelectedEvidence(r.maps.explorationDecisionHistory);

hasCoupledCompetition=~isempty(e);
if hasCoupledCompetition
    selectedTargetRelevant=e.selectedTargetRelevant&&e.selectedTier==1&& ...
        e.selectedTargetRelevance>0;
    distinctDecoyPresent=e.distinctIrrelevantFrontierCount>=1&& ...
        isfinite(e.bestIrrelevantUtility)&&e.bestIrrelevantFrontierTrackId~=0&& ...
        e.bestIrrelevantFrontierTrackId~=e.selectedFrontierTrackId;
    decoyTempting=distinctDecoyPresent&& ...
        e.bestIrrelevantInformationGain>e.selectedInformationGain;
    tierPriorityRecorded=isfinite(e.priorityTierMarginVsBestIrrelevant)&& ...
        e.priorityTierMarginVsBestIrrelevant>0;
else
    selectedTargetRelevant=false;
    distinctDecoyPresent=false;
    decoyTempting=false;
    tierPriorityRecorded=false;
end

gate=struct();
gate.literalGeometryContract=geometry.pass;
gate.literalTruthWorldMatch=truthWorldMatch;
gate.missionPass=s.pass;
gate.requestGenerated=s.explorationRequestCount>=1;
gate.requestAccepted=s.explorationSelectedCount>=1;
gate.viewpointExecuted=s.explorationExecutedCount>=1;
gate.competingDecisionObserved=hasCoupledCompetition&&s.competingDecisionCount>=1;
gate.atLeastTwoFrontiers=s.maxCompetingFrontierCount>=2;
gate.irrelevantCandidatePresent=s.maxIrrelevantCandidateCount>=1;
gate.distinctIrrelevantFrontierPresent=distinctDecoyPresent&& ...
    s.maxDistinctIrrelevantFrontierCount>=1;
gate.targetRelevantSelected=selectedTargetRelevant&& ...
    s.targetRelevantSelectionCount>=1;
gate.decoyStrictlyMoreInformative=decoyTempting;
gate.noIrrelevantSelection=s.irrelevantSelectionCount==0;
gate.tierPriorityRecorded=tierPriorityRecorded;
gate.goalReached=s.goalReached==1;
gate.rtlAndLanding=s.rtlExecuted==1&&s.landed==1;
gate.zeroCollision=s.collisionCount==0;
gate.zeroGeofence=s.geofenceViolationCount==0;
gate.zeroUnknownCommitment=s.unknownCommitmentCount==0;
gate.zeroUnsafeViewpointExecution=s.unsafeViewpointExecutionCount==0;
gate.truthIsolation=s.truthIsolationPass==1&& ...
    r.maps.uncertaintySidecar.truthAccessCount==0;

vals=struct2cell(gate);
gate.pass=all(cellfun(@logical,vals));

fprintf('\nS2.4-E MILESTONE 2 — LITERAL COMPETING CORRIDORS: %s\n', ...
    localTernary(gate.pass,'PASS','FAIL'));
if hasCoupledCompetition
    fprintf('Selected frontier / distinct decoy: %d / %d\n', ...
        e.selectedFrontierTrackId,e.bestIrrelevantFrontierTrackId);
    fprintf('Selected target relevance / tier : %.6f / %.0f\n', ...
        e.selectedTargetRelevance,e.selectedTier);
    fprintf('Selected / decoy information gain: %.6f / %.6f\n', ...
        e.selectedInformationGain,e.bestIrrelevantInformationGain);
    fprintf('Raw utility margin vs decoy      : %.6f\n', ...
        e.rawUtilityMarginVsBestIrrelevant);
    fprintf('Tier-priority margin vs decoy    : %.0f\n', ...
        e.priorityTierMarginVsBestIrrelevant);
else
    fprintf('No selected coupled decision contained a distinct competing frontier.\n');
end
fprintf('Competing frontier count         : %d\n',s.maxCompetingFrontierCount);
fprintf('Distinct irrelevant frontiers   : %d\n', ...
    s.maxDistinctIrrelevantFrontierCount);
disp(gate);
end

function e=firstCompetingSelectedEvidence(history)
e=[];
for i=1:numel(history)
    item=history{i};
    if ~isstruct(item)||~isfield(item,'evidence')||isempty(item.evidence)
        continue
    end
    q=item.evidence;
    if q.hasCompetition&&q.selected&&q.selectedTargetRelevant&& ...
            q.distinctIrrelevantFrontierCount>=1&& ...
            isfinite(q.bestIrrelevantUtility)
        e=q;
        return
    end
end
end

function tf=literalTruthWorldMatches(world,expected)
tf=isstruct(world)&&isfield(world,'staticRects5')&& ...
    size(world.staticRects5,1)==size(expected,1)&&size(world.staticRects5,2)>=4;
if ~tf,return,end
actual=sortrows(double(world.staticRects5(:,1:4)),[1 2 3 4]);
expected=sortrows(double(expected),[1 2 3 4]);
tf=max(abs(actual(:)-expected(:)))<1e-10;
end

function out=localTernary(condition,a,b)
if condition,out=a;else,out=b;end
end

function report = validate_S2_4_E_competing_corridors_multiseed(seeds)
% VALIDATE_S2_4_E_COMPETING_CORRIDORS_MULTISEED Layered physical robustness gate.
% S2_4_E_LAYERED_V5
if nargin<1||isempty(seeds),seeds=0:9;end
policy=test_S2_4_E_competing_decision_contract();
geometry=test_S2_4_E_literal_corridor_geometry_contract();
assert(geometry.pass,'S2_4:LiteralCorridorGeometryFailed','Literal corridor geometry failed.');
seeds=seeds(:);n=numel(seeds);runs=cell(n,1);Seed=seeds;
GoalReached=zeros(n,1);Requests=zeros(n,1);Executed=zeros(n,1);CompetingDecisions=zeros(n,1);
MaxFrontiers=zeros(n,1);IrrelevantCandidates=zeros(n,1);DistinctIrrelevantFrontiers=zeros(n,1);
CleanDecoyFound=zeros(n,1);TargetSelections=zeros(n,1);IrrelevantSelections=zeros(n,1);
SelectedTier=nan(n,1);SelectedTargetRelevance=nan(n,1);SelectedInformationGain=nan(n,1);
CleanDecoyInformationGain=nan(n,1);PhysicalDecoyMoreInformativeObserved=zeros(n,1);
TierPriorityMargin=nan(n,1);UnsafeExecution=zeros(n,1);UnknownCommitments=zeros(n,1);
Collisions=zeros(n,1);GeofenceViolations=zeros(n,1);MissionPass=zeros(n,1);TruthWorldMatch=zeros(n,1);
scenario=scenario_S2_4('active_competing_corridors');expectedTruthRects=double(scenario.validationGeometry.obstacles_xywh_m);
for k=1:n
    seed=seeds(k);fprintf('\n=== S2.4-E LAYERED CORRIDORS %d/%d | SEED %d ===\n',k,n,seed);
    runs{k}=run_S2_4_coupled(seed,'active_competing_corridors',false,false);s=runs{k}.summary;
    e=firstCompetingSelectedEvidence(runs{k}.maps.explorationDecisionHistory);
    GoalReached(k)=double(s.goalReached);Requests(k)=double(s.explorationRequestCount);Executed(k)=double(s.explorationExecutedCount);
    CompetingDecisions(k)=double(s.competingDecisionCount);MaxFrontiers(k)=double(s.maxCompetingFrontierCount);
    IrrelevantCandidates(k)=double(s.maxIrrelevantCandidateCount);DistinctIrrelevantFrontiers(k)=double(s.maxDistinctIrrelevantFrontierCount);
    TargetSelections(k)=double(s.targetRelevantSelectionCount);IrrelevantSelections(k)=double(s.irrelevantSelectionCount);
    if ~isempty(e)
        SelectedTier(k)=double(e.selectedTier);SelectedTargetRelevance(k)=double(e.selectedTargetRelevance);SelectedInformationGain(k)=double(e.selectedInformationGain);
        [found,q]=cleanTier3Decoy(e);CleanDecoyFound(k)=double(found);
        if found
            CleanDecoyInformationGain(k)=double(q.informationGain);
            PhysicalDecoyMoreInformativeObserved(k)=double(q.informationGain>e.selectedInformationGain);
        end
        TierPriorityMargin(k)=double(e.priorityTierMarginVsBestIrrelevant);
    end
    UnsafeExecution(k)=double(s.unsafeViewpointExecutionCount);UnknownCommitments(k)=double(s.unknownCommitmentCount);
    Collisions(k)=double(s.collisionCount);GeofenceViolations(k)=double(s.geofenceViolationCount);MissionPass(k)=double(s.pass);
    TruthWorldMatch(k)=double(literalTruthWorldMatches(runs{k}.maps.truthWorldFinal,expectedTruthRects));
end
summaryTable=table(Seed,GoalReached,Requests,Executed,CompetingDecisions,MaxFrontiers,IrrelevantCandidates, ...
    DistinctIrrelevantFrontiers,CleanDecoyFound,TargetSelections,IrrelevantSelections,SelectedTier, ...
    SelectedTargetRelevance,SelectedInformationGain,CleanDecoyInformationGain, ...
    PhysicalDecoyMoreInformativeObserved,TierPriorityMargin,UnsafeExecution,UnknownCommitments, ...
    Collisions,GeofenceViolations,TruthWorldMatch,MissionPass);disp(summaryTable);
decisionPass=all(CompetingDecisions>=1)&&all(MaxFrontiers>=2)&&all(IrrelevantCandidates>=1)&& ...
    all(DistinctIrrelevantFrontiers>=1)&&all(CleanDecoyFound==1)&&all(TargetSelections>=1)&& ...
    all(IrrelevantSelections==0)&&all(SelectedTier==1)&&all(SelectedTargetRelevance>0)&& ...
    all(isfinite(TierPriorityMargin))&&all(TierPriorityMargin>0);
hardSafetyPass=all(UnsafeExecution==0)&&all(UnknownCommitments==0)&&all(Collisions==0)&& ...
    all(GeofenceViolations==0)&&all(TruthWorldMatch==1);
missionCompletionPass=all(MissionPass==1)&&all(GoalReached==1)&&all(Requests>=1)&&all(Executed>=1);
report=struct();report.schema='S2_4_E_LAYERED_MULTI_SEED_V5';report.scenario='active_competing_corridors';
cfg=init_S2_4_E_config();report.version=cfg.version;report.seeds=seeds;report.runs=runs;report.summaryTable=summaryTable;
report.adversarialPolicyContractPass=policy.pass;report.literalGeometryPass=geometry.pass;report.decisionPass=decisionPass;
report.hardSafetyPass=hardSafetyPass;report.missionCompletionPass=missionCompletionPass;
report.physicalDecoyMoreInformativeObservedCount=sum(PhysicalDecoyMoreInformativeObserved);
report.pass=policy.pass&&geometry.pass&&decisionPass&&hardSafetyPass&&missionCompletionPass;
fprintf('\nS2.4-E LAYERED COMPETING CORRIDORS MULTISEED: %d/%d MISSION PASS | policy=%d | %s\n', ...
    sum(MissionPass),n,policy.pass,localTernary(report.pass,'PASS','FAIL'));
assert(report.pass,'S2_4:CompetingCorridorsGateFailed','One or more layered decision, mission or safety checks failed.');
end

function e=firstCompetingSelectedEvidence(history)
e=[];for i=1:numel(history),item=history{i};if ~isstruct(item)||~isfield(item,'evidence')||isempty(item.evidence),continue,end
q=item.evidence;if q.hasCompetition&&q.selected&&q.selectedTargetRelevant&&q.distinctIrrelevantFrontierCount>=1,e=q;return,end,end
end
function [found,best]=cleanTier3Decoy(e)
found=false;best=struct();if ~isfield(e,'candidateDiagnostics')||isempty(e.candidateDiagnostics),return,end
D=e.candidateDiagnostics;sf=uint64(e.selectedFrontierTrackId);bestIG=-inf;
for i=1:numel(D),q=D(i);if uint64(q.frontierTrackId)==sf,continue,end
if double(q.informationGain)<=0||double(q.targetRelevance)>0||double(q.tier)~=3,continue,end
rr=normaliseReasons(q.rejectionReasons);if numel(rr)~=1||rr(1)~="IRRELEVANT_EXPLORATION",continue,end
if double(q.informationGain)>bestIG,bestIG=double(q.informationGain);best=q;found=true;end,end
end
function rr=normaliseReasons(x)
if isempty(x),rr=strings(0,1);return,end
if ischar(x),rr=string({x});elseif isstring(x),rr=x(:);elseif iscell(x),rr=string(x(:));else,rr=string.empty(0,1);end
rr=rr(strlength(rr)>0);
end
function tf=literalTruthWorldMatches(world,expected)
tf=isstruct(world)&&isfield(world,'staticRects5')&&size(world.staticRects5,2)>=4;if ~tf,return,end
actual=double(world.staticRects5(:,1:4));expected=double(expected);tol=1e-10;
for i=1:size(expected,1),if ~any(max(abs(actual-expected(i,:)),[],2)<tol),tf=false;return,end,end
tf=true;
end
function out=localTernary(condition,a,b)
if condition,out=a;else,out=b;end
end

function report = test_S2_4_E_competing_decision_contract()
% TEST_S2_4_E_COMPETING_DECISION_CONTRACT Controlled adversarial policy gate.
% S2_4_E_LAYERED_V6
% Both target and decoy pass physical/safety feasibility. The decoy has
% strictly higher raw information gain AND raw utility, but is Tier-3
% unrelated and rejected only by IRRELEVANT_EXPLORATION. Tier-1 target wins.
base=struct( ...
    'candidateId',uint64(0),'frontierTrackId',uint64(0), ...
    'targetRelevance',0,'informationGain',0,'utility',0,'tier',uint8(3), ...
    'accepted',false,'rejectionReasons',{{}});

target=base;
target.candidateId=uint64(11);target.frontierTrackId=uint64(101);
target.targetRelevance=4;target.informationGain=5;target.utility=0.32;
target.tier=uint8(1);target.accepted=true;target.rejectionReasons={};

decoy=base;
decoy.candidateId=uint64(12);decoy.frontierTrackId=uint64(202);
decoy.targetRelevance=0;decoy.informationGain=12;decoy.utility=0.47;
decoy.tier=uint8(3);decoy.accepted=false;
decoy.rejectionReasons={'IRRELEVANT_EXPLORATION'};

frontiers(1)=struct('trackId',uint64(101),'centroid',[18 27]);
frontiers(2)=struct('trackId',uint64(202),'centroid',[42 27]);
e=summarize_viewpoint_decision_S2_4([target decoy],target,frontiers);

checks=struct();
checks.twoFrontiers=e.frontierCount==2;
checks.competition=e.hasCompetition;
checks.targetSafetyFeasible=isempty(target.rejectionReasons);
checks.decoySafetyFeasible=numel(decoy.rejectionReasons)==1&& ...
    strcmp(decoy.rejectionReasons{1},'IRRELEVANT_EXPLORATION');
checks.decoyPolicyOnly=isequal(decoy.rejectionReasons,{'IRRELEVANT_EXPLORATION'});
checks.decoyHasMoreInformation=decoy.informationGain>target.informationGain;
checks.decoyHasHigherRawUtility=decoy.utility>target.utility;
checks.targetTier1=target.tier==1&&target.targetRelevance>0;
checks.decoyTier3=decoy.tier==3&&decoy.targetRelevance==0;
checks.targetSelected=e.selectedCandidateId==target.candidateId&&e.selectedTargetRelevant;
checks.noIrrelevantSelection=~e.irrelevantSelected;
checks.distinctDecoy=e.distinctIrrelevantFrontierCount==1;
checks.mostInformativeDecoyRecorded=e.mostInformativeIrrelevantCandidateId==decoy.candidateId;
checks.rawUtilityDecoyBeatsTarget=e.rawUtilityMarginVsBestIrrelevant<0;
checks.tierPriorityWins=e.priorityTierMarginVsBestIrrelevant>0;
vals=struct2cell(checks);
report=struct('schema','S2_4_E_ADVERSARIAL_POLICY_CONTRACT_V6', ...
    'checks',checks,'evidence',e,'pass',all(cellfun(@logical,vals)));

fprintf('\nS2.4-E CONTROLLED ADVERSARIAL POLICY CONTRACT: %s\n', ...
    localTernary(report.pass,'PASS','FAIL'));
fprintf('target IG / utility: %.3f / %.3f\n',target.informationGain,target.utility);
fprintf('decoy  IG / utility: %.3f / %.3f\n',decoy.informationGain,decoy.utility);
disp(checks);
end

function out=localTernary(condition,a,b)
if condition,out=a;else,out=b;end
end

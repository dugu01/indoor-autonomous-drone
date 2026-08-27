function report = test_S2_4_E_competing_decision_contract()
% TEST_S2_4_E_COMPETING_DECISION_CONTRACT Tier-1 target choice beats decoy.
base=struct( ...
    'candidateId',uint64(0), ...
    'frontierTrackId',uint64(0), ...
    'targetRelevance',0, ...
    'informationGain',0, ...
    'utility',0, ...
    'tier',uint8(3), ...
    'accepted',false);

target=base;
target.candidateId=uint64(11);
target.frontierTrackId=uint64(101);
target.targetRelevance=4;
target.informationGain=5;
target.utility=0.32;
target.tier=uint8(1);
target.accepted=true;

decoy=base;
decoy.candidateId=uint64(12);
decoy.frontierTrackId=uint64(202);
decoy.targetRelevance=0;
decoy.informationGain=8;
decoy.utility=0.47;
decoy.tier=uint8(3);
decoy.accepted=false;

frontiers(1)=struct('trackId',uint64(101),'centroid',[18 27]);
frontiers(2)=struct('trackId',uint64(202),'centroid',[42 27]);
e=summarize_viewpoint_decision_S2_4([target decoy],target,frontiers);

checks=struct();
checks.twoFrontiers=e.frontierCount==2;
checks.competition=e.hasCompetition;
checks.distinctDecoyFrontier=e.distinctIrrelevantFrontierCount==1;
checks.targetSelected=e.selectedTargetRelevant;
checks.noIrrelevantSelection=~e.irrelevantSelected;
checks.decoyHasMoreInformation=e.bestIrrelevantInformationGain> ...
    e.selectedInformationGain;
checks.rawUtilityCanBeLower=e.rawUtilityMarginVsBestIrrelevant<0;
checks.tierPriorityWins=e.priorityTierMarginVsBestIrrelevant>0;
vals=struct2cell(checks);
report=struct('checks',checks,'evidence',e, ...
    'pass',all(cellfun(@logical,vals)));

fprintf('\nS2.4-E COMPETING-DECISION CONTRACT: %s\n', ...
    localTernary(report.pass,'PASS','FAIL'));
disp(checks);
end

function out=localTernary(condition,a,b)
if condition,out=a;else,out=b;end
end

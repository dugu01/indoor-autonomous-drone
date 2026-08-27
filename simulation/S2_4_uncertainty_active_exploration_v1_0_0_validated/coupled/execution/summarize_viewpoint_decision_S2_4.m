function evidence = summarize_viewpoint_decision_S2_4(candidates,selected,frontiers,grid)
% SUMMARIZE_VIEWPOINT_DECISION_S2_4 Compact evidence for target-vs-decoy choice.
%
% Diagnostic evidence only. This function reads autonomy-visible candidate,
% frontier and execution-grid outputs. It does not use scenario truth and it
% does not produce any motion command.
arguments
    candidates struct
    selected
    frontiers struct
    grid (1,1) struct = struct()
end

evidence=emptyEvidence();
if isempty(candidates)
    return
end

frontierIds=uint64([candidates.frontierTrackId]);
targetRelevance=double([candidates.targetRelevance]);
informationGain=double([candidates.informationGain]);
utility=double([candidates.utility]);
tier=double([candidates.tier]);
accepted=logical([candidates.accepted]);

hasInformation=informationGain>0;
targetMask=targetRelevance>0;
irrelevantMask=hasInformation&~targetMask;

evidence.candidateCount=numel(candidates);
evidence.frontierCount=numel(unique(frontierIds));
evidence.acceptedCount=nnz(accepted);
evidence.targetRelevantCandidateCount=nnz(targetMask);
evidence.acceptedTargetRelevantCount=nnz(targetMask&accepted);
evidence.irrelevantCandidateCount=nnz(irrelevantMask);
evidence.acceptedIrrelevantCount=nnz(irrelevantMask&accepted);
targetFrontierIds=unique(frontierIds(targetMask));
irrelevantFrontierIds=unique(frontierIds(irrelevantMask));
distinctIrrelevantIds=setdiff(irrelevantFrontierIds,targetFrontierIds);
evidence.targetRelevantFrontierCount=numel(targetFrontierIds);
evidence.irrelevantFrontierCount=numel(irrelevantFrontierIds);
evidence.distinctIrrelevantFrontierCount=numel(distinctIrrelevantIds);
evidence.hasCompetition=evidence.targetRelevantFrontierCount>=1&& ...
    evidence.distinctIrrelevantFrontierCount>=1;

% Preserve a compact per-candidate table so Scenario 2 can be diagnosed from
% the saved MAT file without re-running or guessing which viewpoint produced
% the information-gain numbers.
evidence.candidateDiagnostics=buildCandidateDiagnostics(candidates,grid);

% A decoy must belong to a distinct frontier family, not merely be a
% zero-relevance viewpoint generated from the same frontier as the selected
% target-relevant family.
distinctIrrelevantMask=irrelevantMask&ismember(frontierIds,distinctIrrelevantIds);
if any(distinctIrrelevantMask)
    idx=find(distinctIrrelevantMask);

    [~,localUtility]=max(utility(idx));
    bestUtility=idx(localUtility);
    evidence.bestIrrelevantCandidateId=uint64(candidates(bestUtility).candidateId);
    evidence.bestIrrelevantFrontierTrackId=uint64(candidates(bestUtility).frontierTrackId);
    evidence.bestIrrelevantUtility=utility(bestUtility);
    evidence.bestIrrelevantInformationGain=informationGain(bestUtility);
    evidence.bestIrrelevantTier=tier(bestUtility);
    evidence.bestIrrelevantAccepted=accepted(bestUtility);
    evidence.bestIrrelevantCell=double(fieldOr(candidates(bestUtility),'cell',[nan nan]));
    evidence.bestIrrelevantXY=cellToXY(grid,evidence.bestIrrelevantCell);
    evidence.bestIrrelevantYaw=double(fieldOr(candidates(bestUtility),'yaw',nan));
    evidence.bestIrrelevantPathCellCount=size(fieldOr(candidates(bestUtility),'path',zeros(0,2)),1);
    evidence.bestIrrelevantRejectionReasons=fieldOr(candidates(bestUtility),'rejectionReasons',{});

    [~,localInformation]=max(informationGain(idx));
    mostInfo=idx(localInformation);
    evidence.mostInformativeIrrelevantCandidateId=uint64(candidates(mostInfo).candidateId);
    evidence.mostInformativeIrrelevantFrontierTrackId=uint64(candidates(mostInfo).frontierTrackId);
    evidence.mostInformativeIrrelevantInformationGain=informationGain(mostInfo);
    evidence.mostInformativeIrrelevantUtility=utility(mostInfo);
    evidence.mostInformativeIrrelevantTier=tier(mostInfo);
    evidence.mostInformativeIrrelevantAccepted=accepted(mostInfo);
    evidence.mostInformativeIrrelevantCell=double(fieldOr(candidates(mostInfo),'cell',[nan nan]));
    evidence.mostInformativeIrrelevantXY=cellToXY(grid,evidence.mostInformativeIrrelevantCell);
    evidence.mostInformativeIrrelevantYaw=double(fieldOr(candidates(mostInfo),'yaw',nan));
    evidence.mostInformativeIrrelevantPathCellCount=size(fieldOr(candidates(mostInfo),'path',zeros(0,2)),1);
    evidence.mostInformativeIrrelevantRejectionReasons=fieldOr(candidates(mostInfo),'rejectionReasons',{});
end

if isempty(selected)
    addFrontierCentroids();
    return
end

evidence.selected=true;
evidence.selectedCandidateId=uint64(selected.candidateId);
evidence.selectedFrontierTrackId=uint64(selected.frontierTrackId);
evidence.selectedTier=double(selected.tier);
evidence.selectedUtility=double(selected.utility);
evidence.selectedTargetRelevance=double(selected.targetRelevance);
evidence.selectedInformationGain=double(selected.informationGain);
evidence.selectedAccepted=logical(fieldOr(selected,'accepted',false));
evidence.selectedCell=double(fieldOr(selected,'cell',[nan nan]));
evidence.selectedXY=cellToXY(grid,evidence.selectedCell);
evidence.selectedYaw=double(fieldOr(selected,'yaw',nan));
evidence.selectedPathCellCount=size(fieldOr(selected,'path',zeros(0,2)),1);
evidence.selectedRejectionReasons=fieldOr(selected,'rejectionReasons',{});
evidence.selectedTargetRelevant=selected.targetRelevance>0&&double(selected.tier)==1;
evidence.irrelevantSelected=selected.targetRelevance<=0;

if isfinite(evidence.bestIrrelevantUtility)
    evidence.rawUtilityMarginVsBestIrrelevant= ...
        evidence.selectedUtility-evidence.bestIrrelevantUtility;
    evidence.priorityTierMarginVsBestIrrelevant= ...
        evidence.bestIrrelevantTier-evidence.selectedTier;
end
addFrontierCentroids();

    function addFrontierCentroids()
        if isempty(frontiers),return,end
        ids=uint64([frontiers.trackId]);
        k=find(ids==evidence.selectedFrontierTrackId,1);
        if ~isempty(k)
            evidence.selectedFrontierCentroid=double(frontiers(k).centroid);
            evidence.selectedFrontierCentroidXY=cellToXY(grid,evidence.selectedFrontierCentroid);
        end
        k=find(ids==evidence.mostInformativeIrrelevantFrontierTrackId,1);
        if ~isempty(k)
            evidence.mostInformativeIrrelevantFrontierCentroid=double(frontiers(k).centroid);
            evidence.mostInformativeIrrelevantFrontierCentroidXY= ...
                cellToXY(grid,evidence.mostInformativeIrrelevantFrontierCentroid);
        end
    end
end

function d=buildCandidateDiagnostics(candidates,grid)
template=struct('candidateId',uint64(0),'frontierTrackId',uint64(0), ...
    'cell',[nan nan],'xy',[nan nan],'yaw',nan,'tier',inf,'utility',-inf, ...
    'targetRelevance',0,'informationGain',0,'accepted',false, ...
    'pathCellCount',0,'rejectionReasons',{{}});
d=repmat(template,1,numel(candidates));
for i=1:numel(candidates)
    q=candidates(i);
    d(i).candidateId=uint64(q.candidateId);
    d(i).frontierTrackId=uint64(q.frontierTrackId);
    d(i).cell=double(fieldOr(q,'cell',[nan nan]));
    d(i).xy=cellToXY(grid,d(i).cell);
    d(i).yaw=double(fieldOr(q,'yaw',nan));
    d(i).tier=double(q.tier);
    d(i).utility=double(q.utility);
    d(i).targetRelevance=double(q.targetRelevance);
    d(i).informationGain=double(q.informationGain);
    d(i).accepted=logical(q.accepted);
    d(i).pathCellCount=size(fieldOr(q,'path',zeros(0,2)),1);
    d(i).rejectionReasons=fieldOr(q,'rejectionReasons',{});
end
end

function xy=cellToXY(grid,cellRC)
xy=[nan nan];
if ~isstruct(grid)||~all(isfield(grid,{'xs','ys','nx','ny'})),return,end
if numel(cellRC)~=2||any(~isfinite(cellRC)),return,end
r=round(double(cellRC(1)));c=round(double(cellRC(2)));
if r<1||c<1||r>double(grid.ny)||c>double(grid.nx),return,end
xy=[double(grid.xs(c)) double(grid.ys(r))];
end

function v=fieldOr(s,name,defaultValue)
if isstruct(s)&&isfield(s,name)
    v=s.(name);
else
    v=defaultValue;
end
end

function e=emptyEvidence()
e=struct( ...
    'candidateCount',0, ...
    'frontierCount',0, ...
    'acceptedCount',0, ...
    'targetRelevantCandidateCount',0, ...
    'acceptedTargetRelevantCount',0, ...
    'irrelevantCandidateCount',0, ...
    'acceptedIrrelevantCount',0, ...
    'targetRelevantFrontierCount',0, ...
    'irrelevantFrontierCount',0, ...
    'distinctIrrelevantFrontierCount',0, ...
    'hasCompetition',false, ...
    'candidateDiagnostics',struct([]), ...
    'selected',false, ...
    'selectedCandidateId',uint64(0), ...
    'selectedFrontierTrackId',uint64(0), ...
    'selectedFrontierCentroid',[nan nan], ...
    'selectedFrontierCentroidXY',[nan nan], ...
    'selectedTier',inf, ...
    'selectedUtility',-inf, ...
    'selectedTargetRelevance',0, ...
    'selectedInformationGain',0, ...
    'selectedAccepted',false, ...
    'selectedCell',[nan nan], ...
    'selectedXY',[nan nan], ...
    'selectedYaw',nan, ...
    'selectedPathCellCount',0, ...
    'selectedRejectionReasons',{{}}, ...
    'selectedTargetRelevant',false, ...
    'irrelevantSelected',false, ...
    'bestIrrelevantCandidateId',uint64(0), ...
    'bestIrrelevantFrontierTrackId',uint64(0), ...
    'bestIrrelevantUtility',-inf, ...
    'bestIrrelevantInformationGain',0, ...
    'bestIrrelevantTier',inf, ...
    'bestIrrelevantAccepted',false, ...
    'bestIrrelevantCell',[nan nan], ...
    'bestIrrelevantXY',[nan nan], ...
    'bestIrrelevantYaw',nan, ...
    'bestIrrelevantPathCellCount',0, ...
    'bestIrrelevantRejectionReasons',{{}}, ...
    'mostInformativeIrrelevantCandidateId',uint64(0), ...
    'mostInformativeIrrelevantFrontierTrackId',uint64(0), ...
    'mostInformativeIrrelevantInformationGain',0, ...
    'mostInformativeIrrelevantUtility',-inf, ...
    'mostInformativeIrrelevantTier',inf, ...
    'mostInformativeIrrelevantAccepted',false, ...
    'mostInformativeIrrelevantCell',[nan nan], ...
    'mostInformativeIrrelevantXY',[nan nan], ...
    'mostInformativeIrrelevantYaw',nan, ...
    'mostInformativeIrrelevantPathCellCount',0, ...
    'mostInformativeIrrelevantRejectionReasons',{{}}, ...
    'mostInformativeIrrelevantFrontierCentroid',[nan nan], ...
    'mostInformativeIrrelevantFrontierCentroidXY',[nan nan], ...
    'rawUtilityMarginVsBestIrrelevant',nan, ...
    'priorityTierMarginVsBestIrrelevant',nan);
end

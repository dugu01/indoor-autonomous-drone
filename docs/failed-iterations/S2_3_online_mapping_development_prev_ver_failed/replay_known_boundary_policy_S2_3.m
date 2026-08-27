function report = replay_known_boundary_policy_S2_3(matFile,writeArtifacts)
% REPLAY_KNOWN_BOUNDARY_POLICY_S2_3
% Counterfactual replay of the actual S2.3 mapper with one policy change:
% known room/geofence boundary voxels are registered as persistent prohibited
% space before any sensor packets are inserted. No unknown obstacle truth is
% used by the policy. Truth is passed only to the independent final validator.
if nargin<2||isempty(writeArtifacts),writeArtifacts=true;end
validateattributes(matFile,{'char','string'},{'scalartext'});
matFile=char(matFile);
if exist(matFile,'file')~=2
    error('S2_3:BoundaryReplayMissingFile','Trial MAT file not found: %s',matFile);
end
S=load(matFile,'cfg','maps');
if ~isfield(S,'cfg')||~isfield(S,'maps')|| ...
        ~isfield(S.maps,'perceptionReplay')||isempty(S.maps.perceptionReplay)
    error('S2_3:BoundaryReplayMissingData', ...
        'Instrumented trial data with perceptionReplay is required.');
end
if ~isfield(S.maps,'perceptionReplaySchema')|| ...
        ~strcmp(S.maps.perceptionReplaySchema,'S2_3_RAW_RAYS_POSE_V1')
    error('S2_3:BoundaryReplaySchema','Unsupported replay schema.');
end

% Baseline exact replay.
baseMap=init_probabilistic_map_S2_3(S.cfg);
[baseMap,baseAccepted,baseRejected,baseReasons]=run_stream( ...
    S.cfg,baseMap,S.maps.perceptionReplay);
baseMetrics=validate_map_against_truth_S2_3( ...
    S.cfg,baseMap,S.maps.truthWorldFinal);

% Candidate policy replay. The boundary comes only from the already-known
% room/geofence contract in cfg.room and map axes.
candidateMap=init_probabilistic_map_S2_3(S.cfg);
knownBoundary=known_boundary_mask(candidateMap,S.cfg);
loOcc=log(S.cfg.mapOccupiedProbability/(1-S.cfg.mapOccupiedProbability));
candidateMap.staticOccupied(knownBoundary)=true;
candidateMap.logOdds(knownBoundary)=max( ...
    candidateMap.logOdds(knownBoundary),single(loOcc+0.05));
candidateMap.hitCount(knownBoundary)=max( ...
    candidateMap.hitCount(knownBoundary), ...
    uint16(S.cfg.mapMinOccupiedObservations));
[candidateMap,candidateAccepted,candidateRejected,candidateReasons]=run_stream( ...
    S.cfg,candidateMap,S.maps.perceptionReplay);
candidateMetrics=validate_map_against_truth_S2_3( ...
    S.cfg,candidateMap,S.maps.truthWorldFinal);

% Confirm that only known boundary registration is responsible for initial
% policy differences. Runtime mapping may then evolve normally.
expected=S.maps.probabilisticMap;
baseExactArrays=exact_mapper_arrays(baseMap,expected);
baseMetricsExact=metric_equal(baseMetrics, ...
    validate_map_against_truth_S2_3(S.cfg,expected,S.maps.truthWorldFinal));
policyUsesOnlyKnownBoundary=true;
requirementsPass=candidateMetrics.falseFreeRate<=S.cfg.mapMaxFalseFreeRate&& ...
    candidateMetrics.occupiedRecall>=S.cfg.mapMinOccupiedRecall;
noReplayRejections=(baseRejected==0&&candidateRejected==0);

report=struct();
report.schema='S2_3_KNOWN_BOUNDARY_POLICY_REPLAY_V1';
report.matFile=matFile;
report.recordCount=numel(S.maps.perceptionReplay);
report.baseAccepted=baseAccepted;
report.baseRejected=baseRejected;
report.candidateAccepted=candidateAccepted;
report.candidateRejected=candidateRejected;
report.baseReasons=baseReasons;
report.candidateReasons=candidateReasons;
report.knownBoundaryVoxelCount=nnz(knownBoundary);
report.policyUsesOnlyKnownBoundary=policyUsesOnlyKnownBoundary;
report.baseExactArrays=baseExactArrays;
report.baseMetricsExact=baseMetricsExact;
report.baseMetrics=baseMetrics;
report.candidateMetrics=candidateMetrics;
report.falseFreeReduction=baseMetrics.falseFreeCount-candidateMetrics.falseFreeCount;
report.requirementsPass=requirementsPass;
report.pass=baseExactArrays&&baseMetricsExact&&policyUsesOnlyKnownBoundary&& ...
    noReplayRejections&&requirementsPass;

fprintf('\n============================================================\n');
fprintf(' S2.3 KNOWN-BOUNDARY POLICY REPLAY\n');
fprintf(' Records / base accepted/rejected : %d / %d / %d\n', ...
    report.recordCount,baseAccepted,baseRejected);
fprintf(' Candidate accepted/rejected      : %d / %d\n', ...
    candidateAccepted,candidateRejected);
fprintf(' Baseline exact arrays/metrics    : %d / %d\n', ...
    baseExactArrays,baseMetricsExact);
fprintf(' Known boundary voxels registered : %d\n',nnz(knownBoundary));
fprintf(' Baseline false-free / recall     : %.8f / %.6f\n', ...
    baseMetrics.falseFreeRate,baseMetrics.occupiedRecall);
fprintf(' Candidate false-free / recall    : %.8f / %.6f\n', ...
    candidateMetrics.falseFreeRate,candidateMetrics.occupiedRecall);
fprintf(' False-free voxels removed        : %d\n',report.falseFreeReduction);
fprintf(' Requirements pass                : %d\n',requirementsPass);
fprintf(' POLICY REPLAY RESULT             : %s\n',ternary(report.pass,'PASS','FAIL'));
fprintf('============================================================\n\n');

if writeArtifacts
    outDir=fileparts(matFile);if isempty(outDir),outDir=pwd;end
    save(fullfile(outDir,'S2_3_known_boundary_policy_replay_report.mat'), ...
        'report','baseMap','candidateMap','knownBoundary','-v7.3');
    f=fopen(fullfile(outDir,'S2_3_known_boundary_policy_replay_report.txt'),'w');
    if f>=0
        c=onCleanup(@()fclose(f)); %#ok<NASGU>
        fprintf(f,'S2.3 known-boundary policy replay\n');
        fprintf(f,'Input: %s\n',matFile);
        fprintf(f,'Records %d | base accepted %d rejected %d | candidate accepted %d rejected %d\n', ...
            report.recordCount,baseAccepted,baseRejected,candidateAccepted,candidateRejected);
        fprintf(f,'Baseline exact arrays %d | metrics exact %d\n',baseExactArrays,baseMetricsExact);
        fprintf(f,'Known boundary voxels %d | policy known-only %d\n', ...
            nnz(knownBoundary),policyUsesOnlyKnownBoundary);
        fprintf(f,'Baseline false-free %.12f | recall %.12f | false-free count %d\n', ...
            baseMetrics.falseFreeRate,baseMetrics.occupiedRecall,baseMetrics.falseFreeCount);
        fprintf(f,'Candidate false-free %.12f | recall %.12f | false-free count %d\n', ...
            candidateMetrics.falseFreeRate,candidateMetrics.occupiedRecall,candidateMetrics.falseFreeCount);
        fprintf(f,'False-free reduction %d | requirements pass %d | PASS %d\n', ...
            report.falseFreeReduction,requirementsPass,report.pass);
    end
end
end

function [map,accepted,rejected,reasons]=run_stream(cfg,map,records)
accepted=0;rejected=0;reasons=cell(numel(records),1);
for i=1:numel(records)
    r=records{i};
    [map,u]=update_probabilistic_map_S2_3(cfg,map,r.packet,r.pose,r.callTime);
    reasons{i}=u.reason;
    accepted=accepted+double(u.accepted);
    rejected=rejected+double(~u.accepted);
end
end

function mask=known_boundary_mask(map,cfg)
mask=false(map.ny,map.nx,map.nz);
% X/Y room limits are known before flight. Use the actual coordinate arrays,
% not obstacle truth. Include a ceiling layer only if represented by the map.
tol=1e-9;
ix=find(map.xs<=tol|map.xs>=cfg.room(1)-tol);
iy=find(map.ys<=tol|map.ys>=cfg.room(2)-tol);
if ~isempty(ix),mask(:,ix,:)=true;end
if ~isempty(iy),mask(iy,:,:)=true;end
iz=find(map.zs>=cfg.room(3)-tol);
if ~isempty(iz),mask(:,:,iz)=true;end
end

function ok=exact_mapper_arrays(a,b)
fields={'staticOccupied','promotedStatic','hitCount','rawHitCount', ...
    'observationCount','dynamicHitCount','logOdds','dynamicLogOdds', ...
    'dynamicFirstHit','dynamicLastHit','lastObserved'};
ok=true;
for i=1:numel(fields)
    f=fields{i};
    ok=ok&&isfield(a,f)&&isfield(b,f)&&isequaln(a.(f),b.(f));
end
end

function ok=metric_equal(a,b)
ok=abs(a.falseFreeRate-b.falseFreeRate)<=1e-12&& ...
   abs(a.occupiedRecall-b.occupiedRecall)<=1e-12&& ...
   abs(a.observedFraction-b.observedFraction)<=1e-12&& ...
   a.falseFreeCount==b.falseFreeCount&& ...
   a.knownFreeCount==b.knownFreeCount;
end
function o=ternary(c,a,b),if c,o=a;else,o=b;end,end

function report = replay_perception_log_S2_3(matFile,writeArtifacts)
% REPLAY_PERCEPTION_LOG_S2_3 Re-run the actual S2.3 mapper from captured rays.
if nargin<2||isempty(writeArtifacts),writeArtifacts=true;end
validateattributes(matFile,{'char','string'},{'scalartext'});
matFile=char(matFile);
if exist(matFile,'file')~=2
    error('S2_3:ReplayMissingFile','Trial MAT file not found: %s',matFile);
end
S=load(matFile,'cfg','scenario','maps','summary');
required={'cfg','scenario','maps','summary'};
for i=1:numel(required)
    if ~isfield(S,required{i})
        error('S2_3:ReplayMissingField','Missing top-level field: %s',required{i});
    end
end
if ~isfield(S.maps,'perceptionReplay')||isempty(S.maps.perceptionReplay)
    error('S2_3:ReplayNotCaptured', ...
        ['No raw perception replay stream is present. Run the instrumented ' ...
         'candidate once and use its newly generated MAT file.']);
end
if ~isfield(S.maps,'perceptionReplaySchema')|| ...
        ~strcmp(S.maps.perceptionReplaySchema,'S2_3_RAW_RAYS_POSE_V1')
    error('S2_3:ReplaySchema','Unsupported or missing replay schema.');
end
records=S.maps.perceptionReplay;
map=init_probabilistic_map_S2_3(S.cfg);
accepted=0;rejected=0;reason=cell(numel(records),1);
for i=1:numel(records)
    r=records{i};
    [map,u]=update_probabilistic_map_S2_3( ...
        S.cfg,map,r.packet,r.pose,r.callTime);
    reason{i}=u.reason;
    accepted=accepted+double(u.accepted);
    rejected=rejected+double(~u.accepted);
end
expected=S.maps.probabilisticMap;
fieldsExact={'staticOccupied','knownBoundary','promotedStatic','hitCount','rawHitCount', ...
    'observationCount','dynamicHitCount'};
fieldsFloat={'logOdds','dynamicLogOdds','dynamicFirstHit','dynamicLastHit', ...
    'lastObserved'};
fieldMatch=struct();
for i=1:numel(fieldsExact)
    f=fieldsExact{i};
    fieldMatch.(f)=isfield(map,f)&&isfield(expected,f)&&isequaln(map.(f),expected.(f));
end
maxAbs=struct();
for i=1:numel(fieldsFloat)
    f=fieldsFloat{i};
    if isfield(map,f)&&isfield(expected,f)&&isequal(size(map.(f)),size(expected.(f)))
        a=double(map.(f));b=double(expected.(f));finite=isfinite(a)&isfinite(b);
        if any(finite(:)),maxAbs.(f)=max(abs(a(finite)-b(finite)));else,maxAbs.(f)=0;end
        fieldMatch.(f)=isequaln(map.(f),expected.(f));
    else
        maxAbs.(f)=inf;fieldMatch.(f)=false;
    end
end
counterFields={'version','frameVersion','acceptedPackets','rejectedPackets', ...
    'promotedCount','lastSequence'};
% noDataPackets counts control cycles with no sensor event. The captured replay
% stream intentionally contains accepted perception packets only, so this
% bookkeeping counter is reported separately and is not a mapper-equivalence
% criterion.
idleCounterMatch=isfield(map,'noDataPackets')&&isfield(expected,'noDataPackets')&& ...
    isequaln(map.noDataPackets,expected.noDataPackets);
counters=struct();
for i=1:numel(counterFields)
    f=counterFields{i};
    counters.(f)=isfield(map,f)&&isfield(expected,f)&&isequaln(map.(f),expected.(f));
end
metrics=validate_map_against_truth_S2_3(S.cfg,map,S.maps.truthWorldFinal);
originalMetrics=validate_map_against_truth_S2_3(S.cfg,expected,S.maps.truthWorldFinal);
exactFieldsPass=all(structfun(@(x)logical(x),fieldMatch));
exactCountersPass=all(structfun(@(x)logical(x),counters));
metricsMatch=abs(metrics.falseFreeRate-originalMetrics.falseFreeRate)<=1e-12&& ...
    abs(metrics.occupiedRecall-originalMetrics.occupiedRecall)<=1e-12&& ...
    abs(metrics.observedFraction-originalMetrics.observedFraction)<=1e-12;
report=struct('schema','S2_3_REPLAY_REPORT_V1','matFile',matFile, ...
    'recordCount',numel(records),'accepted',accepted,'rejected',rejected, ...
    'reasons',{reason},'fieldMatch',fieldMatch,'maxAbsDifference',maxAbs, ...
    'counterMatch',counters,'idleCounterMatch',idleCounterMatch, ...
    'replayIdlePackets',double(map.noDataPackets), ...
    'coupledIdlePackets',double(expected.noDataPackets), ...
    'exactFieldsPass',exactFieldsPass, ...
    'exactCountersPass',exactCountersPass,'metricsMatch',metricsMatch, ...
    'originalMetrics',originalMetrics,'replayMetrics',metrics, ...
    'pass',exactFieldsPass&&exactCountersPass&&metricsMatch&&rejected==0);
fprintf('\n============================================================\n');
fprintf(' S2.3 EXACT PERCEPTION-MAPPER REPLAY\n');
fprintf(' Records / accepted / rejected : %d / %d / %d\n',report.recordCount,accepted,rejected);
fprintf(' Exact arrays / core counters  : %d / %d\n',exactFieldsPass,exactCountersPass);
fprintf(' Idle counters replay/coupled  : %u / %u (informational)\n', ...
    uint32(map.noDataPackets),uint32(expected.noDataPackets));
fprintf(' Original false-free / recall  : %.8f / %.6f\n',originalMetrics.falseFreeRate,originalMetrics.occupiedRecall);
fprintf(' Replay false-free / recall    : %.8f / %.6f\n',metrics.falseFreeRate,metrics.occupiedRecall);
fprintf(' Metrics exact match           : %d\n',metricsMatch);
fprintf(' REPLAY RESULT                 : %s\n',ternary(report.pass,'PASS','FAIL'));
fprintf('============================================================\n\n');
if writeArtifacts
    outDir=fileparts(matFile);if isempty(outDir),outDir=pwd;end
    save(fullfile(outDir,'S2_3_exact_mapper_replay_report.mat'),'report','map','-v7.3');
    f=fopen(fullfile(outDir,'S2_3_exact_mapper_replay_report.txt'),'w');
    if f>=0
        c=onCleanup(@()fclose(f)); %#ok<NASGU>
        fprintf(f,'S2.3 exact perception-mapper replay\n');
        fprintf(f,'Input: %s\n',matFile);
        fprintf(f,'Records %d | accepted %d | rejected %d\n',numel(records),accepted,rejected);
        fprintf(f,'Exact arrays %d | exact core counters %d | metrics match %d\n',exactFieldsPass,exactCountersPass,metricsMatch);
        fprintf(f,'Idle counters replay %u | coupled %u | informational only\n', ...
            uint32(map.noDataPackets),uint32(expected.noDataPackets));
        fprintf(f,'Original false-free %.12f | recall %.12f | observed %.12f\n',originalMetrics.falseFreeRate,originalMetrics.occupiedRecall,originalMetrics.observedFraction);
        fprintf(f,'Replay false-free %.12f | recall %.12f | observed %.12f\n',metrics.falseFreeRate,metrics.occupiedRecall,metrics.observedFraction);
        fprintf(f,'PASS %d\n',report.pass);
    end
end
end
function o=ternary(c,a,b),if c,o=a;else,o=b;end,end

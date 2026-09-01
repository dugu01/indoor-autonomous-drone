function report = run_S2_4_AD_shadow_replay(trialMat,outputDir)
% RUN_S2_4_AD_SHADOW_REPLAY Cumulative S2.4 A-D offline/shadow replay.
% The function first invokes the exact inherited S2.3 mapper replay. It then
% computes non-authoritative uncertainty, frontier and viewpoint outputs. No
% position, velocity, yaw, mission-state or actuator command is produced.
arguments
    trialMat (1,:) char
    outputDir (1,:) char = ''
end
here=fileparts(mfilename('fullpath'));root=fileparts(here);parent=fullfile(root,'frozen_parent','S2_3_online_mapping_v1_0_0_validated');if exist(parent,'dir')~=7,error('S2_4:ParentMissing','Frozen S2.3 parent not found: %s',parent);end
addpath(parent);cleanup=onCleanup(@()rmpath(parent)); %#ok<NASGU>
if exist(trialMat,'file')~=2,error('S2_4:TrialMissing','S2.3 trial MAT not found: %s',trialMat);end
S=load(trialMat,'cfg','scenario','maps','summary','log');required={'cfg','scenario','maps','summary','log'};for i=1:numel(required),if ~isfield(S,required{i}),error('S2_4:TrialSchema','Missing %s.',required{i});end,end
if isempty(outputDir),outputDir=fullfile(fileparts(trialMat),'S2_4_AD_shadow');end;if exist(outputDir,'dir')~=7,mkdir(outputDir);end
exactReplay=replay_perception_log_S2_3(trialMat,false);if ~exactReplay.pass,error('S2_4:InheritedReplay','Inherited exact mapper replay failed.');end
c=init_S2_4_AD_config(S.cfg);runtimeInflation=double(S.maps.finalGrid.inflationRadius);if abs(runtimeInflation-0.602)>1e-9,error('S2_4:GeometryContract','Expected final S2.3 effective inflation 0.602 m; found %.12g.',runtimeInflation);end;c.effectiveInflation_m=runtimeInflation;
records=S.maps.perceptionReplay;map=init_probabilistic_map_S2_3(S.cfg);u=init_uncertainty_sidecar_S2_4(map);snapshotTimes=double(S.log.mapSnapshotTimes(:));uSnapshots=cell(numel(snapshotTimes),1);nextSnapshot=1;
for i=1:numel(records)
    r=records{i};[map,update]=update_probabilistic_map_S2_3(S.cfg,map,r.packet,r.pose,r.callTime);u=update_uncertainty_sidecar_S2_4(c,u,map,r,update);
    while nextSnapshot<=numel(snapshotTimes)&&double(r.callTime)>=snapshotTimes(nextSnapshot)-1e-9
        uSnapshots{nextSnapshot}=projectUncertainty(u,map,S.cfg.altitudeNominal_m);nextSnapshot=nextSnapshot+1;
    end
end
while nextSnapshot<=numel(snapshotTimes),uSnapshots{nextSnapshot}=projectUncertainty(u,map,S.cfg.altitudeNominal_m);nextSnapshot=nextSnapshot+1;end
mapperFields={'logOdds','observationCount','hitCount','staticOccupied','knownBoundary','rawHitCount','lastObserved','dynamicLogOdds','dynamicHitCount','dynamicFirstHit','dynamicLastHit','promotedStatic'};mapperExact=true;for i=1:numel(mapperFields),f=mapperFields{i};mapperExact=mapperExact&&isequaln(map.(f),S.maps.probabilisticMap.(f));end;if ~mapperExact,error('S2_4:MapperMutation','S2.4 replay changed inherited S2.3 mapper arrays.');end
frontierState=struct();shadowLog=cell(numel(S.log.mapSnapshots),1);unsafeAccepted=0;maxTracks=0;for k=1:numel(S.log.mapSnapshots)
    snap=S.log.mapSnapshots{k};grid=snapshotGrid(snap,S.maps.finalGrid,S.cfg,c.effectiveInflation_m,uSnapshots{k});[~,idx]=min(abs(double(S.log.t(:))-double(snap.time)));
    if size(S.log.estP,2)==3
        startXY=double(S.log.estP(idx,1:2));
    elseif size(S.log.estP,1)==3
        startXY=double(S.log.estP(1:2,idx)).';
    else
        error('S2_4:EstimateShape','Expected estP as N-by-3 or 3-by-N; found %s.',mat2str(size(S.log.estP)));
    end
    targetXY=double(S.scenario.goal(1:2));targetXY=targetXY(:).';
    [frontierState,frontiers,fdiag]=extract_frontiers_incremental_S2_4(c,frontierState,grid);[candidates,selected,vdiag]=generate_safe_viewpoints_S2_4(c,grid,frontiers,startXY,targetXY,uSnapshots{k});
    for j=find([candidates.accepted]),q=candidates(j);idxp=sub2ind(size(grid.knownFree),q.path(:,1),q.path(:,2));if ~grid.knownFree(q.cell(1),q.cell(2))||grid.occ(q.cell(1),q.cell(2))||any(~grid.knownFree(idxp)|grid.occ(idxp)),unsafeAccepted=unsafeAccepted+1;end,end
    recommendation=struct('action','NONE','frontierTrackId',uint64(0),'candidateId',uint64(0),'positionXY',[nan nan],'yaw',nan,'utility',-inf);if ~isempty(selected),recommendation.action=selected.action;recommendation.frontierTrackId=selected.frontierTrackId;recommendation.candidateId=selected.candidateId;recommendation.positionXY=[grid.xs(selected.cell(2)) grid.ys(selected.cell(1))];recommendation.yaw=selected.yaw;recommendation.utility=selected.utility;end
    shadowLog{k}=struct('time',snap.time,'mapVersion',snap.version,'frontierDiagnostics',fdiag,'viewpointDiagnostics',vdiag,'frontiers',frontiers,'candidates',candidates,'shadowRecommendation',recommendation,'commandIssued',false);maxTracks=max(maxTracks,double(frontierState.nextTrackId-1));
end
report=struct();report.schema='S2_4_AD_SHADOW_REPORT_V1';report.trialMat=trialMat;report.scenario=S.summary.scenario;report.parentExactMapperReplay=exactReplay.pass;report.mapperArraysExact=mapperExact;report.snapshotCount=numel(shadowLog);report.uncertaintyDeterministicDigest=sidecarDigest(u);report.frontierTrackCount=maxTracks;report.acceptedCandidateCount=sum(cellfun(@(x)x.viewpointDiagnostics.acceptedCount,shadowLog));report.frontierViewpointDigest=shadowDigest(shadowLog);report.unsafeAcceptedCandidates=unsafeAccepted;report.commandIssued=false;report.truthAccessCount=double(u.truthAccessCount);report.sourcePackets=double(u.acceptedSourcePackets);report.pass=exactReplay.pass&&mapperExact&&unsafeAccepted==0&&~report.commandIssued&&report.truthAccessCount==0;
save(fullfile(outputDir,'S2_4_AD_shadow_report.mat'),'report','shadowLog','u','c','-v7.3');writeReport(fullfile(outputDir,'S2_4_AD_shadow_report.txt'),report);fprintf('\nS2.4 A-D SHADOW REPLAY: %s\n',ternary(report.pass,'PASS','FAIL'));
end

function grid=snapshotGrid(snap,finalGrid,parentCfg,inflation,u)
staticRaw=logical(snap.staticOccupied);dynamicRaw=logical(snap.dynamicOccupied);rawOccupied=staticRaw|dynamicRaw;
staticInflated=inflateMetric(staticRaw,inflation,parentCfg.mapResolutionXY_m);dynamicInflated=inflateMetric(dynamicRaw,inflation,parentCfg.mapResolutionXY_m);hardInflated=staticInflated|dynamicInflated;
unknown=logical(snap.unknown);unknownInflated=inflateMetric(unknown,inflation,parentCfg.mapResolutionXY_m);occ=hardInflated|unknownInflated;
% Preserve the inherited raw known-free/unknown boundary for frontier
% extraction.  Unknown inflation is an execution constraint only; applying it
% to the frontier predicate would erase every valid frontier by construction.
knownRaw=logical(snap.knownFree)&~rawOccupied&~unknown;
grid=struct('occ',occ,'navigationBlocked',occ,'knownFree',knownRaw,'unknown',unknown,'unknownInflated',unknownInflated, ...
    'rawOccupied',rawOccupied,'staticOccupiedRaw',staticRaw,'dynamicOccupiedRaw',dynamicRaw, ...
    'staticOccupied',staticInflated,'dynamicOccupied',dynamicInflated,'mapVersion',snap.version,'timestamp',double(snap.time), ...
    'resolution',double(finalGrid.resolution),'xs',double(finalGrid.xs(:)).','ys',double(finalGrid.ys(:)).', ...
    'nx',numel(finalGrid.xs),'ny',numel(finalGrid.ys),'lastObservedXY',u.lastObservedXY);
end
function out=inflateMetric(in,radius,res),out=in;maxOffset=floor(radius/res+1e-12);[yy,xx]=find(in);for k=1:numel(xx),for dy=-maxOffset:maxOffset,for dx=-maxOffset:maxOffset,if hypot(dx*res,dy*res)<=radius+1e-12,y=yy(k)+dy;x=xx(k)+dx;if y>=1&&y<=size(in,1)&&x>=1&&x<=size(in,2),out(y,x)=true;end,end,end,end,end
end
function p=projectUncertainty(u,map,altitude),[~,iz]=min(abs(map.zs-altitude));p=struct('entropy',u.entropy(:,:,iz),'observationCount',u.observationCount(:,:,iz),'lastObservedTime',u.lastObservedTime(:,:,iz),'observationAge',u.observationAge(:,:,iz),'sourceMask',u.sourceMask(:,:,iz),'sourceQuality',u.sourceQuality(:,:,iz),'staticConfidence',u.staticConfidence(:,:,iz),'dynamicConfidence',u.dynamicConfidence(:,:,iz),'staleFree',u.staleFree,'lastObservedXY',max(double(map.lastObserved),[],3));end
function d=sidecarDigest(u),v=[sum(double(u.entropy(:))) sum(double(u.observationCount(:))) sum(double(u.lidarHitCount(:))) sum(double(u.lidarFreeCount(:))) sum(double(u.depthHitCount(:))) sum(double(u.depthFreeCount(:))) sum(double(u.staticConfidence(:))) sum(double(u.dynamicConfidence(:))) nnz(u.staleFree) double(u.sidecarVersion)];d=sprintf('%.17g_',v);end
function d=shadowDigest(logs)
parts=cell(numel(logs),1);
for k=1:numel(logs)
    e=logs{k};v=[double(e.mapVersion),double(e.frontierDiagnostics.frontierCellCount), ...
        double(e.frontierDiagnostics.clusterCount),double(numel(e.frontiers)), ...
        double(e.viewpointDiagnostics.candidateCount), ...
        double(e.viewpointDiagnostics.acceptedCount),double(e.viewpointDiagnostics.selectedCandidateId)];
    c=e.candidates;
    for j=1:numel(c)
        v=[v,double(c(j).candidateId),double(c(j).frontierTrackId),double(c(j).tier), ... %#ok<AGROW>
            double(c(j).accepted),double(c(j).informationGain),double(c(j).targetRelevance), ...
            double(c(j).travelCost),double(c(j).dynamicRisk),double(c(j).utility), ...
            double(numel(c(j).rejectionReasons))];
    end
    parts{k}=sprintf('%.17g,',v);
end
d=strjoin(parts,'|');
end
function writeReport(path,r),f=fopen(path,'w');if f<0,error('S2_4:ReportWrite','Cannot write %s',path);end;c=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'S2.4 A-D offline/shadow replay\n');fprintf(f,'Scenario: %s\n',r.scenario);fprintf(f,'Inherited exact mapper replay: %d\n',r.parentExactMapperReplay);fprintf(f,'Mapper arrays exact: %d\n',r.mapperArraysExact);fprintf(f,'Snapshots: %d\n',r.snapshotCount);fprintf(f,'Frontier tracks: %d\n',r.frontierTrackCount);fprintf(f,'Accepted candidates: %d\n',r.acceptedCandidateCount);fprintf(f,'Frontier/viewpoint digest: %s\n',r.frontierViewpointDigest);fprintf(f,'Unsafe accepted candidates: %d\n',r.unsafeAcceptedCandidates);fprintf(f,'Truth accesses: %d\n',r.truthAccessCount);fprintf(f,'Commands issued: %d\n',r.commandIssued);fprintf(f,'RESULT: %s\n',ternary(r.pass,'PASS','FAIL'));end
function o=ternary(c,a,b),if c,o=a;else,o=b;end,end

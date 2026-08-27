function [map,update] = update_probabilistic_map_S2_3(cfg,map,packet,est,t)
% UPDATE_PROBABILISTIC_MAP_S2_3 Insert time-aligned rays using estimated pose.
% No environment-truth input is accepted by this function.
update=struct('accepted',false,'staticChanged',false,'dynamicChanged',false, ...
    'changedVoxelCount',0,'packetAge_s',inf,'reason','no_data');
if isempty(packet)||~isfield(packet,'timestamp')||~isfinite(packet.timestamp)
    map.rejectedPackets=map.rejectedPackets+1;return;
end
if isfield(packet,'sequence')&&uint64(packet.sequence)<=map.lastSequence
    update.reason='duplicate_packet';return;
end
hasLidarEvent=isfield(packet,'hasLidarScan')&&logical(packet.hasLidarScan);
hasDepthEvent=isfield(packet,'hasDepthRays')&&logical(packet.hasDepthRays);
if ~hasLidarEvent&&~hasDepthEvent
    update.reason='no_sensor_event';map.noDataPackets=map.noDataPackets+1;return;
end
update.packetAge_s=t-packet.timestamp;
if update.packetAge_s< -cfg.dt||update.packetAge_s>cfg.mapMaxPacketAge_s
    update.reason='stale_packet';map.rejectedPackets=map.rejectedPackets+1;return;
end
if ~isfield(est,'p')||~isfield(est,'q')||any(~isfinite(est.p))||any(~isfinite(est.q))
    update.reason='invalid_pose';map.rejectedPackets=map.rejectedPackets+1;return;
end
if isfield(est,'xySigma_m')&&est.xySigma_m>cfg.mapPoseCovarianceReject_m
    update.reason='pose_uncertainty';map.rejectedPackets=map.rejectedPackets+1;return;
end
if isfield(map,'staticOccupied')
    oldStaticClass=map.staticOccupied;
else
    oldStaticClass=map.logOdds>=logit(cfg.mapOccupiedProbability)& ...
        map.hitCount>=cfg.mapMinOccupiedObservations;
end
oldFreeClass=(map.logOdds<=logit(cfg.mapFreeProbability))& ...
    (map.observationCount>=cfg.mapMinFreeObservations);
oldDynamic=map.dynamicLogOdds>=logit(cfg.mapDynamicOccupiedProbability);

% Continuous decay of temporary occupancy. Static occupancy is never
% deleted by this decay path.
dtDecay=max(0,t-map.lastUpdateTime);
map.dynamicLogOdds=max(0,map.dynamicLogOdds-cfg.mapDynamicDecayPerSecond*dtDecay);
R=q2R_S2_2(est.q);acceptedRay=false;
if hasLidarEvent
    origin=est.p+R*cfg.r_B_lidar;
    for i=1:numel(packet.lidarRanges)
        dB=cfg.R_B_lidar*[cos(packet.lidarAngles(i));sin(packet.lidarAngles(i));0];
        [map,ok]=insert_ray(map,cfg,origin,R*dB,packet.lidarRanges(i),packet.lidarHits(i),t);
        acceptedRay=acceptedRay||ok;
    end
    map.lastLidarTime=packet.timestamp;
end
if hasDepthEvent
    origin=est.p+R*cfg.r_B_depth;
    for i=1:numel(packet.depthRanges)
        dB=cfg.R_B_depth*packet.depthDirections(i,:).';
        [map,ok]=insert_ray(map,cfg,origin,R*dB,packet.depthRanges(i),packet.depthHits(i),t);
        acceptedRay=acceptedRay||ok;
    end
    map.lastDepthTime=packet.timestamp;
end
if ~acceptedRay
    update.reason='no_valid_rays';map.rejectedPackets=map.rejectedPackets+1;return;
end

% Promote a persistent temporary voxel once. Clamp the static evidence above
% the occupied threshold instead of adding one hit to a saturated-free cell.
% This prevents repeated promotion cycles while preserving the configured
% persistence and hit-count qualification.
promote=(map.dynamicHitCount>=cfg.mapDynamicPromotionHits)& ...
    isfinite(map.dynamicFirstHit)&(t-double(map.dynamicFirstHit)>=cfg.mapDynamicPromotionTime_s)& ...
    (map.dynamicLogOdds>=logit(cfg.mapDynamicOccupiedProbability));
indices=find(promote);
loPromote=logit(cfg.mapOccupiedProbability)+single(0.05);
for i=1:numel(indices)
    idx=indices(i);
    map.logOdds(idx)=max(map.logOdds(idx),loPromote);
    map.hitCount(idx)=max(map.hitCount(idx),uint16(cfg.mapMinOccupiedObservations));
    map.staticOccupied(idx)=true;
    if ~map.promotedStatic(idx)
        map.promotedStatic(idx)=true;
        map.promotedCount=map.promotedCount+1;
    end
    map.dynamicLogOdds(idx)=0;map.dynamicHitCount(idx)=0;
    map.dynamicFirstHit(idx)=nan;map.dynamicLastHit(idx)=-inf;
end
newStaticClass=map.staticOccupied;
newFreeClass=(map.logOdds<=logit(cfg.mapFreeProbability))& ...
    (map.observationCount>=cfg.mapMinFreeObservations);
newDynamic=map.dynamicLogOdds>=logit(cfg.mapDynamicOccupiedProbability);
update.staticChanged=any(oldStaticClass(:)~=newStaticClass(:))|| ...
    any(oldFreeClass(:)~=newFreeClass(:));
update.dynamicChanged=any(oldDynamic(:)~=newDynamic(:));
update.changedVoxelCount=nnz(oldStaticClass~=newStaticClass)+ ...
    nnz(oldFreeClass~=newFreeClass)+nnz(oldDynamic~=newDynamic);
if update.staticChanged||update.dynamicChanged||map.acceptedPackets==0
    map.version=map.version+1;map.lastChangeTime=t;
end
map.lastPacketTime=packet.timestamp;
if isfield(packet,'sequence'),map.lastSequence=uint64(packet.sequence);end
map.lastUpdateTime=t;map.acceptedPackets=map.acceptedPackets+1;
update.accepted=true;update.reason='accepted';
end

function [map,accepted]=insert_ray(map,cfg,origin,direction,range,hit,t)
accepted=false;direction=direction(:);n=norm(direction);
if n<1e-12||~isfinite(range),return;end
direction=direction/n;range=max(0,double(range));

% Determine the occupied endpoint first. The endpoint voxel is explicitly
% excluded from all free updates. The previous half-voxel geometric offset
% was insufficient because nearest-node quantisation still mapped the final
% free sample into the hit voxel for about half of generic grid phases.
hitIndex=0;
if hit
    endpoint=origin(:)+range*direction;
    [ixh,iyh,izh,insideHit]=point_index(map,endpoint);
    if insideHit,hitIndex=sub2ind([map.ny map.nx map.nz],iyh,ixh,izh);end
end

step=0.5*min(map.resolutionXY,map.resolutionZ);
freeIndices=zeros(max(1,ceil(range/step)+2),1,'uint32');nFree=0;
for s=0:step:range
    p=origin(:)+s*direction;
    [ix,iy,iz,inside]=point_index(map,p);
    if ~inside,continue;end
    idx=sub2ind([map.ny map.nx map.nz],iy,ix,iz);
    if hitIndex>0&&idx==hitIndex,continue;end
    nFree=nFree+1;freeIndices(nFree)=uint32(idx);
end
if nFree>0
    freeIndices=unique(freeIndices(1:nFree),'stable');
    for j=1:numel(freeIndices)
        idx=double(freeIndices(j));
        % A persistent static voxel is not erased by an unrelated ray that
        % happens to traverse the same quantised cell. Static-map clearing
        % requires a dedicated consistency mechanism; it is deliberately not
        % inferred from isolated free-ray evidence in S2.3.
        if ~map.staticOccupied(idx)
            map.logOdds(idx)=max(cfg.mapLogOddsMin,map.logOdds(idx)+cfg.mapLogOddsMiss);
        end
        map.observationCount(idx)=inc_u16(map.observationCount(idx));
        map.lastObserved(idx)=single(t);accepted=true;
        map.dynamicLogOdds(idx)=max(0,map.dynamicLogOdds(idx)-0.75*cfg.mapDynamicHit);
    end
end

if hitIndex>0
    idx=hitIndex;
    map.rawHitCount(idx)=inc_u16(map.rawHitCount(idx));
    priorFree=map.logOdds(idx)<=logit(cfg.mapFreeProbability)&& ...
        map.observationCount(idx)>=cfg.mapMinFreeObservations;
    if map.staticOccupied(idx)
        map.logOdds(idx)=min(cfg.mapLogOddsMax,map.logOdds(idx)+cfg.mapLogOddsHit);
        map.hitCount(idx)=inc_u16(map.hitCount(idx));
    elseif priorFree&&~map.promotedStatic(idx)
        map.dynamicLogOdds(idx)=min(cfg.mapLogOddsMax,map.dynamicLogOdds(idx)+cfg.mapDynamicHit);
        map.dynamicHitCount(idx)=inc_u16(map.dynamicHitCount(idx));
        if ~isfinite(map.dynamicFirstHit(idx)),map.dynamicFirstHit(idx)=single(t);end
        map.dynamicLastHit(idx)=single(t);
    else
        map.logOdds(idx)=min(cfg.mapLogOddsMax,map.logOdds(idx)+cfg.mapLogOddsHit);
        map.hitCount(idx)=inc_u16(map.hitCount(idx));
        if map.hitCount(idx)>=cfg.mapMinOccupiedObservations&& ...
                map.logOdds(idx)>=logit(cfg.mapOccupiedProbability)
            map.staticOccupied(idx)=true;
        end
    end
    map.observationCount(idx)=inc_u16(map.observationCount(idx));
    map.lastObserved(idx)=single(t);accepted=true;
end
end

function [ix,iy,iz,inside]=point_index(map,p)
ix=round(p(1)/map.resolutionXY)+1;iy=round(p(2)/map.resolutionXY)+1;
iz=round((p(3)-map.zs(1))/map.resolutionZ)+1;
inside=ix>=1&&ix<=map.nx&&iy>=1&&iy<=map.ny&&iz>=1&&iz<=map.nz;
end
function y=logit(p),y=log(p/max(1-p,eps));end
function v=inc_u16(v)
if v<intmax('uint16'),v=v+1;end
end

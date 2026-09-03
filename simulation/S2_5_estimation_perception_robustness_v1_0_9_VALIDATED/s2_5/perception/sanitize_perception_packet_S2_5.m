function [packet,stats] = sanitize_perception_packet_S2_5(cfg,map,packet,est)
% SANITIZE_PERCEPTION_PACKET_S2_5 Fault-agnostic obstacle-ray integrity gate.
%
% The frozen S2.3 mapper remains unchanged. This S2.5 pre-map overlay enforces
% two physically/algorithmically justified rules before a packet is supplied
% to that mapper:
%   Rule 1: one occupied endpoint update per map voxel per perception packet;
%   Rule 2: a claimed hit may not lie behind an already persistent static voxel on
%      the same ray (opaque-static occlusion consistency).
%
% No environment truth, scenario fault label, or fault timing is used.
% Duplicate/occlusion-rejected hit rays are removed rather than converted to
% no-hit rays so they cannot create false free evidence through an obstacle.

stats=struct('duplicateHitRayCount',0,'occlusionRejectedHitRayCount',0, ...
    'keptHitRayCount',0,'inputHitRayCount',0);
if isempty(packet)||~isstruct(packet)||~isfield(est,'p')||~isfield(est,'q')
    return;
end
if ~isfield(map,'ny')||~isfield(map,'nx')||~isfield(map,'nz')|| ...
        ~isfield(map,'staticOccupied')
    return;
end
R=q2R_S2_2(est.q);
seen=false(map.ny,map.nx,map.nz);

% LiDAR first; depth shares the same endpoint-voxel set so a coincident
% LiDAR/depth endpoint in one packet still contributes only one occupied
% update to that voxel.
if isfield(packet,'hasLidarScan')&&logical(packet.hasLidarScan)&& ...
        isfield(packet,'lidarRanges')&&~isempty(packet.lidarRanges)
    n=numel(packet.lidarRanges);keep=true(n,1);
    origin=est.p+R*cfg.r_B_lidar;
    for i=1:n
        if ~logical(packet.lidarHits(i)),continue;end
        stats.inputHitRayCount=stats.inputHitRayCount+1;
        dB=cfg.R_B_lidar*[cos(packet.lidarAngles(i));sin(packet.lidarAngles(i));0];
        [ok,idx]=accept_hit_local(map,origin,R*dB,double(packet.lidarRanges(i)),seen);
        if ~ok
            keep(i)=false;
            if idx==0,stats.occlusionRejectedHitRayCount=stats.occlusionRejectedHitRayCount+1;
            else,stats.duplicateHitRayCount=stats.duplicateHitRayCount+1;end
        else
            seen(idx)=true;stats.keptHitRayCount=stats.keptHitRayCount+1;
        end
    end
    packet.lidarAngles=packet.lidarAngles(keep);
    packet.lidarRanges=packet.lidarRanges(keep);
    packet.lidarHits=packet.lidarHits(keep);
end

if isfield(packet,'hasDepthRays')&&logical(packet.hasDepthRays)&& ...
        isfield(packet,'depthRanges')&&~isempty(packet.depthRanges)
    n=numel(packet.depthRanges);keep=true(n,1);
    origin=est.p+R*cfg.r_B_depth;
    for i=1:n
        if ~logical(packet.depthHits(i)),continue;end
        stats.inputHitRayCount=stats.inputHitRayCount+1;
        dB=cfg.R_B_depth*packet.depthDirections(i,:).';
        [ok,idx]=accept_hit_local(map,origin,R*dB,double(packet.depthRanges(i)),seen);
        if ~ok
            keep(i)=false;
            if idx==0,stats.occlusionRejectedHitRayCount=stats.occlusionRejectedHitRayCount+1;
            else,stats.duplicateHitRayCount=stats.duplicateHitRayCount+1;end
        else
            seen(idx)=true;stats.keptHitRayCount=stats.keptHitRayCount+1;
        end
    end
    packet.depthDirections=packet.depthDirections(keep,:);
    packet.depthRanges=packet.depthRanges(keep);
    packet.depthHits=packet.depthHits(keep);
end
end

function [accepted,endpointIndex] = accept_hit_local(map,origin,direction,range,seen)
accepted=false;endpointIndex=0;
direction=direction(:);n=norm(direction);
if n<1e-12||~isfinite(range)||range<0,return;end
direction=direction/n;
endpoint=origin(:)+range*direction;
[ixh,iyh,izh,inside]=point_index_local(map,endpoint);
if ~inside
    % Preserve inherited behavior for out-of-map endpoints: this integrity
    % helper does not classify them as occupied and simply removes the hit ray.
    return;
end
endpointIndex=sub2ind([map.ny map.nx map.nz],iyh,ixh,izh);
if seen(endpointIndex),accepted=false;return;end

% An opaque persistent-static voxel more than one map cell before the claimed
% endpoint makes the claimed farther hit physically inconsistent. The one-cell
% margin is grid-resolution derived, not a tuned fault threshold.
step=0.5*min(map.resolutionXY,map.resolutionZ);
margin=max(map.resolutionXY,map.resolutionZ);
limit=max(0,range-margin);
for s=0:step:limit
    p=origin(:)+s*direction;
    [ix,iy,iz,in]=point_index_local(map,p);
    if ~in,continue;end
    idx=sub2ind([map.ny map.nx map.nz],iy,ix,iz);
    if idx~=endpointIndex&&map.staticOccupied(idx)
        endpointIndex=0;accepted=false;return;
    end
end
accepted=true;
end

function [ix,iy,iz,inside] = point_index_local(map,p)
ix=round(p(1)/map.resolutionXY)+1;
iy=round(p(2)/map.resolutionXY)+1;
iz=round((p(3)-map.zs(1))/map.resolutionZ)+1;
inside=ix>=1&&ix<=map.nx&&iy>=1&&iy<=map.ny&&iz>=1&&iz<=map.nz;
end

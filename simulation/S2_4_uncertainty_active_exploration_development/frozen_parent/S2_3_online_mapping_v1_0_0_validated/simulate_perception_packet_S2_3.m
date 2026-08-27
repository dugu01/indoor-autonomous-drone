function [packet,model,world] = simulate_perception_packet_S2_3(cfg,scenario,truth,model,t,k,context)
% SIMULATE_PERCEPTION_PACKET_S2_3 Raw LiDAR/depth ray simulation.
% Environment truth is converted into finite-range, noisy, occluded rays.
% The returned packet contains no obstacle rectangles or truth object IDs.
if nargin<7,context=struct();end
world=truth_world_S2_3(cfg,scenario,t,context);
model.sequence=model.sequence+1;
packet=struct('timestamp',t,'sequence',model.sequence,'frameId','B', ...
    'hasLidarScan',false,'hasDepthRays',false,'lidarAngles',[], ...
    'lidarRanges',[],'lidarHits',[],'depthDirections',[], ...
    'depthRanges',[],'depthHits',[],'sourceTruthFree',true);

perceptionDrop=inside_windows(t,scenario.obstacleSensorDropoutWindows);
lidarDrop=perceptionDrop||inside_windows(t,scenario.perceptionLidarDropoutWindows);
depthDrop=perceptionDrop||inside_windows(t,scenario.depthDropoutWindows);
R=q2R_S2_2(truth.q);

if mod(k-1,cfg.perceptionLidarPeriodSteps)==0&&~lidarDrop
    angles=linspace(-pi,pi,cfg.perceptionLidarRayCount+1);angles(end)=[];
    ranges=zeros(numel(angles),1);hits=false(numel(angles),1);
    origin=truth.p+R*cfg.r_B_lidar;
    for i=1:numel(angles)
        dB=cfg.R_B_lidar*[cos(angles(i));sin(angles(i));0];
        dW=R*dB;
        [ranges(i),hits(i)]=raycast_world_S2_3(cfg,world,origin,dW, ...
            cfg.perceptionLidarMinRange_m,cfg.perceptionLidarMaxRange_m);
    end
    ranges=ranges+cfg.perceptionLidarRangeSigma_m*randn(size(ranges));
    ranges=max(cfg.perceptionLidarMinRange_m,min(cfg.perceptionLidarMaxRange_m,ranges));
    packet.hasLidarScan=true;packet.lidarAngles=angles(:);packet.lidarRanges=ranges;packet.lidarHits=hits;
    model.lastLidarTime=t;model.lastAnyTime=t;
end

if mod(k-1,cfg.depthPeriodSteps)==0&&~depthDrop
    az=linspace(-0.5*cfg.depthHFOV_rad,0.5*cfg.depthHFOV_rad,cfg.depthAzimuthRayCount);
    el=linspace(-0.5*cfg.depthVFOV_rad,0.5*cfg.depthVFOV_rad,cfg.depthElevationRayCount);
    dirs=zeros(numel(az)*numel(el),3);ranges=zeros(size(dirs,1),1);hits=false(size(ranges));
    origin=truth.p+R*cfg.r_B_depth;idx=0;
    for ie=1:numel(el)
        for ia=1:numel(az)
            idx=idx+1;
            dirs(idx,:)=[cos(el(ie))*cos(az(ia)),cos(el(ie))*sin(az(ia)),sin(el(ie))];
            dW=R*(cfg.R_B_depth*dirs(idx,:).');
            [ranges(idx),hits(idx)]=raycast_world_S2_3(cfg,world,origin,dW, ...
                cfg.depthMinRange_m,cfg.depthMaxRange_m);
        end
    end
    ranges=ranges+cfg.depthRangeSigma_m*randn(size(ranges));
    ranges=max(cfg.depthMinRange_m,min(cfg.depthMaxRange_m,ranges));
    packet.hasDepthRays=true;packet.depthDirections=dirs;packet.depthRanges=ranges;packet.depthHits=hits;
    model.lastDepthTime=t;model.lastAnyTime=t;
end
packet.lastLidarTime=model.lastLidarTime;
packet.lastDepthTime=model.lastDepthTime;
packet.lastAnyTime=model.lastAnyTime;
end

function tf=inside_windows(t,w)
tf=false;
for i=1:size(w,1)
    if t>=w(i,1)&&t<=w(i,2),tf=true;return;end
end
end

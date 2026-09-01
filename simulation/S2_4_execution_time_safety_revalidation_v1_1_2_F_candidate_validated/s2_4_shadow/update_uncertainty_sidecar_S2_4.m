function u = update_uncertainty_sidecar_S2_4(c,u,map,record,update)
% UPDATE_UNCERTAINTY_SIDECAR_S2_4 Update entropy, age, source and confidence.
% Occupancy is read from the authoritative S2.3 map and is never written here.
arguments
    c (1,1) struct
    u (1,1) struct
    map (1,1) struct
    record (1,1) struct
    update (1,1) struct
end
if ~update.accepted
    u.rejectedSourcePackets = u.rejectedSourcePackets + uint32(1);
    return
end
p = 1 ./ (1 + exp(-double(map.logOdds)));
epsP = eps('double');
H = -(p.*log2(max(p,epsP)) + (1-p).*log2(max(1-p,epsP)));
u.entropy = single(min(max(H,0),1));
u.observationCount = map.observationCount;u.lastObservedTime = map.lastObserved;t = double(record.callTime);age = t-double(map.lastObserved);age(~isfinite(age)) = inf;u.observationAge = single(max(age,0));
% Source contribution is derived only from the captured onboard rays and pose.
if isfield(record.packet,'hasLidarScan') && record.packet.hasLidarScan
    u.acceptedSourcePackets(1) = u.acceptedSourcePackets(1)+uint32(1);
    u = insertLidarContribution(u,map,record.packet,record.pose);
end
if isfield(record.packet,'hasDepthRays') && record.packet.hasDepthRays
    u.acceptedSourcePackets(2) = u.acceptedSourcePackets(2)+uint32(1);
    u = insertDepthContribution(u,map,record.packet,record.pose);
end
lidarEvidence = double(u.lidarHitCount)+0.35*double(u.lidarFreeCount);depthEvidence = double(u.depthHitCount)+0.35*double(u.depthFreeCount);lidarReliability = c.source.lidarQuality*(1-exp(-lidarEvidence/6));depthReliability = c.source.depthQuality*(1-exp(-depthEvidence/6));u.sourceQuality = single(1-(1-lidarReliability).*(1-depthReliability));
staticEvidence = 1-exp(-double(map.observationCount)/c.source.staticSaturationCount);ageFactor = exp(-min(double(u.observationAge),1e6)/c.source.staticAgeConstant_s);dynamicP = 1./(1+exp(-double(map.dynamicLogOdds)));dynamicEvidence = 1-exp(-double(map.dynamicHitCount)/c.source.dynamicSaturationCount);u.dynamicConfidence = single(dynamicP.*dynamicEvidence);u.staticConfidence = single((1-double(u.entropy)).*staticEvidence.*double(u.sourceQuality).*ageFactor.*(1-double(u.dynamicConfidence)));u.staticConfidence(map.staticOccupied|map.knownBoundary|map.promotedStatic) = 1;
[~,iz] = min(abs(map.zs-c.nominalAltitude_m));
freeLogOdds = log(c.mapFreeProbability/(1-c.mapFreeProbability));
knownFree = map.logOdds(:,:,iz)<=freeLogOdds & ...
    map.observationCount(:,:,iz)>=c.mapMinFreeObservations;
u.staleFree = knownFree & (...
    double(u.observationAge(:,:,iz))>c.staleFreeAge_s | ...
    double(u.entropy(:,:,iz))>0.80 | ...
    double(u.sourceQuality(:,:,iz))<0.20);
u.authoritativeMapVersion = map.version;
u.sidecarVersion = u.sidecarVersion+uint32(1);
u.lastTimestamp = t;
end

function u=insertLidarContribution(u,map,packet,pose)
R=q2Rlocal(pose.q);origin=double(pose.p(:));angles=double(packet.lidarAngles(:));ranges=double(packet.lidarRanges(:));hits=logical(packet.lidarHits(:));n=min([numel(angles),numel(ranges),numel(hits)]);
for i=1:n
    if ~isfinite(ranges(i))||ranges(i)<=0,continue,end
    d=R*[cos(angles(i));sin(angles(i));0];endpoint=origin+ranges(i)*d;idx=rayIndices(map,origin,endpoint);
    if isempty(idx),continue,end
    freeIdx=idx;if hits(i),freeIdx=idx(1:end-1);u.lidarHitCount(idx(end))=satinc(u.lidarHitCount(idx(end)));u.sourceMask(idx(end))=bitor(u.sourceMask(idx(end)),uint8(1));end
    for j=1:numel(freeIdx),u.lidarFreeCount(freeIdx(j))=satinc(u.lidarFreeCount(freeIdx(j)));u.sourceMask(freeIdx(j))=bitor(u.sourceMask(freeIdx(j)),uint8(1));end
end
end

function u=insertDepthContribution(u,map,packet,pose)
R=q2Rlocal(pose.q);origin=double(pose.p(:));dirs=double(packet.depthDirections);if size(dirs,1)~=3,dirs=dirs.';end;ranges=double(packet.depthRanges(:));hits=logical(packet.depthHits(:));n=min([size(dirs,2),numel(ranges),numel(hits)]);
for i=1:n
    if ~isfinite(ranges(i))||ranges(i)<=0,continue,end
    d=R*dirs(:,i);d=d/max(norm(d),eps);endpoint=origin+ranges(i)*d;idx=rayIndices(map,origin,endpoint);if isempty(idx),continue,end
    freeIdx=idx;if hits(i),freeIdx=idx(1:end-1);u.depthHitCount(idx(end))=satinc(u.depthHitCount(idx(end)));u.sourceMask(idx(end))=bitor(u.sourceMask(idx(end)),uint8(2));end
    for j=1:numel(freeIdx),u.depthFreeCount(freeIdx(j))=satinc(u.depthFreeCount(freeIdx(j)));u.sourceMask(freeIdx(j))=bitor(u.sourceMask(freeIdx(j)),uint8(2));end
end
end

function idx=rayIndices(map,a,b)
d=b-a;steps=max(1,ceil(norm(d)/min(map.resolutionXY,map.resolutionZ)*2));idx=zeros(steps+1,1,'uint32');n=0;
for k=0:steps
    p=a+(k/steps)*d;ix=round((p(1)-map.xs(1))/map.resolutionXY)+1;iy=round((p(2)-map.ys(1))/map.resolutionXY)+1;iz=round((p(3)-map.zs(1))/map.resolutionZ)+1;
    if ix>=1&&ix<=map.nx&&iy>=1&&iy<=map.ny&&iz>=1&&iz<=map.nz
        q=uint32(sub2ind([map.ny map.nx map.nz],iy,ix,iz));if n==0||idx(n)~=q,n=n+1;idx(n)=q;end
    end
end
idx=idx(1:n);
end
function y=satinc(x),if x<intmax('uint16'),y=x+uint16(1);else,y=x;end,end
function R=q2Rlocal(q),q=double(q(:));q=q/max(norm(q),eps);w=q(1);x=q(2);y=q(3);z=q(4);R=[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w);2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w);2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)];end

function map = init_probabilistic_map_S2_3(cfg)
% INIT_PROBABILISTIC_MAP_S2_3 Layered 3-D log-odds representation.
xs=0:cfg.mapResolutionXY_m:cfg.room(1);
ys=0:cfg.mapResolutionXY_m:cfg.room(2);
zs=cfg.mapMinZ_m:cfg.mapResolutionZ_m:cfg.mapMaxZ_m;
ny=numel(ys);nx=numel(xs);nz=numel(zs);
map=struct();
map.xs=xs;map.ys=ys;map.zs=zs;map.nx=nx;map.ny=ny;map.nz=nz;
map.resolutionXY=cfg.mapResolutionXY_m;map.resolutionZ=cfg.mapResolutionZ_m;
map.logOdds=cfg.mapLogOddsPrior*ones(ny,nx,nz,'single');
map.observationCount=zeros(ny,nx,nz,'uint16');
map.hitCount=zeros(ny,nx,nz,'uint16');
% Conservative persistent-static latch. Once repeated endpoint evidence
% establishes a voxel as static occupied, ordinary free-ray traversal does
% not immediately erase it. This matches the S2.3 fail-closed mapping
% contract and prevents alternating noisy rays from turning walls or fixed
% obstacles into false free space.
map.staticOccupied=false(ny,nx,nz);
% The room/geofence limits are known before flight. Register their voxelised
% boundary as persistent prohibited space so the mapper, planner, and
% independent validator use one consistent boundary contract. This uses only
% cfg.room and map axes; no unknown obstacle truth is accessed.
map.knownBoundary=false(ny,nx,nz);
tol=1e-9;
ixBoundary=find(xs<=tol|xs>=cfg.room(1)-tol);
iyBoundary=find(ys<=tol|ys>=cfg.room(2)-tol);
izBoundary=find(zs>=cfg.room(3)-tol);
if ~isempty(ixBoundary),map.knownBoundary(:,ixBoundary,:)=true;end
if ~isempty(iyBoundary),map.knownBoundary(iyBoundary,:,:)=true;end
if ~isempty(izBoundary),map.knownBoundary(:,:,izBoundary)=true;end
loBoundary=log(cfg.mapOccupiedProbability/(1-cfg.mapOccupiedProbability));
map.staticOccupied(map.knownBoundary)=true;
map.logOdds(map.knownBoundary)=max( ...
    map.logOdds(map.knownBoundary),single(loBoundary+0.05));
map.hitCount(map.knownBoundary)=max( ...
    map.hitCount(map.knownBoundary),uint16(cfg.mapMinOccupiedObservations));
% rawHitCount records every physical ray endpoint, including endpoints first
% treated as temporary occupancy. It is used only by independent map recall
% validation and never by planning or mission decisions.
map.rawHitCount=zeros(ny,nx,nz,'uint16');
map.lastObserved=-inf(ny,nx,nz,'single');
% Temporary occupancy is maintained per voxel. The earlier 2-D candidate
% lost the endpoint height and repeatedly promoted the same XY cell at a
% fixed altitude, which was incompatible with depth-camera observations.
map.dynamicLogOdds=zeros(ny,nx,nz,'single');
map.dynamicHitCount=zeros(ny,nx,nz,'uint16');
map.dynamicFirstHit=nan(ny,nx,nz,'single');
map.dynamicLastHit=-inf(ny,nx,nz,'single');
map.promotedStatic=false(ny,nx,nz);
map.version=uint32(0);map.frameVersion=uint32(0);
map.lastPacketTime=-inf;map.lastLidarTime=-inf;map.lastDepthTime=-inf;map.lastSequence=uint64(0);
map.acceptedPackets=uint32(0);map.rejectedPackets=uint32(0);map.noDataPackets=uint32(0);
map.lastChangeTime=0;map.lastUpdateTime=0;map.promotedCount=uint32(0);map.truthAccessCount=uint32(0);
end

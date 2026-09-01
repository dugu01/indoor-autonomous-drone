function grid = project_map_to_planner_S2_3(cfg,map,inflationRadius,t)
% PROJECT_MAP_TO_PLANNER_S2_3 Convert layered belief to fail-closed 2-D grid.
if nargin<4,t=map.lastPacketTime;end
zLow=max(cfg.mapMinZ_m,cfg.altitudeNominal_m-cfg.collisionRadius);
zHigh=min(cfg.mapMaxZ_m,cfg.altitudeNominal_m+cfg.collisionRadius);
iz=find(map.zs>=zLow&map.zs<=zHigh);
if isempty(iz),[~,iz]=min(abs(map.zs-cfg.altitudeNominal_m));end
loOcc=log(cfg.mapOccupiedProbability/(1-cfg.mapOccupiedProbability));
loFree=log(cfg.mapFreeProbability/(1-cfg.mapFreeProbability));
if isfield(map,'staticOccupied')
    staticOcc=any(map.staticOccupied(:,:,iz),3);
else
    staticOcc=any(map.logOdds(:,:,iz)>=loOcc&map.hitCount(:,:,iz)>=cfg.mapMinOccupiedObservations,3);
end
[~,izNom]=min(abs(map.zs-cfg.altitudeNominal_m));
knownFree=map.logOdds(:,:,izNom)<=loFree&map.observationCount(:,:,izNom)>=cfg.mapMinFreeObservations;
knownFree=knownFree&~staticOcc;
if ndims(map.dynamicLogOdds)==3
    dynamicOcc=any(map.dynamicLogOdds(:,:,iz)>=log(cfg.mapDynamicOccupiedProbability/(1-cfg.mapDynamicOccupiedProbability)),3);
else
    % Backward-compatible read path for development replays made before the
    % temporary layer became height-aware.
    dynamicOcc=map.dynamicLogOdds>=log(cfg.mapDynamicOccupiedProbability/(1-cfg.mapDynamicOccupiedProbability));
end
unknown=~staticOcc&~knownFree;
inflatedStatic=inflate_binary_metric(staticOcc|dynamicOcc,inflationRadius,map.resolutionXY);
inflatedUnknown=inflate_binary_metric(unknown,inflationRadius,map.resolutionXY);
occ=inflatedStatic|(cfg.mapUnknownIsOccupied&inflatedUnknown);
ny=map.ny;nx=map.nx;
for iy=1:ny
    y=map.ys(iy);
    for ix=1:nx
        x=map.xs(ix);
        if x<inflationRadius||x>cfg.room(1)-inflationRadius|| ...
                y<inflationRadius||y>cfg.room(2)-inflationRadius
            occ(iy,ix)=true;
        end
    end
end
grid=struct('occ',occ,'xs',map.xs,'ys',map.ys,'nx',map.nx,'ny',map.ny, ...
    'resolution',map.resolutionXY,'obstacles',zeros(0,4), ...
    'inflationRadius',inflationRadius,'room',cfg.room,'knownFree',knownFree, ...
    'unknown',unknown,'unknownInflated',inflatedUnknown,'staticOccupied',staticOcc,'dynamicOccupied',dynamicOcc, ...
    'mapVersion',map.version,'frameVersion',map.frameVersion, ...
    'lastObservedXY',max(map.lastObserved,[],3),'timestamp',t);
end

function out=inflate_binary_metric(in,radius,resolution)
% Inflate grid-node centres by the requested physical radius. The previous
% ceil(radius/resolution) disk added up to one full grid cell of artificial
% clearance. With a 0.602 m radius on a 0.10 m grid it blocked nodes 0.70 m
% away, including the nominal S2.2/S2.3 goal. This metric-centre test retains
% the configured physical clearance without the extra quantisation penalty.
if radius<=0,out=in;return;end
[ny,nx]=size(in);out=in;[ys,xs]=find(in);
maxOffset=floor(radius/resolution+1e-12);
offsets=zeros((2*maxOffset+1)^2,2);nOffsets=0;
for dy=-maxOffset:maxOffset
    for dx=-maxOffset:maxOffset
        if hypot(dx*resolution,dy*resolution)<=radius+1e-12
            nOffsets=nOffsets+1;offsets(nOffsets,:)=[dy dx];
        end
    end
end
offsets=offsets(1:nOffsets,:);
for k=1:numel(xs)
    x0=xs(k);y0=ys(k);
    for j=1:nOffsets
        yy=y0+offsets(j,1);xx=x0+offsets(j,2);
        if yy>=1&&yy<=ny&&xx>=1&&xx<=nx,out(yy,xx)=true;end
    end
end
end

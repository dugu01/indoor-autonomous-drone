function [gridOut,mapOut,requestOut,meta] = apply_validation_fault_S2_4_F( ...
    fcfg,grid,mapState,request,currentXY,tNow)
% APPLY_VALIDATION_FAULT_S2_4_F Validation-only estimated-input fault hook.
%
% The caller invokes this only while an accepted exploration authority is
% actively in TRACK_OUTBOUND and after its configured progress/time trigger.
% This function never reads environment truth. It perturbs only the derived
% execution grid, estimated perception freshness metadata, or the accepted
% mission request.
arguments
    fcfg (1,1) struct
    grid (1,1) struct
    mapState (1,1) struct
    request (1,1) struct
    currentXY (1,2) double
    tNow (1,1) double
end
gridOut=grid;mapOut=mapState;requestOut=request;
name=upper(string(fieldOr(fcfg,'name','NONE')));
meta=struct('active',false,'applied',false,'name',char(name),'detail','NONE');
if any(name==["NONE","F1","F12","F15"]),return,end
meta.active=true;

switch name
    case {'F2','F13'}
        xy=futurePoint(request,currentXY,0.65);
        [gridOut,ok]=setCell(gridOut,xy,'occupied');
        meta.applied=ok;meta.detail='FUTURE_ROUTE_OCCUPIED';
    case 'F3'
        xy=futurePoint(request,currentXY,0.65);
        [gridOut,ok]=setCell(gridOut,xy,'unknown');
        meta.applied=ok;meta.detail='FUTURE_ROUTE_UNKNOWN';
    case 'F4'
        xy=fieldOr(request,'positionXY',[nan nan]);
        [gridOut,ok]=setCell(gridOut,xy,'occupied');
        meta.applied=ok;meta.detail='VIEWPOINT_OCCUPIED';
    case 'F5'
        xy=fieldOr(request,'positionXY',[nan nan]);
        if all(isfinite(xy)),xy=double(xy)+[grid.resolution 0];end
        [gridOut,ok]=setCell(gridOut,xy,'occupied');
        meta.applied=ok;meta.detail='LOCAL_HOLD_SUPPORT_OCCUPIED';
    case 'F6'
        if ~isempty(fieldnames(request))&&isfield(request,'retreatRouteXY')
            requestOut.retreatRouteXY=[nan nan;nan nan];
            meta.applied=true;meta.detail='RETREAT_AUTHORITY_CORRUPTED';
        end
    case 'F7'
        gridOut.mapVersion=uint32(double(grid.mapVersion)+double(fieldOr(fcfg,'versionOffset',1000000)));
        meta.applied=true;meta.detail='UNRELATED_MAP_VERSION_CHANGE';
    case 'F8'
        if ~isempty(fieldnames(request))&&isfield(request,'validUntil_s')
            requestOut.validUntil_s=min(double(request.validUntil_s),tNow-2e-9);
            meta.applied=true;meta.detail='REQUEST_FORCED_EXPIRED';
        end
    case 'F9'
        mapOut.lastLidarTime=-inf;mapOut.lastDepthTime=-inf;
        meta.applied=true;meta.detail='PERCEPTION_FRESHNESS_REMOVED';
    case 'F10'
        % Block a future route cell and isolate the original retreat goal by
        % blocking its neighboring cells, not the start/current cell itself.
        % This removes both forward and retreat authority without fabricating
        % an immediate unsafe reference at the vehicle position.
        xy=futurePoint(request,currentXY,0.70);
        [gridOut,ok1]=setCell(gridOut,xy,'occupied');
        startXY=fieldOr(request,'startXY',[nan nan]);
        [gridOut,ok2]=sealStartNeighborhood(gridOut,startXY,currentXY);
        meta.applied=ok1&&ok2;meta.detail='FORWARD_AND_RETREAT_UNAVAILABLE';
    case 'F11'
        % Change a known-free cell in the already traversed region but offset
        % it laterally from the stored/remaining route. This tests that a
        % behind-only map update does not falsely invalidate future authority.
        xy=behindSidePoint(gridOut,request,currentXY);
        [gridOut,ok]=setCell(gridOut,xy,'occupied');
        meta.applied=ok;meta.detail='TRAVERSED_REGION_CHANGE_BEHIND';
    case 'F14'
        xy=fieldOr(request,'positionXY',[nan nan]);
        [gridOut,ok]=setCell(gridOut,xy,'occupied');
        meta.applied=ok;meta.detail='REPEATED_VIEWPOINT_INVALIDATION';
    otherwise
        meta.active=false;meta.detail='UNSUPPORTED_FAULT_NAME';
end
if meta.applied&&any(name==["F2","F3","F4","F5","F10","F11","F13","F14"])
    gridOut.mapVersion=uint32(double(gridOut.mapVersion)+double(fieldOr(fcfg,'versionOffset',1000000)));
end
end

function xy=futurePoint(request,currentXY,fraction)
xy=[nan nan];
if ~isfield(request,'knownFreeRouteXY'),return,end
p=double(request.knownFreeRouteXY);
if isempty(p)||size(p,2)~=2||any(~isfinite(p(:))),return,end
[~,idx]=min(vecnorm(p-currentXY,2,2));
lo=min(size(p,1),max(1,idx+1));hi=size(p,1);
if lo>hi,lo=hi;end
j=lo+round(fraction*max(0,hi-lo));j=max(lo,min(hi,j));xy=p(j,:);
end

function xy=behindSidePoint(g,request,currentXY)
xy=[nan nan];
if ~isfield(request,'knownFreeRouteXY'),return,end
p=double(request.knownFreeRouteXY);
if isempty(p)||size(p,2)~=2||size(p,1)<3||any(~isfinite(p(:))),return,end
[~,idx]=min(vecnorm(p-currentXY,2,2));
if idx<=2,return,end
j=max(1,idx-2);base=p(j,:);
if j<size(p,1),d=p(j+1,:)-p(j,:);else,d=p(j,:)-p(j-1,:);end
if norm(d)<1e-9,return,end
n=[-d(2) d(1)]/norm(d);
offsets=[3 -3 4 -4]*g.resolution;
for a=offsets
    q=base+a*n;
    [iy,ix,inside]=cellIndex(g,q);
    if ~inside||g.occ(iy,ix)||~g.knownFree(iy,ix),continue,end
    % Keep the injected cell away from the current/future route so only the
    % already traversed region changes.
    rem=p(max(1,idx):end,:);
    if min(vecnorm(rem-q,2,2))<=1.5*g.resolution,continue,end
    xy=q;return
end
end

function [g,ok]=sealStartNeighborhood(g,startXY,currentXY)
ok=false;startXY=double(startXY);
if numel(startXY)~=2||any(~isfinite(startXY)),return,end
[iy0,ix0,inside]=cellIndex(g,startXY);if ~inside,return,end
% Do not run this case until the vehicle has moved well away from start.
if norm(double(currentXY)-startXY)<4.0*g.resolution,return,end
changed=0;
for dy=-1:1
    for dx=-1:1
        if dx==0&&dy==0,continue,end
        iy=iy0+dy;ix=ix0+dx;
        if iy<1||iy>g.ny||ix<1||ix>g.nx,continue,end
        q=[g.xs(ix) g.ys(iy)];
        if norm(q-double(currentXY))<2.0*g.resolution,continue,end
        g.occ(iy,ix)=true;g.knownFree(iy,ix)=false;g.unknown(iy,ix)=false;
        if isfield(g,'staticOccupied'),g.staticOccupied(iy,ix)=true;end
        changed=changed+1;
    end
end
ok=changed>=3;
end

function [g,ok]=setCell(g,xy,kind)
ok=false;xy=double(xy);
if numel(xy)~=2||any(~isfinite(xy)),return,end
[iy,ix,inside]=cellIndex(g,xy);if ~inside,return,end
switch kind
    case 'occupied'
        g.occ(iy,ix)=true;g.knownFree(iy,ix)=false;g.unknown(iy,ix)=false;
        if isfield(g,'staticOccupied'),g.staticOccupied(iy,ix)=true;end
    case 'unknown'
        g.knownFree(iy,ix)=false;g.unknown(iy,ix)=true;g.occ(iy,ix)=true;
        if isfield(g,'staticOccupied'),g.staticOccupied(iy,ix)=false;end
        if isfield(g,'dynamicOccupied'),g.dynamicOccupied(iy,ix)=false;end
    otherwise
        return
end
ok=true;
end
function [iy,ix,inside]=cellIndex(g,xy)
ix=round((xy(1)-g.xs(1))/g.resolution)+1;
iy=round((xy(2)-g.ys(1))/g.resolution)+1;
inside=ix>=1&&ix<=g.nx&&iy>=1&&iy<=g.ny;
end
function v=fieldOr(s,name,default)
if isfield(s,name),v=s.(name);else,v=default;end
end

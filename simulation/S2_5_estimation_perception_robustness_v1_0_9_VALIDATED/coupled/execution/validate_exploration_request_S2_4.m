function status = validate_exploration_request_S2_4(c,grid,request,tNow)
% VALIDATE_EXPLORATION_REQUEST_S2_4 Recheck request before mission execution.
arguments
    c (1,1) struct
    grid (1,1) struct
    request (1,1) struct
    tNow (1,1) double
end
reasons={};
if ~isfield(request,'schema') || ~strcmp(request.schema,'S2_4_EXPLORATION_REQUEST_V1')
    reasons{end+1}='REQUEST_SCHEMA_INVALID';
end
if ~isfield(request,'valid') || ~request.valid
    reasons{end+1}='REQUEST_NOT_VALID';
end
if ~isfinite(request.validUntil_s) || tNow>request.validUntil_s+1e-9
    reasons{end+1}='REQUEST_EXPIRED';
end
cells=double(request.pathCells);
if isempty(cells) || size(cells,2)~=2 || any(~isfinite(cells(:))) || ...
        any(cells(:)~=round(cells(:)))
    reasons{end+1}='ROUTE_CELL_SCHEMA_INVALID';
else
    inside=cells(:,1)>=1 & cells(:,1)<=grid.ny & cells(:,2)>=1 & cells(:,2)<=grid.nx;
    if ~all(inside)
        reasons{end+1}='ROUTE_OUTSIDE_MAP';
    else
        idx=sub2ind(size(grid.knownFree),cells(:,1),cells(:,2));
        if any(~logical(grid.knownFree(idx)))
            reasons{end+1}='ROUTE_NOT_KNOWN_FREE';
        end
        if any(logical(grid.occ(idx)))
            reasons{end+1}='ROUTE_BLOCKED';
        end
    end
end
v=xyToCell(grid,request.positionXY);
if ~insideCell(grid,v)
    reasons{end+1}='VIEWPOINT_OUTSIDE_MAP';
else
    if ~grid.knownFree(v(1),v(2)) || grid.occ(v(1),v(2))
        reasons{end+1}='VIEWPOINT_NOT_EXECUTABLE';
    end
    y=max(1,v(1)-1):min(grid.ny,v(1)+1);
    x=max(1,v(2)-1):min(grid.nx,v(2)+1);
    if ~all(grid.knownFree(y,x)&~grid.occ(y,x),'all')
        reasons{end+1}='LOCAL_HOLD_SUPPORT_INVALID';
    end
end
if ~metricRouteExecutable(grid,request.retreatRouteXY)
    reasons{end+1}='RETREAT_ROUTE_INVALID';
end
% A changed map version does not automatically invalidate a request. The
% route and hold support are rechecked against the latest grid above.
mapChanged = uint32(grid.mapVersion)~=uint32(request.mapVersion);
status=struct('valid',isempty(reasons),'reasons',{unique(reasons,'stable')}, ...
    'mapChangedSinceRequest',mapChanged,'checkedMapVersion',uint32(grid.mapVersion), ...
    'requestMapVersion',uint32(request.mapVersion),'commandIssued',false);
if status.valid
    status.reason='VALID';
else
    status.reason=strjoin(status.reasons,'|');
end
end

function c=xyToCell(grid,xy)
c=[round((xy(2)-grid.ys(1))/grid.resolution)+1 ...
   round((xy(1)-grid.xs(1))/grid.resolution)+1];
end
function tf=insideCell(grid,c)
tf=numel(c)==2&&all(isfinite(c))&&all(c>=1)&&c(1)<=grid.ny&&c(2)<=grid.nx;
end
function tf=metricRouteExecutable(grid,route)
tf=false;
route=double(route);
if isempty(route)||size(route,2)~=2||size(route,1)<2||any(~isfinite(route(:))),return,end
for k=1:size(route,1)
    c=xyToCell(grid,route(k,:));
    if ~insideCell(grid,c)||~grid.knownFree(c(1),c(2))||grid.occ(c(1),c(2)),return,end
end
for k=1:size(route,1)-1
    if segment_occupied_grid_S2_2(grid,route(k,:),route(k+1,:)),return,end
end
tf=true;
end


function [status,requestOut] = revalidate_active_exploration_request_S2_4_F( ...
    cfg,c,grid,request,currentXY,currentVelocity,tNow)
% REVALIDATE_ACTIVE_EXPLORATION_REQUEST_S2_4_F Execution-time authority check.
%
% Only autonomy-visible planner-grid/request data are used. The frozen S2.3
% planner/controller are not replaced. A stale request may be refreshed only
% when current geometry proves equivalent known-free authority. An already
% expired request is never renewed.
arguments
    cfg (1,1) struct
    c (1,1) struct
    grid (1,1) struct
    request (1,1) struct
    currentXY (1,2) double
    currentVelocity (2,1) double
    tNow (1,1) double
end
requestOut=request;reasons={};
if ~isfield(request,'schema')||~strcmp(request.schema,'S2_4_EXPLORATION_REQUEST_V1')
    reasons{end+1}='REQUEST_SCHEMA_INVALID';
end
if ~isfield(request,'valid')||~request.valid
    reasons{end+1}='REQUEST_NOT_VALID';
end
expired=~isfield(request,'validUntil_s')||~isfinite(request.validUntil_s)|| ...
    tNow>request.validUntil_s+1e-9;
if expired,reasons{end+1}='REQUEST_EXPIRED';end

remaining=remainingRoute(request,currentXY);
forwardSafe=metricRouteExecutable(grid,remaining);
if ~forwardSafe,reasons{end+1}='REMAINING_ROUTE_INVALID';end

viewpointSafe=false;holdSupportSafe=false;
viewXY=[nan nan];
if isfield(request,'positionXY')&&numel(request.positionXY)==2&& ...
        all(isfinite(request.positionXY))
    viewXY=double(request.positionXY);
    v=xyToCell(grid,viewXY);
    if insideCell(grid,v)
        viewpointSafe=grid.knownFree(v(1),v(2))&&~grid.occ(v(1),v(2));
        y=max(1,v(1)-1):min(grid.ny,v(1)+1);
        x=max(1,v(2)-1):min(grid.nx,v(2)+1);
        holdSupportSafe=all(grid.knownFree(y,x)&~grid.occ(y,x),'all');
    end
end
if ~viewpointSafe,reasons{end+1}='VIEWPOINT_NOT_EXECUTABLE';end
if ~holdSupportSafe,reasons{end+1}='LOCAL_HOLD_SUPPORT_INVALID';end

% Runtime stopping reserve is checked against the REMAINING accepted route,
% not by re-running the frozen S2.3 planning-time terminal gate every cycle.
% The frozen gate is already exercised when the segment is planned. At runtime,
% the execution grid is already vehicle/localization/control/uncertainty-inflated,
% so forwardSafe proves the remaining centreline corridor is collision-clear.
% We therefore require enough known-free arc length for the inherited stop model.
% If the remaining route is shorter than the conservative stopping distance,
% the shortfall must be covered by a known-free terminal overrun disk around the
% already validated viewpoint. This preserves fail-closed stopping support while
% avoiding the v1.1.1 false abort caused by applying a terminal landing-footprint
% clearance test at every intermediate execution sample.
speed=norm(currentVelocity(:));
dStop=speed^2/(2*max(cfg.maxDecelXY_mps2,eps))+ ...
    speed*cfg.sensorControlDelay_s+cfg.mapStopExtraMargin_m;
remainingLength=metricRouteLength(remaining);
routeStopReserveSafe=forwardSafe&&remainingLength+1e-9>=dStop;
terminalOverrunNeeded=max(0,dStop-remainingLength);
terminalOverrunReserveSafe=false;
if forwardSafe&&viewpointSafe&&holdSupportSafe&&all(isfinite(viewXY))
    terminalOverrunReserveSafe=knownFreeDisk(grid,viewXY,terminalOverrunNeeded);
end
stoppingSupportSafe=routeStopReserveSafe||terminalOverrunReserveSafe;
if ~stoppingSupportSafe,reasons{end+1}='KNOWN_FREE_STOP_INVALID';end

retreatStoredSafe=isfield(request,'retreatRouteXY')&& ...
    metricRouteExecutable(grid,request.retreatRouteXY);
retreatRefreshed=false;retreatSafe=retreatStoredSafe;
if ~retreatStoredSafe&&isfield(request,'startXY')&&numel(request.startXY)==2&& ...
        all(isfinite(request.startXY))
    [candidate,~]=astar_grid_S2_2(grid,currentXY,double(request.startXY));
    candidate=double(candidate);
    if metricRouteExecutable(grid,candidate)
        requestOut.retreatRouteXY=candidate;
        retreatSafe=true;retreatRefreshed=true;
    end
end
if ~retreatSafe,reasons{end+1}='RETREAT_ROUTE_INVALID';end

mapChanged=isfield(request,'mapVersion')&& ...
    uint32(grid.mapVersion)~=uint32(request.mapVersion);
status=struct( ...
    'valid',isempty(reasons), ...
    'reasons',{unique(reasons,'stable')}, ...
    'mapChangedSinceRequest',mapChanged, ...
    'checkedMapVersion',uint32(grid.mapVersion), ...
    'requestMapVersion',uint32(fieldOr(request,'mapVersion',0)), ...
    'remainingRouteXY',remaining, ...
    'forwardSafe',forwardSafe, ...
    'viewpointSafe',viewpointSafe, ...
    'holdSupportSafe',holdSupportSafe, ...
    'stoppingSupportSafe',stoppingSupportSafe, ...
    'stoppingDistance_m',dStop, ...
    'remainingRouteLength_m',remainingLength, ...
    'routeStopReserveSafe',routeStopReserveSafe, ...
    'terminalOverrunNeeded_m',terminalOverrunNeeded, ...
    'terminalOverrunReserveSafe',terminalOverrunReserveSafe, ...
    'retreatSafe',retreatSafe, ...
    'retreatRefreshed',retreatRefreshed, ...
    'requestExpired',expired, ...
    'leaseRenewed',false, ...
    'commandIssued',false);
if status.valid
    status.reason='VALID';
    requestOut.mapVersion=uint32(grid.mapVersion);
    renew=isfield(cfg,'executionSafety')&& ...
        logical(fieldOr(cfg.executionSafety,'renewLeaseOnValidRevalidation',false));
    if renew
        % Renewal is downstream of every CURRENT safety check above; F8 forces
        % expiry before this point and therefore cannot be silently revived.
        requestOut.validUntil_s=tNow+double(c.requestValidity_s);
        requestOut.lastRevalidatedTime_s=tNow;
        status.leaseRenewed=true;
    end
else
    status.reason=strjoin(status.reasons,'|');
end
end

function route=remainingRoute(request,currentXY)
route=zeros(0,2);
if ~isfield(request,'knownFreeRouteXY'),return,end
p=double(request.knownFreeRouteXY);
if isempty(p)||size(p,2)~=2||size(p,1)<2||any(~isfinite(p(:))),return,end
best=inf;bestSeg=1;
for k=1:size(p,1)-1
    a=p(k,:);b=p(k+1,:);d=b-a;
    if dot(d,d)<=eps,q=a;else,u=max(0,min(1,dot(currentXY-a,d)/dot(d,d)));q=a+u*d;end
    dist=norm(currentXY-q);if dist<best,best=dist;bestSeg=k;end
end
route=[currentXY;p(bestSeg+1:end,:)];route=removeNearDuplicates(route,1e-9);
end

function tf=metricRouteExecutable(grid,route)
tf=false;route=double(route);
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

function d=metricRouteLength(route)
d=0;route=double(route);
if isempty(route)||size(route,2)~=2||size(route,1)<2||any(~isfinite(route(:))),return,end
for k=1:size(route,1)-1,d=d+norm(route(k+1,:)-route(k,:));end
end

function tf=knownFreeDisk(grid,xy,radius)
tf=false;xy=double(xy);radius=max(0,double(radius));
if numel(xy)~=2||any(~isfinite(xy)),return,end
c=xyToCell(grid,xy);if ~insideCell(grid,c),return,end
rCells=ceil((radius+0.5*sqrt(2)*grid.resolution)/grid.resolution);
for iy=max(1,c(1)-rCells):min(grid.ny,c(1)+rCells)
    for ix=max(1,c(2)-rCells):min(grid.nx,c(2)+rCells)
        q=[grid.xs(ix) grid.ys(iy)];
        if norm(q-xy)<=radius+0.5*sqrt(2)*grid.resolution
            if grid.occ(iy,ix)||~grid.knownFree(iy,ix),return,end
        end
    end
end
tf=true;
end
function p=removeNearDuplicates(p,tol)
if isempty(p),return,end
keep=true(size(p,1),1);last=1;
for k=2:size(p,1)
    if norm(p(k,:)-p(last,:))<=tol,keep(k)=false;else,last=k;end
end
p=p(keep,:);
end
function c=xyToCell(grid,xy)
c=[round((xy(2)-grid.ys(1))/grid.resolution)+1 ...
   round((xy(1)-grid.xs(1))/grid.resolution)+1];
end
function tf=insideCell(grid,c)
tf=numel(c)==2&&all(isfinite(c))&&all(c>=1)&&c(1)<=grid.ny&&c(2)<=grid.nx;
end
function v=fieldOr(s,name,default)
if isfield(s,name),v=s.(name);else,v=default;end
end

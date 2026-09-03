function [planner,path,traj,stats,jump,routeExists,meta] = plan_recovery_viewpoint_S2_5(cfg,c,grid,est,estAcc,goalXY,initialScale)
% PLAN_RECOVERY_VIEWPOINT_S2_5 Execution-qualified informative-path recovery.
%
% S2.5 v1.0.5 architecture (Python exact-snapshot qualified):
%   Stage 0 - Continuous Start Egress (CSE): if only the rounded planner cell is
%      blocked, certify the ACTUAL continuous start pose and a connector to
%      the nearest execution-free node against the unchanged metric inflation.
%      No map cell is cleared globally; a local trajectory grid exempts only
%      that rounded current cell, and every segment using the exemption is
%      re-certified against continuous geometry.
%   Stage 1 - Stop-first Safe Informative Excursion (SIE): rank stop-safe terminals
%      by information visible ALONG a known-free route, preserving the most
%      informative route anchor through path smoothing.
%   Stage 2 - Transit-anchor detour: if Stage 1 has no executable result, route via
%      an informative known-free transit cell to a separate stop-safe terminal.
%
% No estimator, map, controller, execution, collision, freshness, stopping,
% or unknown-space threshold is relaxed.

meta=empty_meta(goalXY);
planner=[];path=zeros(0,2);traj=invalid_traj_local();jump=inf(1,3);
stats=struct('dstarExpanded',0,'astarExpanded',0,'astarRecovery',0);
routeExists=false;

required={'candidateRadii_m','minVisibleUnknownCells','staleFreeAge_s'};
for k=1:numel(required)
    if ~isfield(c,required{k})
        meta.reason=['recovery_config_missing_' required{k}];
        meta.recoveryDiagnostics.configError=true;
        return;
    end
end

g=build_execution_grid_S2_4(grid);
start=xy2cell_local(g,est.p(1:2).');
if ~inside_local(g,start)||~g.knownFree(start(1),start(2))
    meta.reason='recovery_start_not_known_free';return;
end

% -------------------------------------------------------------------------
% Stage 0: Continuous Start Egress (CSE).
% The exact historical snapshots showed four failures where the actual x pose
% was >0.602 m from the wall while round(x/0.1) selected the 0.6 m node, which
% is correctly marked occupied by the discrete wall-inset rule. CSE corrects
% only that discretisation mismatch. It does NOT authorize unknown traversal.
% -------------------------------------------------------------------------
planGrid=grid;
startRoundedBlocked=logical(g.navigationBlocked(start(1),start(2)));
meta.recoveryDiagnostics.startRoundedBlocked=startRoundedBlocked;
if startRoundedBlocked
    meta.recoveryDiagnostics.continuousStartEgressAttempted=true;
    [egressOK,egressCell,egressLength]=continuous_start_egress_local(c,g,est.p(1:2).',start);
    meta.recoveryDiagnostics.continuousStartEgressLength_m=egressLength;
    if ~egressOK
        meta.recoveryDiagnostics.continuousStartEgressRejected=true;
        meta.reason='recovery_continuous_start_egress_invalid';
        return;
    end
    meta.recoveryDiagnostics.continuousStartEgressUsed=true;
    meta.recoveryDiagnostics.continuousStartEgressCell=egressCell;
    % Local planning/trajectory copy only. The authoritative grid is unchanged.
    planGrid.occ(start(1),start(2))=false;
else
    egressLength=0;
end

terminals=stop_safe_terminals_local(cfg,g,grid);
meta.recoveryDiagnostics.stopSafeTerminals=size(terminals,1);
if isempty(terminals)
    meta.reason='recovery_no_stop_safe_terminal';return;
end

astarExpandedTotal=0;
minRelocation=min(c.candidateRadii_m);

% -------------------------------------------------------------------------
% Stage 1: stop-first informative route.
% A terminal is eligible only if it already has inherited hold/landing support.
% Information is evaluated along the current metric A* route. This makes a
% transient sensing pose useful without requiring that same pose to be a stop.
% -------------------------------------------------------------------------
rows=zeros(0,8); % [-gain routeLength goalDistance ty tx ay ax candidateOrdinal]
stage1MetricRejected=0;stage1StaleRejected=0;stage1DynamicRejected=0;
for n=1:size(terminals,1)
    term=terminals(n,:); targetXY=[g.xs(term(2)) g.ys(term(1))];
    [rawA,aInfo]=astar_grid_S2_2(grid,est.p(1:2).',targetXY);
    astarExpandedTotal=astarExpandedTotal+aInfo.expanded;
    if isempty(rawA),stage1MetricRejected=stage1MetricRejected+1;continue;end
    routeCells=metric_path_to_cells_local(g,rawA);
    if isempty(routeCells),stage1MetricRejected=stage1MetricRejected+1;continue;end
    [freshOK,dynamicOK]=route_current_local(c,g,routeCells);
    if ~freshOK,stage1StaleRejected=stage1StaleRejected+1;continue;end
    if ~dynamicOK,stage1DynamicRejected=stage1DynamicRejected+1;continue;end
    [gain,anchor]=best_information_anchor_local(c,g,routeCells);
    if gain<c.minVisibleUnknownCells||isempty(anchor),continue;end
    routeLength=path_length_metric_local(rawA)+egressLength;
    if routeLength+1e-12<minRelocation,continue;end
    gd=norm(targetXY-goalXY(:).');
    rows(end+1,:)=[-gain routeLength gd term anchor n]; %#ok<AGROW>
end
if ~isempty(rows),rows=sortrows(rows,[1 2 3 4 5 6 7 8]);end
meta.recoveryDiagnostics.stage1Candidates=size(rows,1);
meta.recoveryDiagnostics.stage1MetricRouteRejected=stage1MetricRejected;
meta.recoveryDiagnostics.stage1StaleRouteRejected=stage1StaleRejected;
meta.recoveryDiagnostics.stage1DynamicRouteRejected=stage1DynamicRejected;
% Preserve legacy diagnostic fields for downstream reporting.
meta.recoveryDiagnostics.rankedCandidates=size(rows,1);
meta.recoveryDiagnostics.metricRouteRejected=stage1MetricRejected;
meta.recoveryDiagnostics.staleRouteRejected=stage1StaleRejected;
meta.recoveryDiagnostics.dynamicRouteRejected=stage1DynamicRejected;

for k=1:size(rows,1)
    term=round(rows(k,4:5));anchor=round(rows(k,6:7));
    targetXY=[g.xs(term(2)) g.ys(term(1))];
    [rawA,aInfo]=astar_grid_S2_2(grid,est.p(1:2).',targetXY);
    astarExpandedTotal=astarExpandedTotal+aInfo.expanded;
    if isempty(rawA)
        meta.recoveryDiagnostics.stage1MetricRouteRejected=meta.recoveryDiagnostics.stage1MetricRouteRejected+1;
        meta.recoveryDiagnostics.metricRouteRejected=meta.recoveryDiagnostics.metricRouteRejected+1;
        continue;
    end
    routeCells=metric_path_to_cells_local(g,rawA);
    [freshOK,dynamicOK]=route_current_local(c,g,routeCells);
    if ~freshOK
        meta.recoveryDiagnostics.stage1StaleRouteRejected=meta.recoveryDiagnostics.stage1StaleRouteRejected+1;
        meta.recoveryDiagnostics.staleRouteRejected=meta.recoveryDiagnostics.staleRouteRejected+1;continue;
    end
    if ~dynamicOK
        meta.recoveryDiagnostics.stage1DynamicRouteRejected=meta.recoveryDiagnostics.stage1DynamicRouteRejected+1;
        meta.recoveryDiagnostics.dynamicRouteRejected=meta.recoveryDiagnostics.dynamicRouteRejected+1;continue;
    end
    candidatePath=prepare_path_preserve_anchor_local(g,grid,planGrid,est.p(1:2).',rawA,anchor);
    if isempty(candidatePath)||path_length_metric_local(candidatePath)+1e-12<minRelocation
        meta.recoveryDiagnostics.stage1PreparedPathRejected=meta.recoveryDiagnostics.stage1PreparedPathRejected+1;
        meta.recoveryDiagnostics.preparedPathRejected=meta.recoveryDiagnostics.preparedPathRejected+1;continue;
    end
    [candidateTraj,candidateJump]=make_traj_hybrid_local(cfg,g,grid,planGrid,candidatePath,est,estAcc,initialScale);
    if ~candidateTraj.valid
        meta.recoveryDiagnostics.stage1TrajectoryRejected=meta.recoveryDiagnostics.stage1TrajectoryRejected+1;
        meta.recoveryDiagnostics.trajectoryRejected=meta.recoveryDiagnostics.trajectoryRejected+1;continue;
    end
    if ~validate_known_free_stop_S2_3(cfg,grid,candidatePath,est.v(1:2))
        meta.recoveryDiagnostics.stage1StopRejected=meta.recoveryDiagnostics.stage1StopRejected+1;
        meta.recoveryDiagnostics.stopRejected=meta.recoveryDiagnostics.stopRejected+1;continue;
    end
    [planner,~,dstat]=dstar_lite_S2_2('init',grid,est.p(1:2).',targetXY);
    path=candidatePath;traj=candidateTraj;jump=candidateJump;routeExists=true;
    stats=struct('dstarExpanded',dstat.expanded,'astarExpanded',astarExpandedTotal,'astarRecovery',1);
    meta.stopFeasible=true;meta.segmentTarget=targetXY;meta.isFinal=false;meta.frontierUsed=true;
    meta.reason='s25_safe_informative_excursion_stage1';
    meta.visibleUnknownCount=-rows(k,1);meta.recoveryTravelCost_m=path_length_metric_local(candidatePath);
    meta.recoveryGoalProgress_m=norm(est.p(1:2).'-goalXY(:).')-norm(targetXY-goalXY(:).');
    meta.recoveryDiagnostics.selectedRank=k;meta.recoveryDiagnostics.stage1SelectedRank=k;
    meta.recoveryDiagnostics.selectedStage=1;meta.recoveryDiagnostics.selectedGain=-rows(k,1);
    meta.recoveryDiagnostics.selectedAnchorCell=anchor;meta.recoveryDiagnostics.selectedTerminalCell=term;
    meta.recoveryDiagnostics.fullFeasible=meta.recoveryDiagnostics.fullFeasible+1;
    return;
end

% -------------------------------------------------------------------------
% Stage 2: informative transit anchor -> separate stop-safe terminal.
% This is reached only if no Stage-1 path survives every execution check.
% -------------------------------------------------------------------------
anchors=zeros(0,6); % [-gain routeLength goalDistance ay ax ordinal]
allowed=logical(g.knownFree)&~logical(g.navigationBlocked);ordinal=0;
for iy=1:g.ny
    for ix=1:g.nx
        if ~allowed(iy,ix),continue;end
        gain=size(visible_unknown_local(g,[iy ix],0,c),1);
        if gain<c.minVisibleUnknownCells,continue;end
        anchorXY=[g.xs(ix) g.ys(iy)];
        [raw1,aInfo]=astar_grid_S2_2(grid,est.p(1:2).',anchorXY);
        astarExpandedTotal=astarExpandedTotal+aInfo.expanded;
        if isempty(raw1),continue;end
        cells1=metric_path_to_cells_local(g,raw1);[freshOK,dynamicOK]=route_current_local(c,g,cells1);
        if ~freshOK||~dynamicOK,continue;end
        L=path_length_metric_local(raw1)+egressLength;
        if L+1e-12<minRelocation,continue;end
        ordinal=ordinal+1;gd=norm(anchorXY-goalXY(:).');
        anchors(end+1,:)=[-gain L gd iy ix ordinal]; %#ok<AGROW>
    end
end
if ~isempty(anchors),anchors=sortrows(anchors,[1 2 3 4 5 6]);end
meta.recoveryDiagnostics.stage2Anchors=size(anchors,1);
meta.recoveryDiagnostics.rankedCandidates=meta.recoveryDiagnostics.rankedCandidates+size(anchors,1);

for ak=1:size(anchors,1)
    anchor=round(anchors(ak,4:5));anchorXY=[g.xs(anchor(2)) g.ys(anchor(1))];
    [raw1,aInfo]=astar_grid_S2_2(grid,est.p(1:2).',anchorXY);
    astarExpandedTotal=astarExpandedTotal+aInfo.expanded;
    if isempty(raw1),meta.recoveryDiagnostics.stage2MetricRouteRejected=meta.recoveryDiagnostics.stage2MetricRouteRejected+1;continue;end
    cells1=metric_path_to_cells_local(g,raw1);[fresh1,dyn1]=route_current_local(c,g,cells1);
    if ~fresh1||~dyn1,meta.recoveryDiagnostics.stage2RouteRejected=meta.recoveryDiagnostics.stage2RouteRejected+1;continue;end
    order=terminal_order_local(g,terminals,anchor,goalXY);
    for tk=1:numel(order)
        term=terminals(order(tk),:);targetXY=[g.xs(term(2)) g.ys(term(1))];
        [raw2,aInfo2]=astar_grid_S2_2(grid,anchorXY,targetXY);
        astarExpandedTotal=astarExpandedTotal+aInfo2.expanded;
        if isempty(raw2),meta.recoveryDiagnostics.stage2TerminalMetricRejected=meta.recoveryDiagnostics.stage2TerminalMetricRejected+1;continue;end
        cells2=metric_path_to_cells_local(g,raw2);[fresh2,dyn2]=route_current_local(c,g,cells2);
        if ~fresh2||~dyn2,meta.recoveryDiagnostics.stage2TerminalRouteRejected=meta.recoveryDiagnostics.stage2TerminalRouteRejected+1;continue;end
        raw=[raw1;raw2(2:end,:)];
        candidatePath=prepare_path_preserve_anchor_local(g,grid,planGrid,est.p(1:2).',raw,anchor);
        if isempty(candidatePath)||path_length_metric_local(candidatePath)+1e-12<minRelocation
            meta.recoveryDiagnostics.stage2PreparedPathRejected=meta.recoveryDiagnostics.stage2PreparedPathRejected+1;continue;
        end
        [candidateTraj,candidateJump]=make_traj_hybrid_local(cfg,g,grid,planGrid,candidatePath,est,estAcc,initialScale);
        if ~candidateTraj.valid,meta.recoveryDiagnostics.stage2TrajectoryRejected=meta.recoveryDiagnostics.stage2TrajectoryRejected+1;continue;end
        if ~validate_known_free_stop_S2_3(cfg,grid,candidatePath,est.v(1:2))
            meta.recoveryDiagnostics.stage2StopRejected=meta.recoveryDiagnostics.stage2StopRejected+1;continue;
        end
        [planner,~,dstat]=dstar_lite_S2_2('init',grid,est.p(1:2).',targetXY);
        path=candidatePath;traj=candidateTraj;jump=candidateJump;routeExists=true;
        stats=struct('dstarExpanded',dstat.expanded,'astarExpanded',astarExpandedTotal,'astarRecovery',1);
        meta.stopFeasible=true;meta.segmentTarget=targetXY;meta.isFinal=false;meta.frontierUsed=true;
        meta.reason='s25_safe_informative_excursion_stage2';meta.visibleUnknownCount=-anchors(ak,1);
        meta.recoveryTravelCost_m=path_length_metric_local(candidatePath);
        meta.recoveryGoalProgress_m=norm(est.p(1:2).'-goalXY(:).')-norm(targetXY-goalXY(:).');
        meta.recoveryDiagnostics.selectedRank=ak;meta.recoveryDiagnostics.stage2SelectedAnchorRank=ak;
        meta.recoveryDiagnostics.stage2SelectedTerminalRank=tk;meta.recoveryDiagnostics.selectedStage=2;
        meta.recoveryDiagnostics.selectedGain=-anchors(ak,1);meta.recoveryDiagnostics.selectedAnchorCell=anchor;
        meta.recoveryDiagnostics.selectedTerminalCell=term;meta.recoveryDiagnostics.fullFeasible=meta.recoveryDiagnostics.fullFeasible+1;
        return;
    end
end

stats=struct('dstarExpanded',0,'astarExpanded',astarExpandedTotal,'astarRecovery',0);
meta.reason='recovery_no_execution_qualified_informative_excursion';
end

function terminals=stop_safe_terminals_local(cfg,g,grid)
terminals=zeros(0,2);radius=cfg.collisionRadius+cfg.controlMargin;
for iy=2:g.ny-1
    for ix=2:g.nx-1
        if ~g.knownFree(iy,ix)||g.navigationBlocked(iy,ix),continue;end
        if ~hold_support_local(g,[iy ix]),continue;end
        if ~landing_zone_clear_S2_2(grid,[g.xs(ix) g.ys(iy)],radius),continue;end
        terminals(end+1,:)=[iy ix]; %#ok<AGROW>
    end
end
end

function [gain,anchor]=best_information_anchor_local(c,g,routeCells)
gain=0;anchor=[];
for k=1:size(routeCells,1)
    n=size(visible_unknown_local(g,routeCells(k,:),0,c),1);
    if n>gain,gain=n;anchor=routeCells(k,:);end
end
end

function [freshOK,dynamicOK]=route_current_local(c,g,cells)
freshOK=~isempty(cells);dynamicOK=~isempty(cells);if isempty(cells),return;end
if isfield(g,'lastObservedXY')&&isfinite(g.timestamp)
    age=g.timestamp-double(g.lastObservedXY);idx=sub2ind(size(age),cells(:,1),cells(:,2));
    freshOK=all(age(idx)<=c.staleFreeAge_s);
end
if isfield(g,'dynamicOccupiedRaw')
    idx=sub2ind(size(g.dynamicOccupiedRaw),cells(:,1),cells(:,2));dynamicOK=~any(g.dynamicOccupiedRaw(idx));
end
end

function order=terminal_order_local(g,terminals,anchor,goalXY)
rows=zeros(size(terminals,1),4);
for k=1:size(terminals,1)
    t=terminals(k,:);d=hypot(t(1)-anchor(1),t(2)-anchor(2));
    gd=hypot(g.xs(t(2))-goalXY(1),g.ys(t(1))-goalXY(2));rows(k,:)=[d gd t];
end
[~,order]=sortrows(rows,[1 2 3 4]);
end

function [ok,freeCell,connectorLength]=continuous_start_egress_local(c,g,startXY,startCell)
ok=false;freeCell=zeros(1,2);connectorLength=inf;
if ~inside_local(g,startCell)||~g.knownFree(startCell(1),startCell(2)),return;end
if isfield(g,'dynamicOccupiedRaw')&&g.dynamicOccupiedRaw(startCell(1),startCell(2)),return;end
if isfield(g,'lastObservedXY')&&isfinite(g.timestamp)
    if g.timestamp-double(g.lastObservedXY(startCell(1),startCell(2)))>c.staleFreeAge_s,return;end
end
[freeCell,found]=nearest_execution_free_cell_local(g,startCell,12);if ~found,return;end
q=[g.xs(freeCell(2)) g.ys(freeCell(1))];connectorLength=norm(q-startXY);
if ~continuous_segment_safe_local(g,startXY,q),return;end
% The connector terminal itself remains an ordinary known-free execution node.
if ~g.knownFree(freeCell(1),freeCell(2))||g.navigationBlocked(freeCell(1),freeCell(2)),return;end
ok=true;
end

function [best,ok]=nearest_execution_free_cell_local(g,start,maxCells)
best=start;ok=false;if inside_local(g,start)&&~g.navigationBlocked(start(1),start(2))&&g.knownFree(start(1),start(2)),ok=true;return;end
bestD2=inf;
for r=1:maxCells
    for iy=max(1,start(1)-r):min(g.ny,start(1)+r)
        for ix=max(1,start(2)-r):min(g.nx,start(2)+r)
            if g.navigationBlocked(iy,ix)||~g.knownFree(iy,ix),continue;end
            d2=(ix-start(2))^2+(iy-start(1))^2;
            if d2<bestD2,bestD2=d2;best=[iy ix];ok=true;end
        end
    end
    if ok,return;end
end
end

function tf=continuous_segment_safe_local(g,a,b)
% Continuous metric certificate matching project_map_to_planner_S2_3:
% room inset by inflationRadius and raw hazard-node distance strictly > radius.
tf=false;a=double(a(:).');b=double(b(:).');
if numel(a)~=2||numel(b)~=2||any(~isfinite([a b]))||~isfield(g,'inflationRadius'),return;end
r=double(g.inflationRadius);lo=[g.xs(1)+r g.ys(1)+r];hi=[g.xs(end)-r g.ys(end)-r];
if any(a<lo-1e-12)||any(a>hi+1e-12)||any(b<lo-1e-12)||any(b>hi+1e-12),return;end
raw=logical(g.staticOccupiedRaw)|logical(g.dynamicOccupiedRaw)|logical(g.unknown);[ys,xs]=find(raw);
for k=1:numel(xs)
    q=[g.xs(xs(k)) g.ys(ys(k))];
    if point_segment_distance_local(q,a,b)<=r+1e-12,return;end
end
tf=true;
end

function d=point_segment_distance_local(p,a,b)
v=b-a;den=dot(v,v);if den<=1e-18,d=norm(p-a);return;end
t=max(0,min(1,dot(p-a,v)/den));d=norm(p-(a+t*v));
end

function path=prepare_path_preserve_anchor_local(g,grid,planGrid,startXY,raw,anchorCell)
path=zeros(0,2);if isempty(raw),return;end
cells=metric_path_to_cells_local(g,raw);if isempty(cells),return;end
idx=find(cells(:,1)==anchorCell(1)&cells(:,2)==anchorCell(2),1,'first');if isempty(idx),return;end
% metric_path_to_cells_local removes duplicate cells; raw A* itself has one
% point per cell, so the anchor index is also the raw A* point index.
idx=min(idx,size(raw,1));
leg1=[double(startXY(:).');double(raw(1:idx,:))];leg1=remove_duplicates_local(leg1,grid.resolution/4);
leg2=double(raw(idx:end,:));leg2=remove_duplicates_local(leg2,grid.resolution/4);
leg1=smooth_leg_local(planGrid,leg1,startXY);leg2=smooth_leg_local(planGrid,leg2,[]);
if isempty(leg1)&&isempty(leg2),return;elseif isempty(leg1),candidate=leg2;elseif isempty(leg2),candidate=leg1;else,candidate=[leg1;leg2(2:end,:)];end
if ~path_valid_hybrid_local(g,grid,planGrid,candidate),return;end
path=candidate;
end

function leg=smooth_leg_local(planGrid,raw,startXY)
leg=zeros(0,2);if isempty(raw),return;end
if size(raw,1)==1,leg=raw;return;end
candidate=smooth_path_S2_2(planGrid,raw);if ~isempty(startXY)&&~isempty(candidate),candidate(1,:)=startXY(:).';end
if path_valid_grid_local(planGrid,candidate),leg=candidate;elseif path_valid_grid_local(planGrid,raw),leg=raw;end
end

function tf=path_valid_grid_local(grid,path)
tf=~isempty(path);if ~tf,return;end
if size(path,1)==1,tf=~segment_occupied_grid_S2_2(grid,path(1,:),path(1,:));return;end
for i=1:size(path,1)-1
    if segment_occupied_grid_S2_2(grid,path(i,:),path(i+1,:)),tf=false;return;end
end
end

function tf=path_valid_hybrid_local(g,grid,planGrid,path)
tf=path_valid_grid_local(planGrid,path);if ~tf,return;end
for i=1:size(path,1)-1
    if segment_occupied_grid_S2_2(grid,path(i,:),path(i+1,:))&&~continuous_segment_safe_local(g,path(i,:),path(i+1,:))
        tf=false;return;
    end
end
end

function [traj,jump]=make_traj_hybrid_local(cfg,g,grid,planGrid,path,est,estAcc,initialScale)
traj=generate_strict_trajectory_S2_3(cfg,planGrid,path,est.v(1:2),estAcc(1:2),initialScale);jump=inf(1,3);
if ~traj.valid,return;end
if ~trajectory_hybrid_safe_local(g,grid,traj),traj=invalid_traj_local();return;end
r=sample_min_snap_state_S2_2(traj,0);jump=[norm(r.p-est.p(1:2)) norm(r.v-est.v(1:2)) norm(r.a-estAcc(1:2))];
end

function tf=trajectory_hybrid_safe_local(g,grid,traj)
tf=false;if ~traj.valid||~isfield(traj,'sample')||~isfield(traj.sample,'p'),return;end
p=traj.sample.p;if isempty(p)||size(p,2)~=2,return;end
if size(p,1)==1
    if segment_occupied_grid_S2_2(grid,p(1,:),p(1,:))&&~continuous_segment_safe_local(g,p(1,:),p(1,:)),return;end
else
    for i=1:size(p,1)-1
        if segment_occupied_grid_S2_2(grid,p(i,:),p(i+1,:))&&~continuous_segment_safe_local(g,p(i,:),p(i+1,:)),return;end
    end
end
tf=true;
end

function tf=hold_support_local(g,cellXY)
y=cellXY(1);x=cellXY(2);tf=false;if ~inside_local(g,cellXY),return;end
yy=max(1,y-1):min(g.ny,y+1);xx=max(1,x-1):min(g.nx,x+1);
tf=all(g.knownFree(yy,xx)&~g.navigationBlocked(yy,xx),'all');
end

function vis=visible_unknown_local(g,o,yaw,c) %#ok<INUSD>
vis=zeros(0,2);angles=linspace(-pi,pi,121);maxStep=floor(6.5/g.resolution);seen=false(g.ny,g.nx);
for a=angles
    aa=yaw+a;
    for step=1:maxStep
        p=[round(o(1)+step*sin(aa)) round(o(2)+step*cos(aa))];
        if ~inside_local(g,p),break;end
        if g.staticOccupiedRaw(p(1),p(2))||g.dynamicOccupiedRaw(p(1),p(2)),break;end
        if g.unknown(p(1),p(2)),seen(p(1),p(2))=true;break;end
    end
end
[y,x]=find(seen);vis=[y x];
end

function cells=metric_path_to_cells_local(g,path)
cells=zeros(0,2);if isempty(path),return;end
cells=zeros(size(path,1),2);
for i=1:size(path,1)
    cells(i,:)=xy2cell_local(g,path(i,:));if ~inside_local(g,cells(i,:)),cells=zeros(0,2);return;end
end
cells=unique(cells,'rows','stable');
end

function d=path_length_metric_local(p)
d=0;for i=2:size(p,1),d=d+norm(p(i,:)-p(i-1,:));end
end

function p=remove_duplicates_local(p,tol)
if isempty(p),return;end
keep=true(size(p,1),1);last=1;
for i=2:size(p,1)
    if norm(p(i,:)-p(last,:))<tol,keep(i)=false;else,last=i;end
end
p=p(keep,:);
end

function c=xy2cell_local(g,xy)
c=[round((xy(2)-g.ys(1))/g.resolution)+1 round((xy(1)-g.xs(1))/g.resolution)+1];
end
function tf=inside_local(g,c)
tf=numel(c)==2&&all(c>=1)&&c(1)<=g.ny&&c(2)<=g.nx;
end
function tr=invalid_traj_local()
tr=struct('valid',false,'fallbackUsed',false,'timeScale',0,'continuity',zeros(1,4), ...
    'maxSpeed_mps',0,'maxAccel_mps2',0,'maxJerk_mps3',0);
end
function meta=empty_meta(goalXY)
diag=struct('architecture','CSE_SIE_V1','configError',false,'shellFallbackToAllKnownFree',false, ...
    'rankedCandidates',0,'metricRouteRejected',0,'staleRouteRejected',0,'dynamicRouteRejected',0, ...
    'preparedPathRejected',0,'trajectoryRejected',0,'stopRejected',0,'fullFeasible',0,'selectedRank',0, ...
    'startRoundedBlocked',false,'continuousStartEgressAttempted',false,'continuousStartEgressUsed',false, ...
    'continuousStartEgressRejected',false,'continuousStartEgressLength_m',0,'continuousStartEgressCell',[0 0], ...
    'stopSafeTerminals',0,'stage1Candidates',0,'stage1MetricRouteRejected',0,'stage1StaleRouteRejected',0, ...
    'stage1DynamicRouteRejected',0,'stage1PreparedPathRejected',0,'stage1TrajectoryRejected',0,'stage1StopRejected',0, ...
    'stage1SelectedRank',0,'stage2Anchors',0,'stage2MetricRouteRejected',0,'stage2RouteRejected',0, ...
    'stage2TerminalMetricRejected',0,'stage2TerminalRouteRejected',0,'stage2PreparedPathRejected',0, ...
    'stage2TrajectoryRejected',0,'stage2StopRejected',0,'stage2SelectedAnchorRank',0,'stage2SelectedTerminalRank',0, ...
    'selectedStage',0,'selectedGain',0,'selectedAnchorCell',[0 0],'selectedTerminalCell',[0 0]);
meta=struct('segmentTarget',goalXY(:).','isFinal',true,'frontierUsed',false,'goalCurrentlyReachable',false, ...
    'stopFeasible',false,'reason','','explorationUsed',false,'selectedFrontierTrackId',uint64(0), ...
    'selectedCandidateId',uint64(0),'frontierDiagnostics',struct(),'viewpointDiagnostics',struct(), ...
    'decisionEvidence',struct(),'visibleUnknownCount',0,'recoveryTravelCost_m',inf,'recoveryGoalProgress_m',0, ...
    'recoveryDiagnostics',diag);
end

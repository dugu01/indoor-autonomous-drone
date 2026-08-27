function selection = select_safe_landing_zone_S2_2(cfg,grid,currentXY,candidates)
% SELECT_SAFE_LANDING_ZONE_S2_2 Home-preferred safe landing selection.
% Candidate 1 is the home point. It is selected whenever it is both clear
% and reachable. Alternate sites are ranked by path length only when home
% is unavailable. The inflated grid already represents centre-safe cells.

selection=struct('valid',false,'xy',[nan nan],'candidateIndex',0, ...
    'path',zeros(0,2),'pathLength_m',inf,'astarExpanded',0,'tested',0);
if isempty(candidates),return;end
candidates=double(candidates);if size(candidates,2)~=2,return;end
maxCandidates=min(size(candidates,1),cfg.landingSelectionMaxCandidates);
landingRadius=cfg.landingZoneExtraMargin_m;

% Home-preferred first pass.
xy=candidates(1,:);selection.tested=1;
if landing_zone_clear_S2_2(grid,xy,landingRadius)
    [raw,info]=astar_grid_S2_2(grid,currentXY,xy);
    selection.astarExpanded=selection.astarExpanded+info.expanded;
    if ~isempty(raw)
        selection.valid=true;selection.xy=xy;selection.candidateIndex=1;
        selection.path=raw;selection.pathLength_m=sum(vecnorm(diff(raw,1,1),2,2));
        return;
    end
end

% Home is blocked/unreachable: choose the shortest safe alternate.
for i=2:maxCandidates
    xy=candidates(i,:);selection.tested=selection.tested+1;
    if ~landing_zone_clear_S2_2(grid,xy,landingRadius),continue;end
    [raw,info]=astar_grid_S2_2(grid,currentXY,xy);
    selection.astarExpanded=selection.astarExpanded+info.expanded;
    if isempty(raw),continue;end
    d=sum(vecnorm(diff(raw,1,1),2,2));
    if d<selection.pathLength_m
        selection.valid=true;selection.xy=xy;selection.candidateIndex=i;
        selection.path=raw;selection.pathLength_m=d;
    end
end
end

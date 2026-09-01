function selection = select_safe_landing_zone_S2_3(cfg,grid,currentXY,candidates,t)
% SELECT_SAFE_LANDING_ZONE_S2_3 Require observed-free, fresh landing area.
selection=struct('valid',false,'xy',[nan nan],'candidateIndex',0,'path',zeros(0,2), ...
    'pathLength_m',inf,'astarExpanded',0,'tested',0);
if isempty(candidates),return;end
for i=1:min(size(candidates,1),cfg.landingSelectionMaxCandidates)
    selection.tested=selection.tested+1;xy=candidates(i,:);
    if ~landing_zone_fresh(grid,xy,cfg.landingZoneExtraMargin_m,t,cfg.mapLandingFreshness_s),continue;end
    [raw,info]=astar_grid_S2_2(grid,currentXY,xy);selection.astarExpanded=selection.astarExpanded+info.expanded;
    if isempty(raw),continue;end
    d=sum(vecnorm(diff(raw,1,1),2,2));
    if i==1||d<selection.pathLength_m
        selection.valid=true;selection.xy=xy;selection.candidateIndex=i;selection.path=raw;selection.pathLength_m=d;
        if i==1,return;end
    end
end
end
function ok=landing_zone_fresh(grid,xy,radius,t,maxAge)
ok=landing_zone_clear_S2_2(grid,xy,radius);if ~ok,return;end
res=grid.resolution;ix0=round(xy(1)/res)+1;iy0=round(xy(2)/res)+1;r=ceil(radius/res)+1;
for iy=max(1,iy0-r):min(grid.ny,iy0+r)
    for ix=max(1,ix0-r):min(grid.nx,ix0+r)
        p=[grid.xs(ix) grid.ys(iy)];
        if norm(p-xy)<=radius+0.5*sqrt(2)*res
            if ~grid.knownFree(iy,ix)||t-grid.lastObservedXY(iy,ix)>maxAge,ok=false;return;end
        end
    end
end
end

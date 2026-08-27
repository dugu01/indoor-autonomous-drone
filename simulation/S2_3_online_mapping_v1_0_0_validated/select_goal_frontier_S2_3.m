function choice = select_goal_frontier_S2_3(cfg,grid,startXY,goalXY)
% SELECT_GOAL_FRONTIER_S2_3 Goal-directed known-free observation endpoint.
choice=struct('valid',false,'xy',[nan nan],'path',zeros(0,2), ...
    'progress_m',0,'score',inf,'astarExpanded',0);
frontier=false(grid.ny,grid.nx);
for iy=2:grid.ny-1
    for ix=2:grid.nx-1
        if grid.occ(iy,ix)||~grid.knownFree(iy,ix),continue;end
        nUnknown=nnz(grid.unknownInflated(iy-1:iy+1,ix-1:ix+1));
        if nUnknown>=cfg.mapFrontierUnknownNeighborCount,frontier(iy,ix)=true;end
    end
end
[fy,fx]=find(frontier);if isempty(fx),return;end
startDist=norm(startXY(:).'-goalXY(:).');
% Rank cheaply, then run A* only for the most promising candidates.
scores=zeros(numel(fx),1);
for i=1:numel(fx)
    p=[grid.xs(fx(i)) grid.ys(fy(i))];
    progress=startDist-norm(p-goalXY(:).');
    scores(i)=norm(p-startXY(:).')-2.0*progress;
end
[~,order]=sort(scores);order=order(1:min(30,numel(order)));
for oi=1:numel(order)
    i=order(oi);p=[grid.xs(fx(i)) grid.ys(fy(i))];
    progress=startDist-norm(p-goalXY(:).');
    if progress<cfg.mapMinFrontierProgress_m,continue;end
    [raw,info]=astar_grid_S2_2(grid,startXY,p);choice.astarExpanded=choice.astarExpanded+info.expanded;
    if isempty(raw),continue;end
    lengthCost=sum(vecnorm(diff(raw,1,1),2,2));score=lengthCost-2.0*progress;
    if score<choice.score
        choice.valid=true;choice.xy=p;choice.path=raw;choice.progress_m=progress;choice.score=score;
    end
end
end

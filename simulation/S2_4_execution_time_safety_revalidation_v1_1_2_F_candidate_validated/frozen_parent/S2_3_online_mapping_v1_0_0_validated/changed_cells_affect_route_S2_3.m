function affected = changed_cells_affect_route_S2_3(grid,newlyBlockedMask, ...
    traj,trajClock,gridFallbackActive,gridFallbackPath,gridFallbackIndex,currentXY)
% CHANGED_CELLS_AFFECT_ROUTE_S2_3 Replan only when newly blocked inflated
% cells intersect the active future route. The input mask is already
% inflated by vehicle, control, estimator and map margins.
affected=false;
if ~any(newlyBlockedMask(:)),return;end
checkGrid=grid;checkGrid.occ=logical(newlyBlockedMask);
points=double(currentXY(:).');
if gridFallbackActive&&~isempty(gridFallbackPath)
    idx=max(1,min(gridFallbackIndex,size(gridFallbackPath,1)));
    points=[points;double(gridFallbackPath(idx:end,:))]; %#ok<AGROW>
elseif isfield(traj,'valid')&&traj.valid
    if isfield(traj,'sample')&&isfield(traj.sample,'t')
        sample=traj.sample;
    else
        sample=sample_min_snap_trajectory_S2_2(traj,0.05);
    end
    keep=sample.t>=max(0,trajClock)-1e-9;
    points=[points;double(sample.p(keep,:))]; %#ok<AGROW>
else
    % A tracking state without a usable route representation is unsafe to
    % exempt from repair.
    affected=true;return;
end
if size(points,1)==1
    affected=segment_occupied_grid_S2_2(checkGrid,points,points);return;
end
for i=1:size(points,1)-1
    if segment_occupied_grid_S2_2(checkGrid,points(i,:),points(i+1,:))
        affected=true;return;
    end
end
end

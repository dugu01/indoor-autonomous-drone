function [blocked, blockedSegment] = collision_monitor_S2_2(grid, pos, path, pathIndex, lookaheadDistance)
% COLLISION_MONITOR_S2_2  Look ahead along current path for blocked segment.
blocked = false;
blockedSegment = [nan nan nan nan];
if isempty(path) || pathIndex > size(path,1)
    return;
end
remaining = lookaheadDistance;
current = pos(:).';
for j = pathIndex:size(path,1)
    target = path(j,:);
    seg = norm(target-current);
    if seg < 1e-9
        continue;
    end
    if seg <= remaining
        stopPoint = target;
    else
        stopPoint = current + (target-current)*(remaining/seg);
    end
    if segment_occupied_grid_S2_2(grid,current,stopPoint)
        blocked = true;
        blockedSegment = [current stopPoint];
        return;
    end
    remaining = remaining - seg;
    current = target;
    if remaining <= 0
        return;
    end
end
end

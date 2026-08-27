function [path, info] = astar_grid_S2_2(grid, startXY, goalXY)
% ASTAR_GRID_S2_2  8-connected A* over inflated occupancy grid.
% Returns path as N x 2 [x y] in metres, or [] if no path exists.

info = struct('success',false,'expanded',0,'reason','');
path = [];
[sx,sy] = point_to_index_S2_2(grid,startXY);
[gx,gy] = point_to_index_S2_2(grid,goalXY);

if ~in_grid_S2_2(grid,gx,gy) || grid.occ(gy,gx)
    info.reason = 'goal occupied or outside grid';
    return;
end

[sx,sy,ok] = nearest_free_index_S2_2(grid,sx,sy,12);
if ~ok
    info.reason = 'start occupied and no nearby free cell';
    return;
end

ny = grid.ny; nx = grid.nx;
gScore = inf(ny,nx);
fScore = inf(ny,nx);
cameX = zeros(ny,nx);
cameY = zeros(ny,nx);
open = false(ny,nx);
closed = false(ny,nx);

gScore(sy,sx) = 0;
fScore(sy,sx) = hypot(sx-gx, sy-gy);
open(sy,sx) = true;

nbr = [1 0 1; -1 0 1; 0 1 1; 0 -1 1; 1 1 sqrt(2); 1 -1 sqrt(2); -1 1 sqrt(2); -1 -1 sqrt(2)];

while any(open(:))
    tmp = fScore;
    tmp(~open) = inf;
    [~,idx] = min(tmp(:));
    [cy,cx] = ind2sub(size(tmp),idx);
    open(cy,cx) = false;
    closed(cy,cx) = true;
    info.expanded = info.expanded + 1;
    if cx == gx && cy == gy
        path = reconstruct_path_S2_2(grid,cameX,cameY,cx,cy,sx,sy);
        info.success = true;
        info.reason = 'success';
        return;
    end
    for k = 1:size(nbr,1)
        vx = cx + nbr(k,1);
        vy = cy + nbr(k,2);
        if ~in_grid_S2_2(grid,vx,vy) || grid.occ(vy,vx) || closed(vy,vx)
            continue;
        end
        % prevent diagonal corner cutting
        if nbr(k,1) ~= 0 && nbr(k,2) ~= 0
            if grid.occ(cy,vx) || grid.occ(vy,cx)
                continue;
            end
        end
        tentative = gScore(cy,cx) + nbr(k,3);
        if ~open(vy,vx) || tentative < gScore(vy,vx)
            cameX(vy,vx) = cx;
            cameY(vy,vx) = cy;
            gScore(vy,vx) = tentative;
            fScore(vy,vx) = tentative + hypot(vx-gx,vy-gy);
            open(vy,vx) = true;
        end
    end
end
info.reason = 'open set exhausted';
end

function path = reconstruct_path_S2_2(grid,cameX,cameY,cx,cy,sx,sy)
pts = zeros(0,2);
while true
    pts(end+1,:) = index_to_point_S2_2(grid,cx,cy); %#ok<AGROW>
    if cx == sx && cy == sy, break; end
    px = cameX(cy,cx); py = cameY(cy,cx);
    if px == 0 || py == 0, break; end
    cx = px; cy = py;
end
path = flipud(pts);
end

function [ix,iy] = point_to_index_S2_2(grid,p)
ix = round(p(1)/grid.resolution) + 1;
iy = round(p(2)/grid.resolution) + 1;
end

function p = index_to_point_S2_2(grid,ix,iy)
p = [(ix-1)*grid.resolution, (iy-1)*grid.resolution];
end

function tf = in_grid_S2_2(grid,ix,iy)
tf = ix >= 1 && iy >= 1 && ix <= grid.nx && iy <= grid.ny;
end

function [bx,by,ok] = nearest_free_index_S2_2(grid,ix0,iy0,maxCells)
ok = false; bx = ix0; by = iy0;
if in_grid_S2_2(grid,ix0,iy0) && ~grid.occ(iy0,ix0)
    ok = true; return;
end
bestD2 = inf;
for r = 1:maxCells
    for iy = max(1,iy0-r):min(grid.ny,iy0+r)
        for ix = max(1,ix0-r):min(grid.nx,ix0+r)
            if grid.occ(iy,ix), continue; end
            d2 = (ix-ix0)^2 + (iy-iy0)^2;
            if d2 < bestD2
                bestD2 = d2; bx = ix; by = iy; ok = true;
            end
        end
    end
    if ok, return; end
end
end

function occupied = segment_occupied_grid_S2_2(grid, a, b)
% SEGMENT_OCCUPIED_GRID_S2_2
% Conservative supercover traversal of every occupancy-grid cell touched
% by the line segment from a to b.
%
% Grid indexing is consistent with the rest of Stage S2.2:
%   ix = round(x / resolution) + 1
%   iy = round(y / resolution) + 1

% Fail closed on malformed input.
a = double(a(:).');
b = double(b(:).');
if numel(a) ~= 2 || numel(b) ~= 2 || ...
        any(~isfinite(a)) || any(~isfinite(b)) || ...
        ~isfield(grid,'resolution') || grid.resolution <= 0
    occupied = true;
    return;
end

res = grid.resolution;

% Convert physical coordinates to continuous zero-based cell coordinates.
% The +0.5 offset makes floor(q) equivalent to round(x/res) for the
% non-negative room coordinates used by this project.
qa = a / res + 0.5;
qb = b / res + 0.5;

cx = floor(qa(1));
cy = floor(qa(2));
endX = floor(qb(1));
endY = floor(qb(2));

if cell_is_occupied(cx,cy,grid)
    occupied = true;
    return;
end

 dx = qb(1)-qa(1);
 dy = qb(2)-qa(2);
stepX = sign(dx);
stepY = sign(dy);

if stepX ~= 0
    if stepX > 0, nextBoundaryX = cx+1; else, nextBoundaryX = cx; end
    tMaxX = (nextBoundaryX-qa(1))/dx;
    tDeltaX = 1/abs(dx);
else
    tMaxX = inf;
    tDeltaX = inf;
end

if stepY ~= 0
    if stepY > 0, nextBoundaryY = cy+1; else, nextBoundaryY = cy; end
    tMaxY = (nextBoundaryY-qa(2))/dy;
    tDeltaY = 1/abs(dy);
else
    tMaxY = inf;
    tDeltaY = inf;
end

occupied = false;
tol = 1e-12;
visitCount = 0;
maxVisits = 4*(grid.nx+grid.ny)+20;

while cx ~= endX || cy ~= endY
    if tMaxX < tMaxY-tol
        cx = cx+stepX;
        tMaxX = tMaxX+tDeltaX;
        if cell_is_occupied(cx,cy,grid), occupied=true; return; end

    elseif tMaxY < tMaxX-tol
        cy = cy+stepY;
        tMaxY = tMaxY+tDeltaY;
        if cell_is_occupied(cx,cy,grid), occupied=true; return; end

    else
        % Exact corner crossing: conservatively inspect both side cells and
        % the diagonal cell. Touching an inflated occupied cell is blocked.
        nextX = cx+stepX;
        nextY = cy+stepY;
        if stepX ~= 0 && cell_is_occupied(nextX,cy,grid), occupied=true; return; end
        if stepY ~= 0 && cell_is_occupied(cx,nextY,grid), occupied=true; return; end
        cx = nextX;
        cy = nextY;
        tMaxX = tMaxX+tDeltaX;
        tMaxY = tMaxY+tDeltaY;
        if cell_is_occupied(cx,cy,grid), occupied=true; return; end
    end

    visitCount = visitCount+1;
    if visitCount > maxVisits
        occupied = true;
        return;
    end
end
end

function occupied = cell_is_occupied(cx,cy,grid)
% cx and cy are zero-based cell indices.
ix = cx+1;
iy = cy+1;
occupied = ix < 1 || ix > grid.nx || iy < 1 || iy > grid.ny || ...
    grid.occ(iy,ix);
end

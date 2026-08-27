function occupied = segment_occupied_grid_S2_2(grid, a, b)
% SEGMENT_OCCUPIED_GRID_S2_2  Check if a line segment crosses occupied grid cells.
step = grid.resolution/2;
d = norm(b-a);
n = max(2,ceil(d/step));
occupied = false;
for k = 0:n
    u = k/n;
    p = a*(1-u) + b*u;
    ix = round(p(1)/grid.resolution) + 1;
    iy = round(p(2)/grid.resolution) + 1;
    if ix < 1 || iy < 1 || ix > grid.nx || iy > grid.ny || grid.occ(iy,ix)
        occupied = true;
        return;
    end
end
end

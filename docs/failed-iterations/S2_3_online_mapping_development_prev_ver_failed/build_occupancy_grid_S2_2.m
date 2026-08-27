function grid = build_occupancy_grid_S2_2(cfg, obstacles)
% BUILD_OCCUPANCY_GRID_S2_2  Build inflated 2-D occupancy grid.
% The grid stores occupancy for the vehicle centre point. Obstacles and room
% walls are inflated by cfg.inflationRadius.

if nargin < 2 || isempty(obstacles), obstacles = zeros(0,4); end
res = cfg.gridResolution;
nx = round(cfg.room(1)/res) + 1;
ny = round(cfg.room(2)/res) + 1;
occ = false(ny,nx);
xs = (0:nx-1)*res;
ys = (0:ny-1)*res;
r = cfg.inflationRadius;

for iy = 1:ny
    y = ys(iy);
    for ix = 1:nx
        x = xs(ix);
        if x < r || x > cfg.room(1)-r || y < r || y > cfg.room(2)-r
            occ(iy,ix) = true;
        end
    end
end

for k = 1:size(obstacles,1)
    occ = inflate_obstacles_S2_2(occ, cfg, obstacles(k,:), r);
end

grid = struct('occ',occ,'xs',xs,'ys',ys,'nx',nx,'ny',ny, ...
    'resolution',res,'obstacles',obstacles,'inflationRadius',r,'room',cfg.room);
end

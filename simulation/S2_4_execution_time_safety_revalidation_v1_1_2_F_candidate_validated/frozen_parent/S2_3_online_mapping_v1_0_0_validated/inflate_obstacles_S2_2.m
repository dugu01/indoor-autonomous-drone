function occ = inflate_obstacles_S2_2(occ, cfg, rect, radius)
% INFLATE_OBSTACLES_S2_2  Mark an inflated rectangular obstacle in grid cells.
if nargin < 4, radius = cfg.inflationRadius; end
res = cfg.gridResolution;
[ny,nx] = size(occ);

x0 = max(0, rect(1)-radius);
x1 = min(cfg.room(1), rect(1)+rect(3)+radius);
y0 = max(0, rect(2)-radius);
y1 = min(cfg.room(2), rect(2)+rect(4)+radius);

ix0 = max(1, floor(x0/res)+1);
ix1 = min(nx, ceil(x1/res)+1);
iy0 = max(1, floor(y0/res)+1);
iy1 = min(ny, ceil(y1/res)+1);
occ(iy0:iy1, ix0:ix1) = true;
end

function tf = continuous_collision_S2_2(p, cfg, obstacles)
% CONTINUOUS_COLLISION_S2_2  Check inflated wall/obstacle collision in metres.
r = cfg.inflationRadius;
wallClear = min([p(1), cfg.room(1)-p(1), p(2), cfg.room(2)-p(2)]);
if wallClear < r - 1e-3
    tf = true;
    return;
end
for i = 1:size(obstacles,1)
    if dist_point_rect_S2_2(p,obstacles(i,:)) < r - 1e-3
        tf = true;
        return;
    end
end
tf = false;
end

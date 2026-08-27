function [minWall, minObs] = min_clearance_S2_2(P,cfg,obstacles)
% MIN_CLEARANCE_S2_2  Raw geometric clearance, before subtracting inflation.
minWall = inf;
minObs = inf;
for k = 1:size(P,1)
    p = P(k,:);
    minWall = min(minWall,min([p(1),cfg.room(1)-p(1),p(2),cfg.room(2)-p(2)]));
    for j = 1:size(obstacles,1)
        minObs = min(minObs,dist_point_rect_S2_2(p,obstacles(j,:)));
    end
end
if isempty(obstacles), minObs = inf; end
end

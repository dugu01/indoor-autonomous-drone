function [pos, vel, pathIndex, trackingError] = track_trajectory_S2_2(cfg, pos, vel, path, pathIndex)
% TRACK_TRAJECTORY_S2_2  One kinematic tracking step with acceleration limits.
if isempty(path)
    trackingError = nan;
    vel(:) = 0;
    return;
end
while pathIndex < size(path,1) && norm(pos(:).'-path(pathIndex,:)) < cfg.waypointTolerance_m
    pathIndex = pathIndex + 1;
end
target = path(min(pathIndex,size(path,1)),:);
err = target(:) - pos(:);
trackingError = norm(err);
if trackingError > 1e-9
    desiredVel = cfg.maxSpeedXY_mps * err / trackingError;
else
    desiredVel = [0;0];
end
accel = (desiredVel - vel(:)) / cfg.dt;
accNorm = norm(accel);
if accNorm > cfg.maxAccelXY_mps2
    accel = accel * (cfg.maxAccelXY_mps2/accNorm);
end
vel = vel(:) + accel * cfg.dt;
speed = norm(vel);
if speed > cfg.maxSpeedXY_mps
    vel = vel * (cfg.maxSpeedXY_mps/speed);
end
pos = pos(:) + vel * cfg.dt;
end

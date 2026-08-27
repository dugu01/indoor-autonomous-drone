function traj = generate_strict_trajectory_S2_3(cfg,grid,path,startVelocity,startAcceleration,initialScale)
% GENERATE_STRICT_TRAJECTORY_S2_3 Preserve the S2.2 generator but enforce
% the unchanged hard S2.2 kinematic limits at the S2.3 adapter boundary.
if nargin<6||isempty(initialScale),initialScale=1.0;end
scale=max(initialScale,eps);traj=struct('valid',false);
for attempt=1:5
    traj=generate_min_snap_trajectory_S2_2(cfg,grid,path,startVelocity,startAcceleration,scale);
    if ~isfield(traj,'valid')||~traj.valid,return;end
    ratios=[traj.maxSpeed_mps/max(cfg.maxSpeedXY_mps,eps), ...
        sqrt(traj.maxAccel_mps2/max(cfg.maxAccelXY_mps2,eps)), ...
        nthroot(traj.maxJerk_mps3/max(cfg.maxJerkXY_mps3,eps),3)];
    if all(ratios<=1+1e-9),return;end
    scale=scale*1.002*max(ratios);
end
traj.valid=false;
end

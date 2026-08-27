function [desiredVelocity,trackingError,ref] = track_smooth_trajectory_S2_2(cfg,pos,vel,traj,trajClock)
% TRACK_SMOOTH_TRAJECTORY_S2_2 Feedforward + PD reference tracking command.
ref=sample_min_snap_state_S2_2(traj,trajClock);
positionError=ref.p-pos(:);velocityError=ref.v-vel(:);
trackingError=norm(positionError);
accelCommand=ref.a+cfg.trajectoryKp*positionError+cfg.trajectoryKd*velocityError;
if norm(accelCommand)>cfg.maxAccelXY_mps2
    accelCommand=accelCommand*(cfg.maxAccelXY_mps2/norm(accelCommand));
end
desiredVelocity=vel(:)+accelCommand*cfg.dt;
if norm(desiredVelocity)>cfg.maxSpeedXY_mps
    desiredVelocity=desiredVelocity*(cfg.maxSpeedXY_mps/norm(desiredVelocity));
end
end

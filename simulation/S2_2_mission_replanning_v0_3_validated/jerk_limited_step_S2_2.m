function [posNew,velNew,accNew,jerk] = jerk_limited_step_S2_2(cfg,pos,vel,acc,velocityCommand)
% JERK_LIMITED_STEP_S2_2 Point-mass update constrained in v, a and jerk.
targetAcceleration=(velocityCommand(:)-vel(:))/cfg.dt;
if norm(targetAcceleration)>cfg.maxAccelXY_mps2
    targetAcceleration=targetAcceleration*(cfg.maxAccelXY_mps2/norm(targetAcceleration));
end
deltaAcceleration=targetAcceleration-acc(:);maxDelta=cfg.maxJerkXY_mps3*cfg.dt;
if norm(deltaAcceleration)>maxDelta
    deltaAcceleration=deltaAcceleration*(maxDelta/norm(deltaAcceleration));
end
accNew=acc(:)+deltaAcceleration;
if norm(accNew)>cfg.maxAccelXY_mps2
    accNew=accNew*(cfg.maxAccelXY_mps2/norm(accNew));
end
jerk=(accNew-acc(:))/cfg.dt;
velNew=vel(:)+accNew*cfg.dt;
if norm(velNew)>cfg.maxSpeedXY_mps
    velNew=velNew*(cfg.maxSpeedXY_mps/norm(velNew));
end
posNew=pos(:)+velNew*cfg.dt;
end

function [vCmd,aCmd] = shape_velocity_command_S2_2(cfg,vCmd,aCmd,targetVelocity)
% SHAPE_VELOCITY_COMMAND_S2_2 Acceleration/jerk-limited XY command filter.
% This shapes the finite-candidate velocity-obstacle output before it is
% sent to the 6-DOF controller.

vCmd=vCmd(:);aCmd=aCmd(:);targetVelocity=targetVelocity(:);
if numel(vCmd)~=2||numel(aCmd)~=2||numel(targetVelocity)~=2|| ...
        any(~isfinite([vCmd;aCmd;targetVelocity]))
    vCmd=zeros(2,1);aCmd=zeros(2,1);return;
end

rawA=(targetVelocity-vCmd)/cfg.dt;
if norm(rawA)>cfg.safetyVelocityAccelLimit_mps2
    rawA=rawA*(cfg.safetyVelocityAccelLimit_mps2/norm(rawA));
end

deltaA=rawA-aCmd;
maxDelta=cfg.safetyVelocityJerkLimit_mps3*cfg.dt;
if norm(deltaA)>maxDelta
    deltaA=deltaA*(maxDelta/norm(deltaA));
end
aCmd=aCmd+deltaA;
if norm(aCmd)>cfg.safetyVelocityAccelLimit_mps2
    aCmd=aCmd*(cfg.safetyVelocityAccelLimit_mps2/norm(aCmd));
end

vCmd=vCmd+aCmd*cfg.dt;
if norm(vCmd)>cfg.maxSpeedXY_mps
    vCmd=vCmd*(cfg.maxSpeedXY_mps/norm(vCmd));
end

end

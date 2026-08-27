function [p,v,active] = dynamic_obstacle_state_S2_2(obs,t)
% DYNAMIC_OBSTACLE_STATE_S2_2  Piecewise constant-velocity obstacle truth.
active = t >= obs.appearTime && t <= obs.disappearTime;
if ~active, p=[nan nan];v=[nan nan];return;end
moveEnd=min(t,obs.stopTime);
p=obs.start + obs.velocity*max(0,moveEnd-obs.appearTime);
if t < obs.stopTime, v=obs.velocity; else, v=[0 0]; end
end

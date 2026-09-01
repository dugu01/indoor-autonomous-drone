function state = init_quadrotor_state_S2_2(cfg,startXY,startAltitude)
% INIT_QUADROTOR_STATE_S2_2 Initialise airborne or landed F450 state.
airborneDefault=nargin<3||isempty(startAltitude);
if airborneDefault,startAltitude=cfg.altitudeNominal_m;end
initialThrust=0;if airborneDefault,initialThrust=cfg.mass_kg*norm(cfg.gW);end
state=struct('p',[startXY(:);startAltitude],'v',zeros(3,1), ...
    'q',[1 0 0 0],'omega',zeros(3,1),'a',zeros(3,1),'omegaDot',zeros(3,1), ...
    'thrust_N',initialThrust,'moment_Nm',zeros(3,1),'onGround',false);
if isfield(cfg,'groundHeight_m')&&startAltitude<=cfg.groundHeight_m+1e-9
    state.p(3)=cfg.groundHeight_m;state.onGround=true;
end
end

function state = quadrotor_dynamics_S2_2(cfg,state,cmd)
% QUADROTOR_DYNAMICS_S2_2 6-DOF rigid-body F450 simulation step.
% v0.5 adds a simple non-penetrating ground-contact model for autonomous
% takeoff and landing. It is not a wheel/leg impact model.

R=q2R_S2_2(state.q);
thrust=max(0,min(cfg.thrustToWeight*cfg.mass_kg*norm(cfg.gW),cmd.thrust_N));
M=max(-cfg.maxMoment_Nm,min(cfg.maxMoment_Nm,cmd.moment_Nm(:)));
a=R*[0;0;thrust]/cfg.mass_kg+cfg.gW-cfg.linearDrag*state.v;
omegaDot=cfg.inertia_kgm2\(M-cross(state.omega,cfg.inertia_kgm2*state.omega)-cfg.angularDrag*state.omega);
state.p=state.p+state.v*cfg.dt+0.5*a*cfg.dt^2;
state.v=state.v+a*cfg.dt;
omegaMid=state.omega+0.5*omegaDot*cfg.dt;
state.q=qmul_S2_2(state.q,qexp_S2_2((omegaMid*cfg.dt).'));
state.omega=state.omega+omegaDot*cfg.dt;
state.a=a;
state.omegaDot=omegaDot;
state.thrust_N=thrust;
state.moment_Nm=M;
state.onGround=false;

if isfield(cfg,'groundHeight_m')&&state.p(3)<=cfg.groundHeight_m
    state.p(3)=cfg.groundHeight_m;
    if state.v(3)<0,state.v(3)=0;end
    % A stationary landed vehicle has zero world-frame translational
    % acceleration; its accelerometer still measures specific force +g.
    if thrust<=1.05*cfg.mass_kg*norm(cfg.gW)
        state.v(1:2)=0.98*state.v(1:2);
        state.a=zeros(3,1);
        state.onGround=true;
    end
end
end

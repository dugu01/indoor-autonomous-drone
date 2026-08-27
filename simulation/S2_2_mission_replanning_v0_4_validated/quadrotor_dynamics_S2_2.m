function state = quadrotor_dynamics_S2_2(cfg,state,cmd)
% QUADROTOR_DYNAMICS_S2_2 6-DOF rigid-body F450 simulation step.
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
end

function track = alpha_beta_track_S2_2(cfg,track,z,t)
% ALPHA_BETA_TRACK_S2_2  Lightweight constant-velocity obstacle tracker.
if isempty(track) || ~isfield(track,'initialized') || ~track.initialized
    track=struct('initialized',true,'p',z(:).','v',[0 0],'t',t);
    return;
end
dt=max(1e-3,t-track.t);
pred=track.p+track.v*dt;
r=z(:).'-pred;
track.p=pred+cfg.trackerAlpha*r;
track.v=track.v+(cfg.trackerBeta/dt)*r;
track.t=t;
end

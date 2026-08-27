function track = alpha_beta_track_S2_2(cfg,track,z,t)
% ALPHA_BETA_TRACK_S2_2 Constant-velocity obstacle tracker.
% Position uses an alpha-beta correction. A separately low-pass-filtered
% velocity is used by collision prediction and stopped-object promotion so
% 50 Hz position noise cannot repeatedly reset the persistence timer.

z=z(:).';
if isempty(track) || ~isfield(track,'initialized') || ~track.initialized
    track=struct('initialized',true,'p',z,'v',[0 0], ...
        'vFiltered',[0 0],'speedFiltered',0,'t',t,'updates',1);
    return;
end

dt=max(1e-3,t-track.t);
pred=track.p+track.v*dt;
r=z-pred;
track.p=pred+cfg.trackerAlpha*r;
track.v=track.v+(cfg.trackerBeta/dt)*r;

alphaV=cfg.trackerVelocityFilterAlpha;
if ~isfield(track,'vFiltered')||numel(track.vFiltered)~=2
    track.vFiltered=track.v;
end
track.vFiltered=(1-alphaV)*track.vFiltered+alphaV*track.v;
track.speedFiltered=norm(track.vFiltered);
track.t=t;
if ~isfield(track,'updates'),track.updates=1;end
track.updates=track.updates+1;
end

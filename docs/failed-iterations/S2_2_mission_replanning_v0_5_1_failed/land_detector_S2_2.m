function [detected,contactTimer] = land_detector_S2_2(cfg,packet,est,contactTimer)
% LAND_DETECTOR_S2_2 Simulated contact-confirmed landing detector.
%
% Mission logic does not inspect truth position or truth velocity directly.
% Ground contact is exposed as a simulated onboard contact signal and must
% persist while the selected ESKF reports a small vertical speed.

if nargin<4||isempty(contactTimer),contactTimer=0;end
contact=logical(isfield(packet,'groundContact')&&packet.groundContact);
verticalSlow=all(isfinite(est.v))&&abs(est.v(3))<=cfg.landedVerticalSpeed_mps;
nearGround=all(isfinite(est.p))&&est.p(3)<= ...
    cfg.groundHeight_m+cfg.landingAltitudeTolerance_m+0.05;

if contact&&verticalSlow&&nearGround
    contactTimer=contactTimer+cfg.dt;
else
    contactTimer=0;
end
detected=contactTimer>=cfg.landContactConfirmTime_s;
end

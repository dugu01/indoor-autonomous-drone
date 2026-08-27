function [detected,contactTimer] = land_detector_S2_2( ...
    cfg,packet,est,contactTimer,previousDetected,landingDetectionArmed)
% LAND_DETECTOR_S2_2 Gated, contact-confirmed touchdown latch.
%
% A landing is accepted only after the commanded vertical profile is
% complete and an explicit simulated ground-contact pulse coincides with a
% near-ground, low-vertical-speed estimate.  The event is then latched.
% Requiring uninterrupted contact for several samples is inappropriate for
% the current rigid ground model: its hover controller can lift the vehicle
% a few millimetres after a physically valid first touchdown.
%
% Truth is not read here.  packet.groundContact is the simulated onboard
% contact signal produced by simulate_sensor_packet_S2_2.

if nargin<4||isempty(contactTimer),contactTimer=0;end
if nargin<5||isempty(previousDetected),previousDetected=false;end
if nargin<6||isempty(landingDetectionArmed),landingDetectionArmed=true;end

if previousDetected
    detected=true;
    contactTimer=max(contactTimer,cfg.landContactConfirmTime_s);
    return;
end

contact=logical(isfield(packet,'groundContact')&&packet.groundContact);
verticalSlow=all(isfinite(est.v))&&abs(est.v(3))<=cfg.landedVerticalSpeed_mps;
nearGround=all(isfinite(est.p))&&est.p(3)<= ...
    cfg.groundHeight_m+cfg.landingAltitudeTolerance_m+0.05;
qualifiedTouchdown=logical(landingDetectionArmed)&&contact&&verticalSlow&&nearGround;

if qualifiedTouchdown
    % The contact event is already strongly gated by profile completion,
    % altitude and vertical speed. Latch it so model rebound cannot erase a
    % valid touchdown before the state machine cuts motor command.
    detected=true;
    contactTimer=cfg.landContactConfirmTime_s;
else
    detected=false;
    contactTimer=0;
end
end

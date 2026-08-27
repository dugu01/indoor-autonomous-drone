function cmd = geometric_controller_S2_2(cfg,est,ref,previousAccelCmd)
% GEOMETRIC_CONTROLLER_S2_2 Near-hover SE(3)-consistent tracking controller.
% The translational command is horizontally acceleration-limited,
% jerk-limited, and guarded by a soft executed-speed envelope.

if nargin<4||isempty(previousAccelCmd),previousAccelCmd=zeros(3,1);end
previousAccelCmd=previousAccelCmd(:);

ep=ref.p(:)-est.p(:);
ev=ref.v(:)-est.v(:);
refAccel=ref.a(:);
if isfield(ref,'horizontalControlEnabled')&&~ref.horizontalControlEnabled
    % With unobservable XY position, never command a return to a stale
    % horizontal coordinate. A short-term inertial velocity estimate is still
    % used only for damping when explicitly enabled, so residual translational
    % motion is arrested before/during the local descent.
    ep(1:2)=0;
    feedforwardEnabled=isfield(ref,'horizontalFeedforwardAccelEnabled')&& ...
        logical(ref.horizontalFeedforwardAccelEnabled);
    if ~feedforwardEnabled
        refAccel(1:2)=0;
    end
    dampingEnabled=isfield(ref,'horizontalVelocityDampingEnabled')&& ...
        logical(ref.horizontalVelocityDampingEnabled);
    if ~dampingEnabled
        ev(1:2)=0;
    end
end
velocityGain=cfg.velocityKd;
if isfield(ref,'horizontalControlEnabled')&&~ref.horizontalControlEnabled&& ...
        isfield(ref,'horizontalVelocityDampingEnabled')&& ...
        logical(ref.horizontalVelocityDampingEnabled)&& ...
        isfield(ref,'horizontalVelocityDampingGain')&& ...
        isfinite(ref.horizontalVelocityDampingGain)
    % During a confidence-limited position-loss brake, use a dedicated
    % horizontal velocity-damping gain. Absolute XY position remains disabled.
    velocityGain(1:2)=ref.horizontalVelocityDampingGain;
end
aCmd=refAccel+cfg.positionKp.*ep+velocityGain.*ev;

% Separate horizontal and vertical limits preserve near-hover validity.
h=norm(aCmd(1:2));
if h>cfg.maxHorizontalCommandAccel_mps2
    aCmd(1:2)=aCmd(1:2)*(cfg.maxHorizontalCommandAccel_mps2/h);
end
aCmd(3)=max(-cfg.maxVerticalCommandAccel_mps2, ...
    min(cfg.maxVerticalCommandAccel_mps2,aCmd(3)));

% Begin removing positive acceleration along the current velocity before
% the hard executed-speed limit is reached. This is especially important
% during REJOIN, where position error can otherwise keep requesting forward
% acceleration after the reference velocity itself has been capped.
aCmd(1:2)=guard_horizontal_speed(cfg,est.v(1:2),aCmd(1:2));

% Limit the change of commanded acceleration. This prevents a discrete VO
% candidate switch from becoming an impulsive body-tilt command.
deltaA=aCmd-previousAccelCmd;
maxDelta=cfg.maxControllerJerk_mps3*cfg.dt;
if norm(deltaA)>maxDelta
    deltaA=deltaA*(maxDelta/norm(deltaA));
    aCmd=previousAccelCmd+deltaA;
end

% Reapply limits after jerk shaping.
h=norm(aCmd(1:2));
if h>cfg.maxHorizontalCommandAccel_mps2
    aCmd(1:2)=aCmd(1:2)*(cfg.maxHorizontalCommandAccel_mps2/h);
end
aCmd(3)=max(-cfg.maxVerticalCommandAccel_mps2, ...
    min(cfg.maxVerticalCommandAccel_mps2,aCmd(3)));

% Final non-accelerating guard. It only removes an unsafe tangential
% component; it never adds outward acceleration.
aCmd(1:2)=guard_horizontal_speed(cfg,est.v(1:2),aCmd(1:2));

F=cfg.mass_kg*(aCmd-cfg.gW);
if norm(F)<1e-9,F=cfg.mass_kg*(-cfg.gW);end
b3=F/norm(F);

% Final geometric tilt guard.
maxHorizontal=tan(cfg.maxTilt_rad)*max(b3(3),1e-6);
h=norm(b3(1:2));
if h>maxHorizontal
    b3(1:2)=b3(1:2)*(maxHorizontal/h);
    b3=b3/norm(b3);
end

yaw=ref.yaw;
b1c=[cos(yaw);sin(yaw);0];
b2=cross(b3,b1c);
if norm(b2)<1e-6,b2=[0;1;0];end
b2=b2/norm(b2);
b1=cross(b2,b3);
Rd=[b1,b2,b3];
R=q2R_S2_2(est.q);
E=Rd'*R-R'*Rd;
eR=0.5*[E(3,2);E(1,3);E(2,1)];
omega=est.omega(:);
M=-cfg.attitudeKp.*eR-cfg.rateKd.*omega+ ...
    cross(omega,cfg.inertia_kgm2*omega);
thrust=dot(F,R(:,3));
thrust=max(0,min(cfg.thrustToWeight*cfg.mass_kg*norm(cfg.gW),thrust));
M=max(-cfg.maxMoment_Nm,min(cfg.maxMoment_Nm,M));
cmd=struct('thrust_N',thrust,'moment_Nm',M,'aCmd',aCmd, ...
    'Rd',Rd,'eR',eR,'tiltCmd_rad',acos(max(-1,min(1,Rd(3,3)))));
end

function aXY=guard_horizontal_speed(cfg,vXY,aXY)
vXY=vXY(:);aXY=aXY(:);
speed=norm(vXY);
if speed<1e-8||speed<cfg.executedSpeedSoftLimit_mps
    return;
end

dir=vXY/speed;
tangential=dot(aXY,dir);
allowed=cfg.speedGuardGain*(cfg.maxExecutedSpeed_mps-speed);
allowed=min(cfg.maxHorizontalCommandAccel_mps2,allowed);
if speed>=cfg.maxExecutedSpeed_mps
    allowed=min(allowed,-cfg.speedGuardBrake_mps2);
end
if tangential>allowed
    aXY=aXY+(allowed-tangential)*dir;
end
end

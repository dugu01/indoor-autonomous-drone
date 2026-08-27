function results = run_S2_lidar_slam(seed, runStress, makePlots, makeAnimation)
% RUN_S2_LIDAR_SLAM
% Hardware-oriented 6-DOF indoor-drone navigation simulation.
%
% Architecture
%   Local/control frame (continuous, causal):
%     200 Hz IMU -> quaternion 16-error-state ESKF
%     D435i host-generated VIO -> p,v,q update at 30 Hz
%     RPLidar A1 scan-to-local-map ICP -> x,y,yaw update at 5.5 Hz
%     VL53L0X/downward range -> tilt/lever-arm-aware update at 30 Hz
%     Barometer -> altitude + estimated barometer-bias update at 25 Hz
%
%   Global/map frame (not fed as a jump into control):
%     keyframes -> ScanContext candidate -> pairwise ICP verification
%     -> robust SE(2) pose graph -> smooth map-to-local correction.
%
% The D435i does not natively output a full VIO pose. In hardware, vio.*
% must come from a host VIO pipeline using D435i imagery/depth/IMU.
%
% Plotting/output
%   Put plot_S2_dashboard.m in the same folder as this file.
%   Figures and numerical trial data are written under ../results/S2_visual_slam/.
%
% Requirements
%   MATLAB R2025a or compatible
%   Statistics and Machine Learning Toolbox: knnsearch
%   Optimization Toolbox: lsqnonlin (global pose graph)
%
% Examples
%   run_S2_lidar_slam
%   run_S2_lidar_slam(4, true, true, true)  % also animate + save MP4
%
% IMPORTANT
%   This is a validated software-in-the-loop estimator architecture. Before
%   untethered flight it still requires time synchronization, calibrated
%   sensor extrinsics, vibration testing, HIL testing, flight-envelope
%   limits, an independent kill switch, and estimator-health failsafes.

narginchk(0,4);
fprintf('[RUN] run_S2_lidar_slam FOUR-ARG VERSION 4.2\n');
if nargin < 1 || isempty(seed), seed = 0; end
if nargin < 2 || isempty(runStress), runStress = true; end
if nargin < 3 || isempty(makePlots), makePlots = true; end
if nargin < 4 || isempty(makeAnimation), makeAnimation = false; end
scriptDir = fileparts(mfilename('fullpath'));
if makePlots && exist(fullfile(scriptDir,'plot_S2_dashboard.m'),'file')~=2
    error(['plot_S2_dashboard.m was not found beside ' ...
           'run_S2_lidar_slam.m.']);
end
if makeAnimation && exist(fullfile(scriptDir,'animate_S2_flight.m'),'file')~=2
    error(['animate_S2_flight.m was not found beside ' ...
           'run_S2_lidar_slam.m.']);
end
addpath(scriptDir,'-begin');
rehash path;
if makePlots
    fprintf('[PLOT] Using: %s\n', which('plot_S2_dashboard'));
end

fprintf('\n============================================================\n');
fprintf(' S2 PRODUCTION 6-DOF LOCAL ESKF + GLOBAL POSE GRAPH\n');
fprintf(' seed=%d | nominal + stress=%d | animation=%d\n', seed, runStress, makeAnimation);
fprintf('============================================================\n\n');

cfg = defaultConfig();
cfg.modelAssetDir = fullfile(scriptDir,'assets','F450');
simulationDir = fileparts(scriptDir);
cfg.resultsRoot = fullfile(simulationDir,'results','S2_visual_slam');
if ~exist(cfg.resultsRoot,'dir'), mkdir(cfg.resultsRoot); end
fprintf('Results root: %s\n\n', cfg.resultsRoot);

nominal = runTrial(seed, false, cfg, makePlots, makeAnimation);
results.nominal = nominal.summary;
results.nominal.animationFile = nominal.animationFile;

if runStress
    stress = runTrial(seed + 100, true, cfg, makePlots, makeAnimation);
    results.stress = stress.summary;
    results.stress.animationFile = stress.animationFile;
else
    stress = [];
end

results.config = cfg;
results.pass = nominal.summary.pass && (~runStress || stress.summary.pass);

fprintf('\n================ FINAL RESULT ================\n');
if results.pass
    fprintf('*** PASS *** 6-DOF estimator remained within %.0f cm requirement.\n', ...
        100*cfg.requirement_m);
else
    fprintf('*** FAIL *** inspect sensor-health, NIS and ICP diagnostics.\n');
end
fprintf('Local pose is for flight control. Global pose is for map/mission use.\n');
fprintf('================================================\n\n');
end

% ========================================================================
function cfg = defaultConfig()
cfg.duration = 60;
cfg.imuRate = 200;
cfg.vioRate = 30;
cfg.lidarRate = 5.5;
cfg.rangeRate = 30;
cfg.baroRate = 25;
cfg.room = [6 6 2.5];
cfg.obstacles = [1.0 1.0 0.5 0.5; 4.0 3.5 0.5 0.5];
cfg.obstacleHeight = 1.60;
cfg.requirement_m = 0.10;
cfg.attRequirement_deg = 2.0;
cfg.gW = [0;0;-9.81];

% Hardware/animation geometry. The selected CAD assembly has a 450 mm
% diagonal motor-to-motor wheelbase and separate 10 x 4.5 inch props.
cfg.motorKV = 920;                    % rpm/V, motor constant
cfg.motorVoltageRange = [7 12];       % user-specified operating range
cfg.motorVoltageNominal = 11.1;       % 3S nominal; change for actual battery
cfg.motorLoadFactor = 0.85;           % visual estimate; replace with bench data
cfg.thrustToWeight = 2.40;
cfg.motorArmRadius = 0.225;           % centre to motor [m]
cfg.propDiameter = 0.254;             % 10 inch propeller diameter [m]
cfg.propRadius = cfg.propDiameter/2;
cfg.collisionRadius = cfg.motorArmRadius + cfg.propRadius;
cfg.motorRPMNoLoadRange = cfg.motorKV*cfg.motorVoltageRange;
cfg.motorMaxLoadedRPM = cfg.motorLoadFactor*cfg.motorKV*cfg.motorVoltageNominal;
cfg.motorHoverRPM = cfg.motorMaxLoadedRPM/sqrt(cfg.thrustToWeight);

% Camera visualization. FOV values supplied for the D435i.
cfg.cameraHFOV_deg = 87;
cfg.cameraVFOV_deg = 58;
cfg.cameraFrustumRange = 1.35;

% Geofence is the safe volume for the DRONE CENTRE, not merely the room.
% It includes the propeller envelope, the 10 cm navigation requirement and
% a 5 cm control/braking allowance. Obstacles use the same inflation.
cfg.localizationMargin = cfg.requirement_m;
cfg.controlMargin = 0.05;
cfg.geofenceMarginXY = cfg.collisionRadius + ...
    cfg.localizationMargin + cfg.controlMargin;
cfg.geofence = [cfg.geofenceMarginXY, cfg.room(1)-cfg.geofenceMarginXY, ...
                cfg.geofenceMarginXY, cfg.room(2)-cfg.geofenceMarginXY, ...
                0.35, cfg.room(3)-0.30]; % [xmin xmax ymin ymax zmin zmax]

% Sensor extrinsics. R_BS maps sensor axes to body axes; r_BS is sensor
% origin expressed in body coordinates. Replace with measured calibration.
cfg.r_BC = [0.08; 0.00; 0.02]; cfg.R_BC = eye(3); % camera
cfg.r_BL = [0.00; 0.00; 0.05]; cfg.R_BL = eye(3); % planar lidar
cfg.r_BR = [0.00; 0.00;-0.05]; cfg.d_BR = [0;0;-1]; % range beam

% Continuous-time IMU noise densities and bias random walks.
cfg.accelND = 0.003;
cfg.gyroND = deg2rad(0.025);
cfg.accelBiasRW = 2e-4;
cfg.gyroBiasRW = deg2rad(0.002);

% D435i host-VIO measurement and simulated drift.
cfg.vioPosSigma = 0.015;
cfg.vioVelSigma = 0.025;
cfg.vioAttSigma = deg2rad(0.45);
cfg.vioDriftPosRW = 8e-4;
cfg.vioDriftAttRW = deg2rad(0.012);
cfg.vioOutlierProb = 0.005;

% LiDAR and altimeters.
cfg.lidarSigmaXY = 0.025;
cfg.lidarSigmaYaw = deg2rad(0.7);
cfg.rangeSigma = 0.012;
cfg.baroSigma = 0.06;
cfg.baroBiasRW = 0.002;

% RPLidar simulation and ICP.
cfg.nBeams = 360;
cfg.rangeNoise = 0.012;
cfg.icpMaxCorr = 0.22;
cfg.icpTrim = 0.72;
cfg.icpIterations = 14;
cfg.icpStepXY = 0.08;
cfg.icpStepYaw = deg2rad(3);
cfg.localSubmapScans = 45;
cfg.mapVoxel = 0.045;
cfg.icpHealthRMSE = 0.080;
cfg.icpHealthOverlap = 80;
cfg.icpHealthCorrection = 0.12;
cfg.icpHealthYaw = deg2rad(5);

% ScanContext + keyframe graph. Candidate retrieval is never sufficient by
% itself; every closure also passes metric ICP and consistency gates.
cfg.enableScanContext = true;
cfg.scRings = 20;
cfg.scSectors = 60;
cfg.scExcludeRecent = 45;
cfg.scThreshold = 0.24;
cfg.scVerifyRMSE = 0.065;
cfg.keyframeStride = 5;
cfg.maxLoopClosures = 8;

% Chi-square 99.9% gates (hard-coded to avoid toolbox dependency).
cfg.gateVIO9 = 27.877;
cfg.gateLidar3 = 16.266;
cfg.gate1 = 10.828;
end

% ========================================================================
function out = runTrial(seed, stress, cfg, makePlots, makeAnimation)
rng(seed,'twister');
label = 'NOMINAL'; if stress, label = 'STRESS'; end
fprintf('--- %s TRIAL (seed %d) ---\n', label, seed);
results_dir = fullfile(cfg.resultsRoot, lower(label));
if ~exist(results_dir,'dir'), mkdir(results_dir); end

gt = simulateTruth(cfg);
pathSafety = validateReferencePath(gt,cfg);
fprintf('  Reference path length : %6.2f m\n',pathSafety.pathLength_m);
fprintf('  Minimum clearance     : %6.2f m (required %.2f m)\n', ...
    pathSafety.minStaticClearance_m,cfg.geofenceMarginXY);
if ~pathSafety.safe
    error(['Reference path violates the production geofence or an inflated ' ...
           'obstacle keep-out zone.']);
end
imu = simulateIMU(gt,cfg);
if stress, vioDrop = [25 32]; rangeDrop = [18 38];
else, vioDrop = []; rangeDrop = [20 35]; end
vio = simulateVIO(gt,cfg,vioDrop);
alt = simulateAltimeters(gt,cfg,rangeDrop);
lidar = simulateLidar(gt,cfg);

[est, localLidar] = runIntegratedESKF(gt,imu,vio,alt,lidar,cfg);
[globalLidar, loops, keyframes] = buildGlobalPoseGraph( ...
    lidar.scans, localLidar.pose, localLidar.valid, cfg);

% Truth at LiDAR sensor origin.
L = numel(lidar.t); lidarTruth = zeros(L,3);
for i=1:L
    k = lidar.idx(i); R = q2R(gt.q(k,:));
    pL = gt.p(k,:).' + R*cfg.r_BL; RWL = R*cfg.R_BL;
    lidarTruth(i,:) = [pL(1), pL(2), atan2(RWL(2,1),RWL(1,1))];
end
localErr = vecnorm(localLidar.pose(:,1:2)-lidarTruth(:,1:2),2,2);
globalErr = vecnorm(globalLidar(:,1:2)-lidarTruth(:,1:2),2,2);

ss = gt.t >= 5;
summary.seed = seed;
summary.stress = stress;
summary.fusedMaxAfter5_m = max(est.posErr(ss));
summary.fusedRMSEAfter5_m = sqrt(mean(est.posErr(ss).^2));
summary.fusedFinal_m = est.posErr(end);
summary.attMaxAfter5_deg = rad2deg(max(est.attErr(ss)));
summary.lidarLocalMax_m = max(localErr);
summary.lidarLocalRMSE_m = sqrt(mean(localErr.^2));
summary.lidarGlobalMax_m = max(globalErr);
summary.lidarHealthyFraction = mean(localLidar.valid);
summary.verifiedLoops = size(loops,1);
% MATLAB does not support indexing a direct function return in older releases.
dLocal = se2Between(localLidar.pose(1,:),localLidar.pose(end,:));
dGlobal = se2Between(globalLidar(1,:),globalLidar(end,:));
summary.localReturnError_m = norm(dLocal(1:2));
summary.globalReturnError_m = norm(dGlobal(1:2));
summary.counts = est.counts;
summary.unhealthySamples = est.unhealthySamples;
summary.pathLength_m = pathSafety.pathLength_m;
summary.minStaticClearance_m = pathSafety.minStaticClearance_m;
summary.pathSafe = pathSafety.safe;
summary.pass = summary.fusedMaxAfter5_m < cfg.requirement_m && ...
               summary.attMaxAfter5_deg < cfg.attRequirement_deg && ...
               summary.lidarHealthyFraction > 0.80 && summary.unhealthySamples == 0 && ...
               summary.pathSafe;

fprintf('  Fused max after 5 s : %6.2f cm\n',100*summary.fusedMaxAfter5_m);
fprintf('  Fused RMSE after 5 s: %6.2f cm\n',100*summary.fusedRMSEAfter5_m);
fprintf('  Attitude max        : %6.3f deg\n',summary.attMaxAfter5_deg);
fprintf('  Local LiDAR max     : %6.2f cm\n',100*summary.lidarLocalMax_m);
fprintf('  LiDAR healthy       : %6.2f %%\n',100*summary.lidarHealthyFraction);
fprintf('  Verified loops      : %d\n',summary.verifiedLoops);
fprintf('  Unhealthy samples   : %d\n',summary.unhealthySamples);
fprintf('  Path clearance      : %6.2f cm\n',100*summary.minStaticClearance_m);
fprintf('  VIO accepted/rej    : %d / %d\n',est.counts.vioAcc,est.counts.vioRej);
fprintf('  LiDAR accepted/rej  : %d / %d\n',est.counts.lidarAcc,est.counts.lidarRej);
fprintf('  Range accepted/rej  : %d / %d\n',est.counts.rangeAcc,est.counts.rangeRej);
fprintf('  Baro accepted/rej   : %d / %d\n',est.counts.baroAcc,est.counts.baroRej);
if summary.pass, fprintf('  RESULT: PASS\n\n'); else, fprintf('  RESULT: FAIL\n\n'); end

if makePlots
    plot_S2_dashboard(gt,imu,vio,alt,lidar,est,lidarTruth, ...
        localLidar,globalLidar,loops,keyframes,summary,cfg,label,results_dir);
end

animationFile='';
if makeAnimation
    animationFile=animate_S2_flight(gt,est,vio,alt,lidar,localLidar,globalLidar, ...
        cfg,label,results_dir);
end

% Save the numerical trial output alongside the figures for later replay.
save(fullfile(results_dir,'S2_visual_slam_trial_data.mat'), ...
    'summary','gt','imu','vio','alt','lidar','est','lidarTruth', ...
    'localLidar','globalLidar','loops','keyframes','pathSafety','cfg','-v7.3');
writeTrialSummary(summary,results_dir,label);

out.summary = summary; out.est = est; out.localLidar = localLidar;
out.globalLidar = globalLidar; out.loops = loops; out.keyframes = keyframes;
out.pathSafety = pathSafety; out.animationFile = animationFile;
end

% ========================================================================
function gt = simulateTruth(cfg)
dt=1/cfg.imuRate; t=(0:dt:cfg.duration).';
% Collision-checked figure-eight. The earlier path intersected the second
% obstacle. This path remains close to 20 m while retaining clearance from
% the inflated 10-inch-propeller envelope and room boundaries.
cx=3.2; cy=2.0; rx=1.5; ry=0.9; phi=deg2rad(250);
w1=2*pi/30; w2=4*pi/30;
p=[cx+rx*sin(w1*t), cy+ry*sin(w2*t+phi), ...
   1.15+0.16*sin(2*pi*t/18)];
v=[rx*w1*cos(w1*t), ry*w2*cos(w2*t+phi), ...
   0.16*(2*pi/18)*cos(2*pi*t/18)];
a=[-rx*w1^2*sin(w1*t), -ry*w2^2*sin(w2*t+phi), ...
   -0.16*(2*pi/18)^2*sin(2*pi*t/18)];
yaw=unwrap(atan2(v(:,2),v(:,1)));
roll=deg2rad(5)*sin(2*pi*t/8); pitch=deg2rad(4)*sin(2*pi*t/10+0.4);
N=numel(t); q=zeros(N,4);
for k=1:N, q(k,:)=rpy2q(roll(k),pitch(k),yaw(k)); end
omega=zeros(N,3);
for k=1:N-1
    dq=qmul(qconj(q(k,:)),q(k+1,:)); omega(k,:)=qlog(dq)/dt;
end
omega(end,:)=omega(end-1,:);
gt=struct('t',t,'p',p,'pRef',p,'v',v,'a',a,'q',q,'omega',omega, ...
          'rpy',[roll pitch yaw]);
end

% ========================================================================
function imu = simulateIMU(gt,cfg)
dt=1/cfg.imuRate; N=numel(gt.t);
ba=zeros(N,3); bg=zeros(N,3);
ba(1,:)=[0.018 -0.014 0.012]; bg(1,:)=deg2rad([0.06 -0.04 0.05]);
for k=2:N
    ba(k,:)=ba(k-1,:)+cfg.accelBiasRW*sqrt(dt)*randn(1,3);
    bg(k,:)=bg(k-1,:)+cfg.gyroBiasRW*sqrt(dt)*randn(1,3);
end
sa=cfg.accelND/sqrt(dt); sg=cfg.gyroND/sqrt(dt);
acc=zeros(N,3);
for k=1:N
    R=q2R(gt.q(k,:)); acc(k,:)=(R.'*(gt.a(k,:).'-cfg.gW)).';
end
acc=acc+ba+sa*randn(N,3); gyro=gt.omega+bg+sg*randn(N,3);
imu=struct('acc',acc,'gyro',gyro,'baTrue',ba,'bgTrue',bg);
end

% ========================================================================
function vio = simulateVIO(gt,cfg,dropWindow)
t=(0:1/cfg.vioRate:cfg.duration).'; idx=timeToIndex(t,cfg.imuRate,numel(gt.t));
Ngt=numel(gt.t); dt=1/cfg.imuRate;
dp=zeros(Ngt,3); dth=zeros(Ngt,3);
for k=2:Ngt
    dp(k,:)=dp(k-1,:)+cfg.vioDriftPosRW*sqrt(dt)*randn(1,3);
    dth(k,:)=dth(k-1,:)+cfg.vioDriftAttRW*sqrt(dt)*randn(1,3);
end
M=numel(t); p=zeros(M,3); v=zeros(M,3); q=zeros(M,4);
for i=1:M
    k=idx(i); RWB=q2R(gt.q(k,:)); RWC=RWB*cfg.R_BC;
    pWC=gt.p(k,:).'+RWB*cfg.r_BC;
    RWCm=RWC*q2R(qexp(dth(k,:)+cfg.vioAttSigma*randn(1,3)));
    pWCm=pWC+dp(k,:).'+cfg.vioPosSigma*randn(3,1);
    RWBm=RWCm*cfg.R_BC.'; p(i,:)=(pWCm-RWBm*cfg.r_BC).';
    q(i,:)=R2q(RWBm); v(i,:)=gt.v(k,:)+cfg.vioVelSigma*randn(1,3);
end
valid=true(M,1);
if ~isempty(dropWindow), valid=valid & ~(t>=dropWindow(1)&t<=dropWindow(2)); end
outlier=rand(M,1)<cfg.vioOutlierProb; outlier(1)=false;
p(outlier,:)=p(outlier,:)+0.35*randn(nnz(outlier),3);
ids=find(outlier); for n=1:numel(ids), q(ids(n),:)=qmul(q(ids(n),:),qexp(deg2rad(12)*randn(1,3))); end
vio=struct('t',t,'idx',idx,'p',p,'v',v,'q',q,'valid',valid,'outlier',outlier);
end

% ========================================================================
function alt = simulateAltimeters(gt,cfg,rangeDrop)
tr=(0:1/cfg.rangeRate:cfg.duration).'; ir=timeToIndex(tr,cfg.imuRate,numel(gt.t));
zr=zeros(numel(tr),1);
for i=1:numel(tr)
    k=ir(i); R=q2R(gt.q(k,:)); pS=gt.p(k,:).'+R*cfg.r_BR; dW=R*cfg.d_BR;
    zr(i)=-pS(3)/dW(3)+cfg.rangeSigma*randn;
end
validR=rand(numel(tr),1)>0.05;
if ~isempty(rangeDrop), validR=validR & ~(tr>=rangeDrop(1)&tr<=rangeDrop(2)); end

tb=(0:1/cfg.baroRate:cfg.duration).'; ib=timeToIndex(tb,cfg.imuRate,numel(gt.t));
b=zeros(numel(tb),1); b(1)=0.12; dt=1/cfg.baroRate;
for i=2:numel(tb), b(i)=b(i-1)+cfg.baroBiasRW*sqrt(dt)*randn; end
zb=gt.p(ib,3)+b+cfg.baroSigma*randn(numel(tb),1);
alt=struct('tr',tr,'ir',ir,'zr',zr,'validR',validR, ...
           'tb',tb,'ib',ib,'zb',zb,'baroBiasTrue',b);
end

% ========================================================================
function lidar = simulateLidar(gt,cfg)
t=(0:1/cfg.lidarRate:cfg.duration).'; idx=timeToIndex(t,cfg.imuRate,numel(gt.t));
angles=linspace(0,2*pi,cfg.nBeams+1).'; angles(end)=[];
obstacles=cfg.obstacles; scans=cell(numel(t),1);
for i=1:numel(t)
    k=idx(i); RWB=q2R(gt.q(k,:)); pL=gt.p(k,:).'+RWB*cfg.r_BL;
    RWL=RWB*cfg.R_BL; yaw=atan2(RWL(2,1),RWL(1,1));
    ranges=zeros(cfg.nBeams,1);
    for b=1:cfg.nBeams
        wa=angles(b)+yaw; dx=cos(wa); dy=sin(wa); cand=[];
        if abs(dx)>1e-12, cand=[cand,(0-pL(1))/dx,(cfg.room(1)-pL(1))/dx]; end %#ok<AGROW>
        if abs(dy)>1e-12, cand=[cand,(0-pL(2))/dy,(cfg.room(2)-pL(2))/dy]; end %#ok<AGROW>
        hit=min(cand(cand>1e-6));
        for o=1:size(obstacles,1)
            ox=obstacles(o,1);oy=obstacles(o,2);ow=obstacles(o,3);od=obstacles(o,4);
            if abs(dx)>1e-12, tx1=(ox-pL(1))/dx;tx2=(ox+ow-pL(1))/dx; else, tx1=-inf;tx2=inf; end
            if abs(dy)>1e-12, ty1=(oy-pL(2))/dy;ty2=(oy+od-pL(2))/dy; else, ty1=-inf;ty2=inf; end
            te=max(min(tx1,tx2),min(ty1,ty2)); tx=min(max(tx1,tx2),max(ty1,ty2));
            if te>0 && te<tx && te<hit, hit=te; end
        end
        ranges(b)=min(12,max(0.15,hit+cfg.rangeNoise*randn));
    end
    scans{i}=[ranges.*cos(angles),ranges.*sin(angles)];
end
lidar=struct('t',t,'idx',idx,'angles',angles,'scans',{scans});
end

% ========================================================================
function [est,lout] = runIntegratedESKF(gt,imu,vio,alt,lidar,cfg)
N=numel(gt.t); dt=1/cfg.imuRate;
f=initFilter(vio.p(1,:).',vio.v(1,:).',vio.q(1,:));
pHist=zeros(N,3);qHist=zeros(N,4);baHist=zeros(N,3);bgHist=zeros(N,3);bbHist=zeros(N,1);
pHist(1,:)=f.p.';qHist(1,:)=f.q;
kv=2;kl=1;kr=2;kb=2; L=numel(lidar.t);
pose=zeros(L,3); valid=false(L,1); ldiag=zeros(L,7); mapBlocks={};
counts=struct('vioAcc',1,'vioRej',0,'lidarAcc',0,'lidarRej',0, ...
              'rangeAcc',1,'rangeRej',0,'baroAcc',1,'baroRej',0);
nis=struct('vio',[],'vio_t',[],'lidar',[],'lidar_t',[], ...
           'range',[],'range_t',[],'baro',[],'baro_t',[]);
health=true(N,1);lastVio=0;lastLidar=0;lastRange=0;lastBaro=0;
Rvio=diag([repmat(cfg.vioPosSigma^2,1,3),repmat(cfg.vioVelSigma^2,1,3),repmat(cfg.vioAttSigma^2,1,3)]);
Rlid=diag([cfg.lidarSigmaXY^2,cfg.lidarSigmaXY^2,cfg.lidarSigmaYaw^2]);

% Initial LiDAR scan.
while kl<=L && lidar.t(kl)<=gt.t(1)+eps
    [f,pose,valid,ldiag,mapBlocks,counts,nis]=processLidar( ...
        f,kl,lidar.t(kl),lidar.scans,pose,valid,ldiag,mapBlocks,counts,nis,Rlid,cfg);
    if valid(kl),lastLidar=lidar.t(kl);end
    kl=kl+1;
end

for k=2:N
    f=propagateFilter(f,imu.acc(k,:).',imu.gyro(k,:).',dt,cfg);
    now=gt.t(k)+1e-12;
    while kv<=numel(vio.t) && vio.t(kv)<=now
        if vio.valid(kv)
            r=[vio.p(kv,:).'-f.p;vio.v(kv,:).'-f.v;orientationResidual(f.q,vio.q(kv,:))];
            H=zeros(9,16);H(1:3,1:3)=eye(3);H(4:6,4:6)=eye(3);H(7:9,7:9)=eye(3);
            [f,ok,n]=filterUpdate(f,r,H,Rvio,cfg.gateVIO9);
            if ok,counts.vioAcc=counts.vioAcc+1;lastVio=vio.t(kv);else,counts.vioRej=counts.vioRej+1;end
            nis.vio(end+1,1)=n;nis.vio_t(end+1,1)=vio.t(kv); %#ok<AGROW>
        end
        kv=kv+1;
    end
    while kl<=L && lidar.t(kl)<=now
        [f,pose,valid,ldiag,mapBlocks,counts,nis]=processLidar( ...
            f,kl,lidar.t(kl),lidar.scans,pose,valid,ldiag,mapBlocks,counts,nis,Rlid,cfg);
        if valid(kl),lastLidar=lidar.t(kl);end
        kl=kl+1;
    end
    while kr<=numel(alt.tr) && alt.tr(kr)<=now
        if alt.validR(kr)
            [h,H]=rangeMeasurement(f,cfg);
            if isfinite(h)
                [f,ok,n]=filterUpdate(f,alt.zr(kr)-h,H,cfg.rangeSigma^2,cfg.gate1);
                if ok,counts.rangeAcc=counts.rangeAcc+1;lastRange=alt.tr(kr);else,counts.rangeRej=counts.rangeRej+1;end
                nis.range(end+1,1)=n;nis.range_t(end+1,1)=alt.tr(kr); %#ok<AGROW>
            else,counts.rangeRej=counts.rangeRej+1; end
        end
        kr=kr+1;
    end
    while kb<=numel(alt.tb) && alt.tb(kb)<=now
        r=alt.zb(kb)-(f.p(3)+f.bbaro);H=zeros(1,16);H(3)=1;H(16)=1;
        [f,ok,n]=filterUpdate(f,r,H,cfg.baroSigma^2,cfg.gate1);
        if ok,counts.baroAcc=counts.baroAcc+1;lastBaro=alt.tb(kb);else,counts.baroRej=counts.baroRej+1;end
        nis.baro(end+1,1)=n;nis.baro_t(end+1,1)=alt.tb(kb);kb=kb+1; %#ok<AGROW>
    end
    xyHealthy=(now-lastVio<1.0)||(now-lastLidar<1.0);
    zHealthy=(now-lastVio<1.0)||(now-lastRange<0.75)||(now-lastBaro<1.0);
    covHealthy=max(diag(f.P(1:3,1:3)))<0.25^2;
    health(k)=xyHealthy&&zHealthy&&covHealthy;
    pHist(k,:)=f.p.';qHist(k,:)=f.q;baHist(k,:)=f.ba.';bgHist(k,:)=f.bg.';bbHist(k)=f.bbaro;
end
posErr=vecnorm(pHist-gt.p,2,2);attErr=zeros(N,1);
for k=1:N,attErr(k)=norm(orientationResidual(qHist(k,:),gt.q(k,:)));end
est=struct('p',pHist,'q',qHist,'ba',baHist,'bg',bgHist,'bbaro',bbHist, ...
           'posErr',posErr,'attErr',attErr,'counts',counts,'nis',nis,'health',health, ...
           'unhealthySamples',nnz(~health));
lout=struct('pose',pose,'valid',valid,'diag',ldiag);
end

% ========================================================================
function [f,pose,valid,ldiag,mapBlocks,counts,nis] = processLidar( ...
    f,i,tMeas,scans,pose,valid,ldiag,mapBlocks,counts,nis,Rlid,cfg)
[pred,H]=lidarMeasurement(f,cfg);
if isempty(mapBlocks)
    z=pred;rmse=0;overlap=size(scans{i},1);healthy=true;c=[0 0 0];
else
    first=max(1,numel(mapBlocks)-cfg.localSubmapScans+1);
    mappts=voxelDown(vertcat(mapBlocks{first:end}),cfg.mapVoxel);
    [z,rmse,overlap]=icpToMap(scans{i},mappts,pred,cfg);
    c=se2Between(pred,z);
    healthy=isfinite(rmse)&&rmse<cfg.icpHealthRMSE&&overlap>=cfg.icpHealthOverlap&& ...
        norm(c(1:2))<cfg.icpHealthCorrection&&abs(c(3))<cfg.icpHealthYaw;
end
ok=false;n=inf;
if healthy
    r=z-pred;r(3)=wrapPi(r(3));[f,ok,n]=filterUpdate(f,r.',H,Rlid,cfg.gateLidar3);
end
if ok
    pose(i,:)=z;valid(i)=true;mapBlocks{end+1}=transformPoints(scans{i}(1:2:end,:),z); %#ok<AGROW>
    counts.lidarAcc=counts.lidarAcc+1;nis.lidar(end+1,1)=n;nis.lidar_t(end+1,1)=tMeas; %#ok<AGROW>
else
    pose(i,:)=pred;counts.lidarRej=counts.lidarRej+1;
    if isfinite(n),nis.lidar(end+1,1)=n;nis.lidar_t(end+1,1)=tMeas;end %#ok<AGROW>
end
ldiag(i,:)=[rmse,overlap,healthy,ok,n,norm(c(1:2)),abs(c(3))];
end

% ========================================================================
function f=initFilter(p,v,q)
f.p=p;f.v=v;f.q=qnormalize(q);f.ba=zeros(3,1);f.bg=zeros(3,1);f.bbaro=0;
s=[repmat(0.05,1,3),repmat(0.12,1,3),deg2rad([3 3 4]), ...
   repmat(0.06,1,3),deg2rad([0.4 0.4 0.4]),0.20];
f.P=diag(s.^2);
end

function f=propagateFilter(f,acc,gyro,dt,cfg)
fb=acc-f.ba;w=gyro-f.bg;R=q2R(f.q);aw=R*fb+cfg.gW;
f.p=f.p+f.v*dt+0.5*aw*dt^2;f.v=f.v+aw*dt;f.q=qnormalize(qmul(f.q,qexp((w*dt).')));
F=zeros(16);F(1:3,4:6)=eye(3);F(4:6,7:9)=-R*skew3(fb);F(4:6,10:12)=-R;
F(7:9,7:9)=-skew3(w);F(7:9,13:15)=-eye(3);Phi=eye(16)+F*dt;
G=zeros(16,13);G(4:6,1:3)=-R;G(7:9,4:6)=-eye(3);G(10:12,7:9)=eye(3);G(13:15,10:12)=eye(3);G(16,13)=1;
qc=[repmat(cfg.accelND^2,1,3),repmat(cfg.gyroND^2,1,3), ...
    repmat(cfg.accelBiasRW^2,1,3),repmat(cfg.gyroBiasRW^2,1,3),cfg.baroBiasRW^2];
f.P=Phi*f.P*Phi.'+G*diag(qc)*G.'*dt;f.P=(f.P+f.P.')/2;
end

function [f,ok,nis]=filterUpdate(f,r,H,Rm,gate)
r=r(:);S=H*f.P*H.'+Rm;nis=r.'*(S\r);
if ~isfinite(nis)||nis>gate,ok=false;return;end
K=(f.P*H.')/S;dx=K*r;I=eye(16);J=I-K*H;
f.P=J*f.P*J.'+K*Rm*K.';f=injectError(f,dx);
G=eye(16);G(7:9,7:9)=eye(3)-0.5*skew3(dx(7:9));f.P=G*f.P*G.';f.P=(f.P+f.P.')/2;ok=true;
end

function f=injectError(f,dx)
f.p=f.p+dx(1:3);f.v=f.v+dx(4:6);f.q=qnormalize(qmul(f.q,qexp(dx(7:9).')));
f.ba=f.ba+dx(10:12);f.bg=f.bg+dx(13:15);f.bbaro=f.bbaro+dx(16);
end

% ========================================================================
function [h,H]=lidarMeasurement(f,cfg)
R=q2R(f.q);pL=f.p+R*cfg.r_BL;RWL=R*cfg.R_BL;
h=[pL(1),pL(2),atan2(RWL(2,1),RWL(1,1))];H=zeros(3,16);
H(1,1)=1;H(2,2)=1;A=-R*skew3(cfg.r_BL);H(1:2,7:9)=A(1:2,:);
base=h(3);epsa=1e-7;
for j=1:3
    d=zeros(1,3);d(j)=epsa;Rp=R*q2R(qexp(d));RWLp=Rp*cfg.R_BL;
    H(3,6+j)=wrapPi(atan2(RWLp(2,1),RWLp(1,1))-base)/epsa;
end
end

function [h,H]=rangeMeasurement(f,cfg)
R=q2R(f.q);pS=f.p+R*cfg.r_BR;dW=R*cfg.d_BR;den=dW(3);H=zeros(1,16);
if den>=-0.2,h=nan;return;end
num=-pS(3);h=num/den;H(3)=-1/den;
Ar=(-R*skew3(cfg.r_BR));Ad=(-R*skew3(cfg.d_BR));
H(7:9)=(-Ar(3,:)*den-num*Ad(3,:))/den^2;
end

% ========================================================================
function [pose,rmse,overlap]=icpToMap(scan,mappts,pose,cfg)
rmse=inf;overlap=0;
for it=1:cfg.icpIterations
    Q=transformPoints(scan,pose);[idx,d]=knnsearch(mappts,Q);
    ids=find(d<cfg.icpMaxCorr);overlap=numel(ids);if overlap<40,break;end
    [~,ord]=sort(d(ids));keep=max(40,floor(cfg.icpTrim*numel(ids)));ids=ids(ord(1:keep));
    delta=rigidFit(Q(ids,:),mappts(idx(ids),:));
    delta(1:2)=max(-cfg.icpStepXY,min(cfg.icpStepXY,delta(1:2)));
    delta(3)=max(-cfg.icpStepYaw,min(cfg.icpStepYaw,delta(3)));
    pose=se2Compose(delta,pose);rmse=sqrt(mean(d(ids).^2));
    if norm(delta(1:2))<1e-4&&abs(delta(3))<1e-4,break;end
end
end

function delta=rigidFit(src,dst)
cs=mean(src,1);cd=mean(dst,1);X=src-cs;Y=dst-cd;[U,~,V]=svd(X.'*Y);R=V*U.';
if det(R)<0,V(:,end)=-V(:,end);R=V*U.';end
t=cd.'-R*cs.';delta=[t(1),t(2),atan2(R(2,1),R(1,1))];
end

function P=voxelDown(P,v)
if isempty(P),return;end;K=floor(P/v);[~,ia]=unique(K,'rows','stable');P=P(ia,:);
end

% ========================================================================
function [globalPose,loops,kfIdx]=buildGlobalPoseGraph(scans,localPose,valid,cfg)
accepted=find(valid);globalPose=localPose;loops=zeros(0,4);
if ~cfg.enableScanContext||numel(accepted)<3,kfIdx=accepted;return;end
kfIdx=accepted(1);
for i=2:numel(accepted),if accepted(i)-kfIdx(end)>=cfg.keyframeStride,kfIdx(end+1,1)=accepted(i);end,end %#ok<AGROW>
if kfIdx(end)~=accepted(end),kfIdx(end+1,1)=accepted(end);end
K=numel(kfIdx);desc=cell(K,1);keys=zeros(K,cfg.scRings);
for a=1:K,[desc{a},keys(a,:)]=scanContextDescriptor2D(scans{kfIdx(a)},cfg);end
edges=struct('i',{},'j',{},'z',{},'sig',{});
for a=2:K,edges(end+1)=makeEdge(a-1,a,se2Between(localPose(kfIdx(a-1),:),localPose(kfIdx(a),:)),[0.055 0.055 deg2rad(1.5)]);end %#ok<AGROW>
exclude=max(6,ceil(cfg.scExcludeRecent/cfg.keyframeStride));last=-1e9;
for a=exclude+1:K
    if size(loops,1)>=cfg.maxLoopClosures||a-last<6,continue;end
    old=1:a-exclude;[~,ord]=sort(vecnorm(keys(old,:)-keys(a,:),2,2));short=old(ord(1:min(6,numel(ord))));cand=[];
    for b=short
        [d,shift]=scanContextDistance(desc{a},desc{b});cand=[cand;d,b,shift]; %#ok<AGROW>
    end
    cand=sortrows(cand,1);
    for c=1:min(4,size(cand,1))
        d=cand(c,1);b=cand(c,2);if d>cfg.scThreshold,break;end
        if norm(localPose(kfIdx(a),1:2)-localPose(kfIdx(b),1:2))>1.25,continue;end
        init=se2Between(localPose(kfIdx(b),:),localPose(kfIdx(a),:));
        [z,rmse,overlap]=icpPair(scans{kfIdx(a)},scans{kfIdx(b)},init,cfg);
        consistency=se2Between(init,z);
        if rmse<cfg.scVerifyRMSE&&overlap>110&&norm(consistency(1:2))<0.08&&abs(consistency(3))<deg2rad(3)
            edges(end+1)=makeEdge(b,a,z,[0.035 0.035 deg2rad(1)]); %#ok<AGROW>
            loops(end+1,:)=[kfIdx(a),kfIdx(b),d,rmse];last=a;break; %#ok<AGROW>
        end
    end
end
kfLocal=localPose(kfIdx,:);if isempty(loops),kfGlobal=kfLocal;else,kfGlobal=optimizeGraph(kfLocal,edges);end
globalPose=interpolateGlobal(localPose,kfIdx,kfGlobal);
end

function e=makeEdge(i,j,z,sig),e=struct('i',i,'j',j,'z',z,'sig',sig);end

function [desc,key]=scanContextDescriptor2D(scan,cfg)
r=vecnorm(scan,2,2);th=mod(atan2(scan(:,2),scan(:,1)),2*pi);desc=zeros(cfg.scRings,cfg.scSectors);
ri=min(cfg.scRings,max(1,floor(r/8*cfg.scRings)+1));si=min(cfg.scSectors,max(1,floor(th/(2*pi)*cfg.scSectors)+1));
for n=1:numel(r),desc(ri(n),si(n))=max(desc(ri(n),si(n)),1-r(n)/8);end
key=mean(desc,2).';
end

function [best,bshift]=scanContextDistance(a,b)
best=1;bshift=0;
for s=0:size(a,2)-1
    bs=circshift(b,[0 s]);valid=vecnorm(a,2,1)>1e-8&vecnorm(bs,2,1)>1e-8;if ~any(valid),continue;end
    dots=sum(a(:,valid).*bs(:,valid),1);den=vecnorm(a(:,valid),2,1).*vecnorm(bs(:,valid),2,1);
    d=1-mean(dots./max(den,1e-12));if d<best,best=d;bshift=s;end
end
end

function [pose,rmse,overlap]=icpPair(cur,ref,pose,cfg)
rmse=inf;overlap=0;
for it=1:20
    Q=transformPoints(cur,pose);[idx,d]=knnsearch(ref,Q);ids=find(d<0.25);overlap=numel(ids);if overlap<50,break;end
    [~,ord]=sort(d(ids));ids=ids(ord(1:max(50,floor(0.75*numel(ids)))));delta=rigidFit(Q(ids,:),ref(idx(ids),:));
    delta(1:2)=max(-0.10,min(0.10,delta(1:2)));delta(3)=max(-deg2rad(5),min(deg2rad(5),delta(3)));
    pose=se2Compose(delta,pose);rmse=sqrt(mean(d(ids).^2));if norm(delta(1:2))<1e-4&&abs(delta(3))<1e-4,break;end
end
end

function P=optimizeGraph(P0,edges)
K=size(P0,1);x0=reshape(P0(2:end,:).',[],1);anchor=P0(1,:);
fun=@(x)graphResidual(x,K,edges,anchor);opts=optimoptions('lsqnonlin','Display','off','MaxIterations',25,'MaxFunctionEvaluations',3000,'FunctionTolerance',1e-7,'StepTolerance',1e-7);
try,x=lsqnonlin(fun,x0,[],[],opts);P=P0;P(2:end,:)=reshape(x,3,[]).';P(:,3)=wrapPi(P(:,3));catch ME
    warning('Pose graph optimization skipped: %s',ME.message);P=P0;
end
end

function r=graphResidual(x,K,edges,anchor)
P=zeros(K,3);P(1,:)=anchor;P(2:end,:)=reshape(x,3,[]).';r=zeros(3*numel(edges),1);
for n=1:numel(edges)
    pred=se2Between(P(edges(n).i,:),P(edges(n).j,:));e=se2Between(edges(n).z,pred);e(3)=wrapPi(e(3));q=e./edges(n).sig;
    % Huberized normalized residual protects against a surviving false edge.
    a=abs(q);big=a>1.5;q(big)=sign(q(big)).*sqrt(2*1.5*a(big)-1.5^2);
    r(3*n-2:3*n)=q.';
end
end

function out=interpolateGlobal(local,kf,kg)
C=zeros(numel(kf),3);for a=1:numel(kf),C(a,:)=se2Compose(kg(a,:),se2Inverse(local(kf(a),:)));end
out=local;
for a=1:numel(kf)-1
    i0=kf(a);i1=kf(a+1);
    for i=i0:i1
        u=(i-i0)/max(1,i1-i0);ci=(1-u)*C(a,1:2)+u*C(a+1,1:2);cy=wrapPi(C(a,3)+u*wrapPi(C(a+1,3)-C(a,3)));
        out(i,:)=se2Compose([ci cy],local(i,:));
    end
end
end

% ========================================================================
function safety = validateReferencePath(gt,cfg)
% Validate the prescribed centre-of-mass path against the room geofence and
% obstacle keep-out regions inflated by the full propeller envelope.
p=gt.p;
wallClear=min([p(:,1),cfg.room(1)-p(:,1),p(:,2),cfg.room(2)-p(:,2)],[],2);
obsClear=inf(size(p,1),1);
for i=1:size(cfg.obstacles,1)
    o=cfg.obstacles(i,:);
    dx=max([o(1)-p(:,1),zeros(size(p,1),1),p(:,1)-(o(1)+o(3))],[],2);
    dy=max([o(2)-p(:,2),zeros(size(p,1),1),p(:,2)-(o(2)+o(4))],[],2);
    horizontal=hypot(dx,dy);
    active=p(:,3) <= cfg.obstacleHeight + cfg.controlMargin;
    horizontal(~active)=inf;
    obsClear=min(obsClear,horizontal);
end
verticalSafe=p(:,3)>=cfg.geofence(5) & p(:,3)<=cfg.geofence(6);
minimum=min([wallClear;obsClear]);
safety=struct;
safety.pathLength_m=sum(vecnorm(diff(p),2,2));
safety.minWallClearance_m=min(wallClear);
safety.minObstacleClearance_m=min(obsClear);
safety.minStaticClearance_m=minimum;
safety.requiredClearance_m=cfg.geofenceMarginXY;
safety.verticalSafe=all(verticalSafe);
safety.safe=minimum>=cfg.geofenceMarginXY && safety.verticalSafe;
end

% ========================================================================
function writeTrialSummary(summary,results_dir,label)
fid=fopen(fullfile(results_dir,'S2_visual_slam_summary.txt'),'w');
if fid<0, warning('Could not create summary text file in %s',results_dir); return; end
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'Stage S2 visual SLAM / 6-DOF production estimator - %s\n',label);
fprintf(fid,'Seed: %d\n',summary.seed);
fprintf(fid,'Stress trial: %d\n',summary.stress);
fprintf(fid,'Fused max after 5 s [m]: %.8f\n',summary.fusedMaxAfter5_m);
fprintf(fid,'Fused RMSE after 5 s [m]: %.8f\n',summary.fusedRMSEAfter5_m);
fprintf(fid,'Fused final error [m]: %.8f\n',summary.fusedFinal_m);
fprintf(fid,'Attitude max after 5 s [deg]: %.8f\n',summary.attMaxAfter5_deg);
fprintf(fid,'Local LiDAR max [m]: %.8f\n',summary.lidarLocalMax_m);
fprintf(fid,'Local LiDAR RMSE [m]: %.8f\n',summary.lidarLocalRMSE_m);
fprintf(fid,'Global LiDAR max [m]: %.8f\n',summary.lidarGlobalMax_m);
fprintf(fid,'LiDAR healthy fraction: %.8f\n',summary.lidarHealthyFraction);
fprintf(fid,'Verified loop closures: %d\n',summary.verifiedLoops);
fprintf(fid,'Local return error [m]: %.8f\n',summary.localReturnError_m);
fprintf(fid,'Global return error [m]: %.8f\n',summary.globalReturnError_m);
fprintf(fid,'Unhealthy samples: %d\n',summary.unhealthySamples);
fprintf(fid,'Reference path length [m]: %.8f\n',summary.pathLength_m);
fprintf(fid,'Minimum static clearance [m]: %.8f\n',summary.minStaticClearance_m);
fprintf(fid,'Reference path safe: %d\n',summary.pathSafe);
fprintf(fid,'VIO accepted/rejected: %d/%d\n',summary.counts.vioAcc,summary.counts.vioRej);
fprintf(fid,'LiDAR accepted/rejected: %d/%d\n',summary.counts.lidarAcc,summary.counts.lidarRej);
fprintf(fid,'Range accepted/rejected: %d/%d\n',summary.counts.rangeAcc,summary.counts.rangeRej);
fprintf(fid,'Barometer accepted/rejected: %d/%d\n',summary.counts.baroAcc,summary.counts.baroRej);
fprintf(fid,'PASS: %d\n',summary.pass);
end

% ========================================================================
function idx=timeToIndex(t,rate,N),idx=min(N,max(1,round(t*rate)+1));end
function P=transformPoints(P,p),c=cos(p(3));s=sin(p(3));R=[c -s;s c];P=P*R.'+p(1:2);end
function c=se2Compose(a,b),ca=cos(a(3));sa=sin(a(3));c=[a(1)+ca*b(1)-sa*b(2),a(2)+sa*b(1)+ca*b(2),wrapPi(a(3)+b(3))];end
function a=se2Inverse(p),c=cos(p(3));s=sin(p(3));a=[-c*p(1)-s*p(2),s*p(1)-c*p(2),wrapPi(-p(3))];end
function d=se2Between(a,b),d=se2Compose(se2Inverse(a),b);end
function a=wrapPi(a),a=mod(a+pi,2*pi)-pi;end
function S=skew3(v),v=v(:);S=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];end

function q=qnormalize(q),q=q(:).';q=q/norm(q);if q(1)<0,q=-q;end,end
function q=qconj(q),q=[q(1),-q(2:4)];end
function q=qmul(a,b),a=a(:).';b=b(:).';q=[a(1)*b(1)-dot(a(2:4),b(2:4)),a(1)*b(2:4)+b(1)*a(2:4)+cross(a(2:4),b(2:4))];q=qnormalize(q);end
function q=qexp(v),v=v(:).';a=norm(v);if a<1e-10,q=qnormalize([1,0.5*v]);else,q=[cos(a/2),sin(a/2)*v/a];end,end
function v=qlog(q),q=qnormalize(q);nv=norm(q(2:4));if nv<1e-10,v=2*q(2:4);else,a=2*atan2(nv,q(1));if a>pi,a=a-2*pi;end;v=a*q(2:4)/nv;end,end
function R=q2R(q),q=qnormalize(q);w=q(1);x=q(2);y=q(3);z=q(4);R=[1-2*(y^2+z^2),2*(x*y-z*w),2*(x*z+y*w);2*(x*y+z*w),1-2*(x^2+z^2),2*(y*z-x*w);2*(x*z-y*w),2*(y*z+x*w),1-2*(x^2+y^2)];end
function q=rpy2q(r,p,y),cr=cos(r/2);sr=sin(r/2);cp=cos(p/2);sp=sin(p/2);cy=cos(y/2);sy=sin(y/2);q=qnormalize([cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy]);end
function q=R2q(R)
tr=trace(R);
if tr>0
    S=sqrt(tr+1)*2;
    q=[0.25*S,(R(3,2)-R(2,3))/S,(R(1,3)-R(3,1))/S,(R(2,1)-R(1,2))/S];
else
    [~,i]=max(diag(R));
    if i==1
        S=sqrt(1+R(1,1)-R(2,2)-R(3,3))*2;
        q=[(R(3,2)-R(2,3))/S,0.25*S,(R(1,2)+R(2,1))/S,(R(1,3)+R(3,1))/S];
    elseif i==2
        S=sqrt(1+R(2,2)-R(1,1)-R(3,3))*2;
        q=[(R(1,3)-R(3,1))/S,(R(1,2)+R(2,1))/S,0.25*S,(R(2,3)+R(3,2))/S];
    else
        S=sqrt(1+R(3,3)-R(1,1)-R(2,2))*2;
        q=[(R(2,1)-R(1,2))/S,(R(1,3)+R(3,1))/S,(R(2,3)+R(3,2))/S,0.25*S];
    end
end
q=qnormalize(q);
end
function rpy=q2rpy(q),R=q2R(q);p=asin(max(-1,min(1,-R(3,1))));r=atan2(R(3,2),R(3,3));y=atan2(R(2,1),R(1,1));rpy=[r p y];end
function r=orientationResidual(qpred,qmeas),r=qlog(qmul(qconj(qpred),qmeas)).';end

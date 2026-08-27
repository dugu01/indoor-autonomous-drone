%% =========================================================================
% run_S2_unified_6dof.m
% Stage S2: Unified Multi-Lane ESKF + Global Pose Graph + RTL
%
% Full integration of:
%   - S2 Production: sensor extrinsics, ESKF, barometer, VIO, range,
%                     ScanContext loop closure, ICP health metrics
%   - S3 Multi-Lane: 4-lane failover, NIS health switching, RTL logic,
%                    gravity roll/pitch, literature citations
%
% Literature:
%   [1] Qin et al. 2018 (VINS-Mono): visual-inertial gravity alignment
%   [2] Xu & Zhang 2021 (FAST-LIO): LiDAR-inertial tightly-coupled EKF
%   [3] Falchi et al. 2024 (PX4 EKF2): multi-instance NIS health failover
%   [4] Rutkowski 2024 (ArduPilot EKF3): lane affinity switching
%   [5] Forster et al. 2017: IMU preintegration on manifold, ESKF
%
% State (16-error-state ESKF):
%   Nominal: [p(3), v(3), q(4), ba(3), bg(3), bbaro(1)]  -> 16 states
%   Error:   [dp(3), dv(3), dtheta(3), dba(3), dbg(3), dbbaro(1)] -> 16 errors
%
% MATLAB: R2021b or newer
% Toolboxes: Statistics/ML (knnsearch), Optimization (lsqnonlin)
% =========================================================================
function results = run_S2_unified_6dof(seed, runStress, makePlots, makeAnimation)
%RUN_S2_UNIFIED_6DOF Corrected Stage-S2 multi-lane ESKF regression.
%   results = run_S2_unified_6dof(seed,runStress,makePlots,makeAnimation)
%
% The estimator remains a 16-error-state quaternion ESKF.  This corrected
% version fixes frame consistency, gravity linearisation, per-lane LiDAR
% fusion, lane observability/failover, scenario injection and returned data.
%
% makeAnimation is accepted for interface compatibility.  Animation remains
% the responsibility of the production animate_S2_flight.m function.

if nargin < 1 || isempty(seed),          seed = 0;          end
if nargin < 2 || isempty(runStress),     runStress = false; end
if nargin < 3 || isempty(makePlots),     makePlots = true;  end
if nargin < 4 || isempty(makeAnimation), makeAnimation = false; end

validateattributes(seed, {'numeric'}, {'scalar','integer','nonnegative','finite'});
validateattributes(runStress, {'logical','numeric'}, {'scalar'});
validateattributes(makePlots, {'logical','numeric'}, {'scalar'});
validateattributes(makeAnimation, {'logical','numeric'}, {'scalar'});
runStress = logical(runStress);
makePlots = logical(makePlots);
makeAnimation = logical(makeAnimation);

fprintf('\n============================================================\n');
fprintf('  S2 UNIFIED FIXED: Multi-Lane ESKF + Pose Graph + RTL\n');
fprintf('============================================================\n\n');

cfg = defaultConfig_S2();

nominalScenario = defaultScenario_S2('nominal');
results = struct();
results.nominal = runTrial_S2(seed, nominalScenario, cfg, makePlots);
results.nominal.animationFile = '';
printResult_S2(results.nominal, cfg);

if runStress
    stressScenario = defaultScenario_S2('stress');
    results.stress = runTrial_S2(seed, stressScenario, cfg, makePlots);
    results.stress.animationFile = '';
    printResult_S2(results.stress, cfg);
    if cfg.runStressRegression
        results.stressRegression = runStressRegression_S2(cfg, seed);
    else
        results.stressRegression = [];
    end
else
    results.stress = [];
    results.stressRegression = [];
end

if cfg.runMonteCarlo
    fprintf('\n=== NOMINAL MONTE CARLO ===\n');
    seeds = cfg.monteCarloSeeds;
    rows = repmat(emptySummaryRow_S2(), numel(seeds), 1);
    for ii = 1:numel(seeds)
        trial = runTrial_S2(seeds(ii), nominalScenario, cfg, false);
        rows(ii) = summaryRow_S2(trial);
        fprintf('seed=%2d | max5=%6.2f cm | RMSE5=%6.2f cm | lane=%d | loops=%d | pass=%d\n', ...
            seeds(ii), 100*trial.summary.fusedMaxAfter5_m, ...
            100*trial.summary.fusedRMSEAfter5_m, ...
            trial.summary.finalLane, trial.summary.verifiedLoops, ...
            trial.summary.pass);
    end
    results.monteCarlo = summarizeRows_S2(rows, cfg);
else
    results.monteCarlo = [];
end

if makeAnimation
    warning('S2:AnimationNotInUnifiedFile', ...
        ['This corrected unified regression does not contain the production ' ...
         'animate_S2_flight.m implementation. animationFile is returned empty.']);
end

fprintf('\n*** S2 unified fixed run completed. ***\n');

%% =========================================================================
% CONFIGURATION
%% =========================================================================
function cfg = defaultConfig_S2()
    % Timing
    cfg.duration = 60;
    cfg.imuRate = 200;
    cfg.vioRate = 30;
    cfg.lidarRate = 5.5;
    cfg.rangeRate = 30;
    cfg.baroRate = 25;

    % Regression controls. Keep false during ordinary production runs.
    cfg.runMonteCarlo = false;
    cfg.monteCarloSeeds = 0:9;
    cfg.runStressRegression = false;

    % Room
    cfg.room = [6 6 2.5];
    cfg.obstacles = [1.0 1.0 0.5 0.5; 4.0 3.5 0.5 0.5];
    cfg.requirement_m = 0.10;
    cfg.attRequirement_deg = 2.0;
    cfg.gW = [0; 0; -9.81];

    % Sensor extrinsics
    cfg.r_BC = [0.08; 0.00; 0.02]; cfg.R_BC = eye(3);
    cfg.r_BL = [0.00; 0.00; 0.05]; cfg.R_BL = eye(3);
    cfg.r_BR = [0.00; 0.00;-0.05]; cfg.d_BR = [0; 0; -1];

    % IMU noise densities / random walks
    cfg.accelND = 0.003;
    cfg.gyroND = deg2rad(0.025);
    cfg.accelBiasRW = 2e-4;
    cfg.gyroBiasRW = deg2rad(0.002);

    % VIO
    cfg.vioPosSigma = 0.015;
    cfg.vioVelSigma = 0.025;
    cfg.vioAttSigma = deg2rad(0.45);
    cfg.vioDriftPosRW = 8e-4;
    cfg.vioDriftAttRW = deg2rad(0.012);
    cfg.vioOutlierProb = 0.005;

    % LiDAR / ICP
    cfg.lidarSigmaXY = 0.025;
    cfg.lidarSigmaYaw = deg2rad(0.7);
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

    % Range / barometer
    cfg.rangeSigma = 0.012;
    cfg.baroSigma = 0.06;
    cfg.baroBiasRW = 0.002;

    % Gravity pseudo-measurement. It is enabled only near 1 g.
    cfg.gravitySigma = 0.15;
    cfg.gravityAccelNormGate = 0.35;

    % Observability-aware lane selection
    cfg.laneNisWindow = 20;
    cfg.minLaneObservations = 5;
    cfg.horizontalAidTimeout = 0.75;
    cfg.maxXYCovariance = 0.30^2;
    cfg.maxVelXYCovariance = 0.60^2;
    cfg.laneSwitchMargin = 0.12;
    cfg.laneMinDwellTime = 1.0;
    cfg.maxLanePositionJump = 0.35;
    cfg.maxLaneAttitudeJump = deg2rad(8);
    cfg.laneCovarianceWeight = 0.15;
    cfg.lanePriorityPenalty = [0.00 0.02 0.05 1.00];

    % RTL
    cfg.rtlAltitude = 2.0;
    cfg.rtlHome = [3.0; 3.0];
    cfg.rtlSpeedXY = 0.5;
    cfg.rtlDescentRate = 0.3;

    % ScanContext + pose graph
    cfg.enableScanContext = true;
    cfg.scRings = 20;
    cfg.scSectors = 60;
    cfg.scExcludeRecent = 45;
    cfg.scThreshold = 0.24;
    cfg.scVerifyRMSE = 0.065;
    cfg.keyframeStride = 5;
    cfg.maxLoopClosures = 8;

    % Chi-square gates
    cfg.gateVIO9 = 27.877;
    cfg.gateLidar3 = 16.266;
    cfg.gate1 = 10.828;
    cfg.gateGravity3 = 16.266;
end

function scenario = defaultScenario_S2(mode)
    scenario = struct();
    scenario.name = upper(mode);
    scenario.motorFailTime = inf;
    scenario.lidarFailTime = inf;
    scenario.camFailTime = inf;
    scenario.vioOutageWindows = zeros(0,2);
    scenario.rangeOutageWindows = zeros(0,2);
    scenario.lidarDropProbability = 0;
    scenario.lidarFalsePoseProbability = 0;
    scenario.lidarFalseTranslationSigma = 0.30;
    scenario.lidarFalseYawSigma = deg2rad(8);

    switch lower(mode)
        case 'nominal'
            scenario.name = 'NOMINAL';
        case 'stress'
            scenario.name = 'STRESS';
            scenario.vioOutageWindows = [25 32];
            scenario.rangeOutageWindows = [18 38];
            scenario.lidarDropProbability = 0.20;
            scenario.lidarFalsePoseProbability = 0.02;
        otherwise
            error('Unknown S2 scenario mode: %s', mode);
    end
end

%% =========================================================================
% SENSOR SIMULATION (6-DOF with extrinsics)
%% =========================================================================
function [gt, meas, failureLog] = simulateSensors_S2(seed, cfg, scenario)
    rng(seed, 'twister');
    dt = 1/cfg.imuRate;
    t = (0:dt:cfg.duration)';
    N = numel(t);

    % Trajectory
    r = 1.5; w1 = 2*pi/30; w2 = 4*pi/30;
    p = [3+r*sin(w1*t), 3+0.6*r*sin(w2*t), 1.15+0.16*sin(2*pi*t/18)];
    v = [r*w1*cos(w1*t), 0.6*r*w2*cos(w2*t), 0.16*(2*pi/18)*cos(2*pi*t/18)];
    a = [-r*w1^2*sin(w1*t), -0.6*r*w2^2*sin(w2*t), -0.16*(2*pi/18)^2*sin(2*pi*t/18)];
    yaw = unwrap(atan2(v(:,2),v(:,1)));
    roll = deg2rad(5)*sin(2*pi*t/8);
    pitch = deg2rad(4)*sin(2*pi*t/10+0.4);

    N = numel(t); q = zeros(N,4);
    for k=1:N, q(k,:) = rpy2q(roll(k),pitch(k),yaw(k)); end

    % Angular velocity from quaternion derivative (Forster 2017)
    omega = zeros(N,3);
    for k=1:N-1
        dq = qmul(qconj(q(k,:)), q(k+1,:));
        omega(k,:) = qlog(dq)/dt;
    end
    omega(end,:) = omega(end-1,:);

    % Scenario-controlled failure injection. Nominal means no forced failure.
    motorFailTime = scenario.motorFailTime;
    lidarFailTime = scenario.lidarFailTime;
    camFailTime = scenario.camFailTime;

    failureLog.motorFailTime = motorFailTime;
    failureLog.lidarFailTime = lidarFailTime;
    failureLog.camFailTime = camFailTime;
    failureLog.motorFailCount = 0;

    % RTL trajectory if motor fails
    if motorFailTime < cfg.duration
        failureLog.motorFailCount = 1;
        [p, v, a, q, yaw, omega] = generateRTLTrajectory_S2(t, p, v, a, q, yaw, omega, cfg, motorFailTime);
    end

    % IMU (primary + backup)
    ba = zeros(N,3); bg = zeros(N,3);
    ba(1,:) = [0.018 -0.014 0.012];
    bg(1,:) = deg2rad([0.06 -0.04 0.05]);
    for k=2:N
        ba(k,:) = ba(k-1,:) + cfg.accelBiasRW*sqrt(dt)*randn(1,3);
        bg(k,:) = bg(k-1,:) + cfg.gyroBiasRW*sqrt(dt)*randn(1,3);
    end
    sa = cfg.accelND/sqrt(dt); sg = cfg.gyroND/sqrt(dt);

    accBody = zeros(N,3); gyroBody = zeros(N,3);
    for k=1:N
        R = q2R(q(k,:));
        accBody(k,:) = (R'*(a(k,:)' - cfg.gW) + ba(k,:)' + sa*randn(3,1))';
        gyroBody(k,:) = (omega(k,:)' + bg(k,:)' + sg*randn(3,1))';
    end

    % Backup IMU (higher noise)
    baB = ba + [0.005 -0.003 0.002];
    bgB = bg + deg2rad([0.010 -0.008 0.005]);
    accBodyB = zeros(N,3); gyroBodyB = zeros(N,3);
    for k=1:N
        R = q2R(q(k,:));
        accBodyB(k,:) = (R'*(a(k,:)' - cfg.gW) + baB(k,:)' + sa*randn(3,1))';
        gyroBodyB(k,:) = (omega(k,:)' + bgB(k,:)' + sg*randn(3,1))';
    end

    % VIO (D435i host, S2 style with extrinsics)
    tv = (0:1/cfg.vioRate:cfg.duration)';
    iv = timeToIndex_S2(tv, cfg.imuRate, N);
    M = numel(tv);
    dp = zeros(N,3); dth = zeros(N,3);
    for k=2:N
        dp(k,:) = dp(k-1,:) + cfg.vioDriftPosRW*sqrt(dt)*randn(1,3);
        dth(k,:) = dth(k-1,:) + cfg.vioDriftAttRW*sqrt(dt)*randn(1,3);
    end
    vioP = zeros(M,3); vioV = zeros(M,3); vioQ = zeros(M,4);
    for i=1:M
        k = iv(i); RWB = q2R(q(k,:)); RWC = RWB*cfg.R_BC;
        pWC = p(k,:)' + RWB*cfg.r_BC;
        RWCm = RWC*q2R(qexp(dth(k,:)+cfg.vioAttSigma*randn(1,3)));
        pWCm = pWC + dp(k,:)' + cfg.vioPosSigma*randn(3,1);
        RWBm = RWCm*cfg.R_BC';
        vioP(i,:) = (pWCm - RWBm*cfg.r_BC)';
        vioQ(i,:) = R2q(RWBm);
        vioV(i,:) = v(k,:) + cfg.vioVelSigma*randn(1,3);
    end
    vioValid = true(M,1);
    vioOutlier = rand(M,1) < cfg.vioOutlierProb; vioOutlier(1) = false;
    vioP(vioOutlier,:) = vioP(vioOutlier,:) + 0.35*randn(nnz(vioOutlier),3);

    % Range (VL53L0X, tilt-aware)
    tr = (0:1/cfg.rangeRate:cfg.duration)';
    ir = timeToIndex_S2(tr, cfg.imuRate, N);
    zr = zeros(numel(tr),1);
    for i=1:numel(tr)
        k = ir(i); R = q2R(q(k,:));
        pS = p(k,:)' + R*cfg.r_BR; dW = R*cfg.d_BR;
        zr(i) = -pS(3)/dW(3) + cfg.rangeSigma*randn;
    end
    rangeValid = rand(numel(tr),1) > 0.05;

    % Barometer
    tb = (0:1/cfg.baroRate:cfg.duration)';
    ib = timeToIndex_S2(tb, cfg.imuRate, N);
    bbaro = zeros(numel(tb),1); bbaro(1) = 0.12;
    for i=2:numel(tb), bbaro(i) = bbaro(i-1) + cfg.baroBiasRW*sqrt(1/cfg.baroRate)*randn; end
    zb = p(ib,3) + bbaro + cfg.baroSigma*randn(numel(tb),1);

    % LiDAR
    tl = (0:1/cfg.lidarRate:cfg.duration)';
    il = timeToIndex_S2(tl, cfg.imuRate, N);
    scans = simulateLidarScans_S2(p, q, tl, il, cfg);

    % Pack
    gt = struct('t',t,'p',p,'v',v,'a',a,'q',q,'omega',omega,'rpy',[roll pitch yaw]);
    meas = struct('acc0',accBody,'gyro0',gyroBody,'acc1',accBodyB,'gyro1',gyroBodyB, ...
                  'tVio',tv,'iVio',iv,'vioP',vioP,'vioV',vioV,'vioQ',vioQ,'vioValid',vioValid,'vioOutlier',vioOutlier, ...
                  'tRange',tr,'iRange',ir,'zr',zr,'rangeValid',rangeValid, ...
                  'tBaro',tb,'iBaro',ib,'zb',zb,'baroBiasTrue',bbaro, ...
                  'tLidar',tl,'iLidar',il,'scans',{scans}, ...
                   'lidarValid',true(numel(tl),1),'lidarFalse',false(numel(tl),1));
end

%% =========================================================================
% RTL TRAJECTORY GENERATOR
%% =========================================================================
function [p, v, a, q, yaw, omega] = generateRTLTrajectory_S2(t, p, v, a, q, yaw, omega, cfg, tFail)
    dt = t(2)-t(1); N = numel(t);
    idxFail = find(t >= tFail, 1, 'first');
    if isempty(idxFail), return; end

    for k = idxFail:N
        currentP = p(k-1,:)';
        currentV = v(k-1,:)';

        if currentP(3) < cfg.rtlAltitude - 0.1
            target = [currentP(1:2); cfg.rtlAltitude];
            dir = target - currentP;
            if norm(dir) > 0.01, dir = dir/norm(dir); end
            desiredV = dir * cfg.rtlSpeedXY;
        else
            distXY = norm(currentP(1:2) - cfg.rtlHome);
            if distXY > 0.3
                dir = [cfg.rtlHome; cfg.rtlAltitude] - currentP;
                if norm(dir) > 0.01, dir = dir/norm(dir); end
                desiredV = dir * cfg.rtlSpeedXY;
            else
                desiredV = [0; 0; -cfg.rtlDescentRate];
            end
        end

        alpha = 0.05;
        v(k,:) = (1-alpha)*v(k-1,:) + alpha*desiredV';
        p(k,:) = p(k-1,:) + v(k,:)*dt;
        p(k,3) = max(0.15, p(k,3));
    end

    v = gradient(p, dt);
    a = gradient(v, dt);
    yaw = unwrap(atan2(v(:,2), v(:,1)));
    roll = atan2(a(:,2).*cos(yaw) - a(:,1).*sin(yaw), 9.81) * 0.1;
    pitch = atan2(-(a(:,1).*cos(yaw) + a(:,2).*sin(yaw)), 9.81) * 0.1;
    for k=1:N, q(k,:) = rpy2q(roll(k), pitch(k), yaw(k)); end

    omega = zeros(N,3);
    for k=1:N-1
        dq = qmul(qconj(q(k,:)), q(k+1,:));
        omega(k,:) = qlog(dq)/dt;
    end
    omega(end,:) = omega(end-1,:);
end

%% =========================================================================
% LIDAR RAYCASTING (with extrinsics)
%% =========================================================================
function scans = simulateLidarScans_S2(p, q, tLidar, iLidar, cfg)
    angles = linspace(0, 2*pi, cfg.nBeams+1)';
    angles(end) = [];
    scans = cell(numel(tLidar), 1);

    for i = 1:numel(tLidar)
        k = iLidar(i);
        RWB = q2R(q(k,:));
        pL = p(k,:)' + RWB*cfg.r_BL;
        RWL = RWB*cfg.R_BL;
        yaw = atan2(RWL(2,1), RWL(1,1));
        ranges = zeros(cfg.nBeams, 1);

        for b = 1:cfg.nBeams
            wa = angles(b) + yaw;
            dx = cos(wa); dy = sin(wa);
            cand = [];
            if abs(dx) > 1e-12
                cand = [cand, (0-pL(1))/dx, (cfg.room(1)-pL(1))/dx]; %#ok<AGROW>
            end
            if abs(dy) > 1e-12
                cand = [cand, (0-pL(2))/dy, (cfg.room(2)-pL(2))/dy]; %#ok<AGROW>
            end
            cand = cand(cand > 1e-6);
            hit = min(cand);

            for o = 1:size(cfg.obstacles,1)
                obs = cfg.obstacles(o,:);
                ox = obs(1); oy = obs(2); ow = obs(3); od = obs(4);
                if abs(dx) > 1e-12, tx1 = (ox-pL(1))/dx; tx2 = (ox+ow-pL(1))/dx;
                else, tx1 = -inf; tx2 = inf; end
                if abs(dy) > 1e-12, ty1 = (oy-pL(2))/dy; ty2 = (oy+od-pL(2))/dy;
                else, ty1 = -inf; ty2 = inf; end
                te = max(min(tx1,tx2), min(ty1,ty2));
                tx = min(max(tx1,tx2), max(ty1,ty2));
                if te > 0 && te < tx && te < hit, hit = te; end
            end
            ranges(b) = min(12, max(0.15, hit + cfg.rangeNoise*randn));
        end
        scans{i} = [ranges.*cos(angles), ranges.*sin(angles)];
    end
end

function idx = timeToIndex_S2(t, rate, N)
    idx = min(N, max(1, round(t*rate) + 1));
end

%% =========================================================================
% MULTI-LANE ESKF CORE
%% =========================================================================
function [est, localLidar] = runMultiLaneESKF(gt, meas, cfg, failureLog)
    N = numel(gt.t);
    dt = 1/cfg.imuRate;
    nLanes = 4;
    lanes = initAllLanes_S2(nLanes, cfg, meas);

    pHist = zeros(N,3); qHist = zeros(N,4); baHist = zeros(N,3);
    bgHist = zeros(N,3); bbHist = zeros(N,1);
    laneHist = ones(N,1); rtlActive = false(N,1);
    degradedHist = false(N,1); laneScoreHist = nan(N,nLanes);
    xyCovHist = nan(N,nLanes);

    currentLane = 1;
    lastSwitchTime = -inf;
    pHist(1,:) = lanes(currentLane).p';
    qHist(1,:) = lanes(currentLane).q;
    baHist(1,:) = lanes(currentLane).ba';
    bgHist(1,:) = lanes(currentLane).bg';
    bbHist(1) = lanes(currentLane).bbaro;

    kv = 2; kl = 1; kr = 2; kb = 2;
    L = numel(meas.tLidar);
    pose = zeros(L,3); valid = false(L,1);
    ldiag = nan(L,5); mapBlocks = {};

    lidarFailTime = failureLog.lidarFailTime;
    camFailTime = failureLog.camFailTime;
    motorFailTime = failureLog.motorFailTime;

    counts = struct('vioAcc',0,'vioRej',0,'lidarAcc',0,'lidarRej',0, ...
        'rangeAcc',0,'rangeRej',0,'baroAcc',0,'baroRej',0, ...
        'gravAcc',0,'gravRej',0,'laneSwitches',0);
    nis = struct('vio',[],'lidar',[],'range',[],'baro',[],'gravity',[]);

    Rvio = diag([repmat(cfg.vioPosSigma^2,1,3), ...
        repmat(cfg.vioVelSigma^2,1,3), repmat(cfg.vioAttSigma^2,1,3)]);
    Rlid = diag([cfg.lidarSigmaXY^2, cfg.lidarSigmaXY^2, cfg.lidarSigmaYaw^2]);

    % Process every measurement whose timestamp is not later than the IMU epoch.
    tNow = gt.t(1);
    while kl <= L && meas.tLidar(kl) <= tNow + eps
        [lanes, pose, valid, ldiag, mapBlocks, counts, nis] = processLidarMultiLane( ...
            lanes, currentLane, kl, meas.scans, meas.lidarValid(kl), pose, valid, ...
            ldiag, mapBlocks, counts, nis, Rlid, cfg, lidarFailTime, tNow);
        kl = kl + 1;
    end

    for k = 2:N
        tNow = gt.t(k);
        rtlActive(k) = tNow >= motorFailTime && motorFailTime < cfg.duration;

        for laneId = 1:nLanes
            lanes(laneId) = propagateLaneESKF(lanes(laneId), meas, k, dt, cfg);
        end

        while kv <= numel(meas.tVio) && meas.tVio(kv) <= tNow + 1e-12
            if meas.vioValid(kv) && tNow < camFailTime
                for laneId = 1:nLanes
                    if laneId == 4, continue; end
                    [lanes(laneId), counts, nis] = updateVIO( ...
                        lanes(laneId), meas, kv, Rvio, cfg, counts, nis, tNow);
                end
            end
            kv = kv + 1;
        end

        while kl <= L && meas.tLidar(kl) <= tNow + 1e-12
            [lanes, pose, valid, ldiag, mapBlocks, counts, nis] = processLidarMultiLane( ...
                lanes, currentLane, kl, meas.scans, meas.lidarValid(kl), pose, valid, ...
                ldiag, mapBlocks, counts, nis, Rlid, cfg, lidarFailTime, tNow);
            kl = kl + 1;
        end

        while kr <= numel(meas.tRange) && meas.tRange(kr) <= tNow + 1e-12
            if meas.rangeValid(kr)
                for laneId = [1 4]
                    [lanes(laneId), counts, nis] = updateRange( ...
                        lanes(laneId), meas, kr, cfg, counts, nis, tNow);
                end
            end
            kr = kr + 1;
        end

        while kb <= numel(meas.tBaro) && meas.tBaro(kb) <= tNow + 1e-12
            for laneId = [1 4]
                [lanes(laneId), counts, nis] = updateBaro( ...
                    lanes(laneId), meas, kb, cfg, counts, nis, tNow);
            end
            kb = kb + 1;
        end

        if mod(k, round(cfg.imuRate/10)) == 0
            for laneId = 1:nLanes
                [lanes(laneId), counts, nis] = updateGravity( ...
                    lanes(laneId), meas, k, cfg, counts, nis, tNow);
            end
        end

        [bestLane, healthScores, degraded] = laneSelector_S2( ...
            lanes, cfg, tNow, currentLane, lastSwitchTime);
        if bestLane ~= currentLane
            currentLane = bestLane;
            lastSwitchTime = tNow;
            counts.laneSwitches = counts.laneSwitches + 1;
        end

        laneHist(k) = currentLane;
        degradedHist(k) = degraded;
        laneScoreHist(k,:) = healthScores(:)';
        for laneId = 1:nLanes
            xyCovHist(k,laneId) = trace(lanes(laneId).P(1:2,1:2));
        end

        pHist(k,:) = lanes(currentLane).p';
        qHist(k,:) = lanes(currentLane).q;
        baHist(k,:) = lanes(currentLane).ba';
        bgHist(k,:) = lanes(currentLane).bg';
        bbHist(k) = lanes(currentLane).bbaro;
    end

    posErr = vecnorm(pHist - gt.p, 2, 2);
    attErr = zeros(N,1);
    for k = 1:N
        attErr(k) = norm(qlog(qmul(qconj(qHist(k,:)), gt.q(k,:))));
    end

    ss = gt.t >= 5;
    est = struct('p',pHist,'q',qHist,'ba',baHist,'bg',bgHist,'bbaro',bbHist, ...
        'posErr',posErr,'attErr',attErr,'laneHist',laneHist,'rtlActive',rtlActive, ...
        'degraded',degradedHist,'laneScores',laneScoreHist,'xyCovariance',xyCovHist, ...
        'counts',counts,'nis',nis, ...
        'unhealthySamples',nnz(ss & posErr >= cfg.requirement_m));
    localLidar = struct('pose',pose,'valid',valid,'diag',ldiag);
end

%% =========================================================================
% LANE INITIALIZATION (ESKF)
%% =========================================================================
function lanes = initAllLanes_S2(nLanes, cfg, meas)
    q0 = qnormalize(meas.vioQ(1,:));
    lanes = struct('id',{},'p',{},'v',{},'q',{},'ba',{},'bg',{},'bbaro',{}, ...
        'P',{},'nisNormalized',{},'nisType',{},'acceptedCount',{}, ...
        'rejectedCount',{},'lastHorizontalAidTime',{}, ...
        'lastVerticalAidTime',{},'lastAttitudeAidTime',{});

    s = [repmat(0.05,1,3), repmat(0.12,1,3), deg2rad([3 3 4]), ...
         repmat(0.06,1,3), deg2rad([0.4 0.4 0.4]), 0.20];
    P0 = diag(s.^2);

    for laneId = 1:nLanes
        lanes(laneId).id = laneId;
        lanes(laneId).p = meas.vioP(1,:)';
        lanes(laneId).v = meas.vioV(1,:)';
        lanes(laneId).q = q0;
        lanes(laneId).ba = zeros(3,1);
        lanes(laneId).bg = zeros(3,1);
        lanes(laneId).bbaro = 0.12;
        lanes(laneId).P = P0;
        lanes(laneId).nisNormalized = [];
        lanes(laneId).nisType = {};
        lanes(laneId).acceptedCount = 0;
        lanes(laneId).rejectedCount = 0;
        if laneId <= 3
            lanes(laneId).lastHorizontalAidTime = 0;
            lanes(laneId).lastAttitudeAidTime = 0;
        else
            lanes(laneId).lastHorizontalAidTime = -inf;
            lanes(laneId).lastAttitudeAidTime = -inf;
        end
        lanes(laneId).lastVerticalAidTime = 0;
    end
end

%% =========================================================================
% ESKF PREDICTION (Forster et al. 2017)
%% =========================================================================
function lane = propagateLaneESKF(lane, meas, k, dt, cfg)
    % Select IMU per lane
    if lane.id == 2
        acc = meas.acc1(k,:)';
        gyro = meas.gyro1(k,:)';
    else
        acc = meas.acc0(k,:)';
        gyro = meas.gyro0(k,:)';
    end

    fb = acc - lane.ba;
    w = gyro - lane.bg;
    R = q2R(lane.q);
    aw = R*fb + cfg.gW;

    % Nominal state propagation
    lane.p = lane.p + lane.v*dt + 0.5*aw*dt^2;
    lane.v = lane.v + aw*dt;
    lane.q = qnormalize(qmul(lane.q, qexp((w*dt).')));

    % Error-state Jacobian (F) and noise matrix (G)
    F = zeros(16); F(1:3,4:6) = eye(3);
    F(4:6,7:9) = -R*skew3(fb); F(4:6,10:12) = -R;
    F(7:9,7:9) = -skew3(w); F(7:9,13:15) = -eye(3);
    Phi = eye(16) + F*dt;

    G = zeros(16,13);
    G(4:6,1:3) = -R; G(7:9,4:6) = -eye(3);
    G(10:12,7:9) = eye(3); G(13:15,10:12) = eye(3); G(16,13) = 1;

    qc = [repmat(cfg.accelND^2,1,3), repmat(cfg.gyroND^2,1,3), ...
          repmat(cfg.accelBiasRW^2,1,3), repmat(cfg.gyroBiasRW^2,1,3), cfg.baroBiasRW^2];

    lane.P = Phi*lane.P*Phi' + G*diag(qc)*G'*dt;
    lane.P = (lane.P + lane.P')/2;
end

%% =========================================================================
% VIO UPDATE (9-DOF with extrinsics)
%% =========================================================================
function [lane, counts, nisLog] = updateVIO(lane, meas, kv, Rvio, cfg, counts, nisLog, tNow)
    % meas.vioP/vioQ are body-origin pose measurements. Do not reapply the
    % camera lever arm here; doing so creates a deterministic frame error.
    r = [meas.vioP(kv,:)' - lane.p; ...
         meas.vioV(kv,:)' - lane.v; ...
         qlog(qmul(qconj(lane.q), meas.vioQ(kv,:)))'];

    H = zeros(9,16);
    H(1:3,1:3) = eye(3);
    H(4:6,4:6) = eye(3);
    H(7:9,7:9) = eye(3);

    [lane, ok, n] = filterUpdateESKF(lane, r, H, Rvio, cfg.gateVIO9);
    lane = recordLaneNIS_S2(lane, n, cfg.gateVIO9, 'vio', ok);
    if isfinite(n), nisLog.vio(end+1,1) = n; end
    if ok
        counts.vioAcc = counts.vioAcc + 1;
        lane.lastHorizontalAidTime = tNow;
        lane.lastVerticalAidTime = tNow;
        lane.lastAttitudeAidTime = tNow;
    else
        counts.vioRej = counts.vioRej + 1;
    end
end

%% =========================================================================
% LIDAR UPDATE (with extrinsics and ICP health)
%% =========================================================================
function [lanes, pose, valid, ldiag, mapBlocks, counts, nisLog] = processLidarMultiLane( ...
    lanes, activeLane, i, scans, sensorAvailable, pose, valid, ldiag, ...
    mapBlocks, counts, nisLog, Rlid, cfg, lidarFailTime, tNow)

    priorLane = activeLane;
    if priorLane > 2 || ~isfinite(trace(lanes(priorLane).P(1:2,1:2)))
        priorLane = 1;
    end
    [pred, ~] = lidarMeasurementESKF(lanes(priorLane), cfg);

    rmse = inf; overlap = 0; healthy = false;
    z = pred;
    if sensorAvailable && tNow < lidarFailTime
        if isempty(mapBlocks)
            rmse = 0;
            overlap = size(scans{i},1);
            healthy = true;
        else
            first = max(1, numel(mapBlocks) - cfg.localSubmapScans + 1);
            mappts = voxelDown_S2(vertcat(mapBlocks{first:end}), cfg.mapVoxel);
            [z, rmse, overlap] = icpToMap_S2(scans{i}, mappts, pred, cfg);
            c = se2Between_S2(pred, z);
            healthy = isfinite(rmse) && rmse < cfg.icpHealthRMSE && ...
                overlap >= cfg.icpHealthOverlap && ...
                norm(c(1:2)) < cfg.icpHealthCorrection && ...
                abs(c(3)) < cfg.icpHealthYaw;
        end
    end

    anyOk = false;
    nReported = nan;
    attempted = false;
    if healthy
        attempted = true;
        for laneId = 1:2
            [h_i, H_i] = lidarMeasurementESKF(lanes(laneId), cfg);
            r_i = z - h_i;
            r_i(3) = wrapPi(r_i(3));
            [lanes(laneId), ok_i, n_i] = filterUpdateESKF( ...
                lanes(laneId), r_i', H_i, Rlid, cfg.gateLidar3);
            lanes(laneId) = recordLaneNIS_S2( ...
                lanes(laneId), n_i, cfg.gateLidar3, 'lidar', ok_i);
            if isfinite(n_i), nisLog.lidar(end+1,1) = n_i; end
            if isnan(nReported) || (isfinite(n_i) && n_i < nReported)
                nReported = n_i;
            end
            if ok_i
                anyOk = true;
                lanes(laneId).lastHorizontalAidTime = tNow;
                lanes(laneId).lastAttitudeAidTime = tNow;
            end
        end
    end

    if anyOk
        pose(i,:) = z;
        valid(i) = true;
        mapBlocks{end+1} = transformPoints_S2(scans{i}(1:2:end,:), z); %#ok<AGROW>
        counts.lidarAcc = counts.lidarAcc + 1;
    else
        pose(i,:) = pred;
        if attempted || sensorAvailable
            counts.lidarRej = counts.lidarRej + 1;
        end
    end
    ldiag(i,:) = [rmse, overlap, healthy, anyOk, nReported];
end

%% =========================================================================
% RANGE UPDATE (tilt-aware with extrinsics)
%% =========================================================================
function [lane, counts, nisLog] = updateRange(lane, meas, kr, cfg, counts, nisLog, tNow)
    [h, H] = rangeMeasurementESKF(lane, cfg);
    if ~isfinite(h)
        counts.rangeRej = counts.rangeRej + 1;
        return;
    end
    r = meas.zr(kr) - h;
    [lane, ok, n] = filterUpdateESKF(lane, r, H, cfg.rangeSigma^2, cfg.gate1);
    lane = recordLaneNIS_S2(lane, n, cfg.gate1, 'range', ok);
    if isfinite(n), nisLog.range(end+1,1) = n; end
    if ok
        counts.rangeAcc = counts.rangeAcc + 1;
        lane.lastVerticalAidTime = tNow;
    else
        counts.rangeRej = counts.rangeRej + 1;
    end
end

%% =========================================================================
% BAROMETER UPDATE
%% =========================================================================
function [lane, counts, nisLog] = updateBaro(lane, meas, kb, cfg, counts, nisLog, tNow)
    r = meas.zb(kb) - (lane.p(3) + lane.bbaro);
    H = zeros(1,16); H(3) = 1; H(16) = 1;
    [lane, ok, n] = filterUpdateESKF(lane, r, H, cfg.baroSigma^2, cfg.gate1);
    lane = recordLaneNIS_S2(lane, n, cfg.gate1, 'baro', ok);
    if isfinite(n), nisLog.baro(end+1,1) = n; end
    if ok
        counts.baroAcc = counts.baroAcc + 1;
        lane.lastVerticalAidTime = tNow;
    else
        counts.baroRej = counts.baroRej + 1;
    end
end

%% =========================================================================
% GRAVITY UPDATE (Qin et al. 2018)
%% =========================================================================
function [lane, counts, nisLog] = updateGravity(lane, meas, k, cfg, counts, nisLog, tNow)
    if lane.id == 2
        accel = meas.acc1(k,:)';
    else
        accel = meas.acc0(k,:)';
    end

    specificForce = accel - lane.ba;
    if abs(norm(specificForce) - norm(cfg.gW)) > cfg.gravityAccelNormGate
        counts.gravRej = counts.gravRej + 1;
        return;
    end

    R = q2R(lane.q);
    gBodyPred = R'*(-cfg.gW);
    h = gBodyPred + lane.ba;
    r = accel - h;

    H = zeros(3,16);
    % Positive sign is required by q_true = q_nominal * Exp(delta_theta).
    H(:,7:9) = skew3(gBodyPred);
    H(:,10:12) = eye(3);

    Rmat = cfg.gravitySigma^2 * eye(3);
    [lane, ok, n] = filterUpdateESKF(lane, r, H, Rmat, cfg.gateGravity3);
    lane = recordLaneNIS_S2(lane, n, cfg.gateGravity3, 'gravity', ok);
    if isfinite(n), nisLog.gravity(end+1,1) = n; end
    if ok
        counts.gravAcc = counts.gravAcc + 1;
        lane.lastAttitudeAidTime = tNow;
    else
        counts.gravRej = counts.gravRej + 1;
    end
end

%% =========================================================================
% ESKF FILTER UPDATE (error-state)
%% =========================================================================
function [lane, ok, nis] = filterUpdateESKF(lane, r, H, Rm, gate)
    r = r(:);
    S = H*lane.P*H' + Rm;
    nis = r'*(S\r);
    if ~isfinite(nis) || nis > gate, ok = false; return; end

    K = (lane.P*H')/S;
    dx = K*r;

    % Inject error into nominal state
    lane.p = lane.p + dx(1:3);
    lane.v = lane.v + dx(4:6);
    lane.q = qnormalize(qmul(lane.q, qexp(dx(7:9).')));
    lane.ba = lane.ba + dx(10:12);
    lane.bg = lane.bg + dx(13:15);
    lane.bbaro = lane.bbaro + dx(16);

    % Reset error state and update covariance
    I = eye(16); J = I - K*H;
    lane.P = J*lane.P*J' + K*Rm*K';
    G = eye(16); G(7:9,7:9) = eye(3) - 0.5*skew3(dx(7:9));
    lane.P = G*lane.P*G';
    lane.P = (lane.P + lane.P')/2;
    ok = true;
end


function lane = recordLaneNIS_S2(lane, nis, gate, typeName, accepted)
    if isfinite(nis) && isfinite(gate) && gate > 0
        lane.nisNormalized(end+1) = nis/gate;
        lane.nisType{end+1} = typeName;
    end
    if accepted
        lane.acceptedCount = lane.acceptedCount + 1;
    else
        lane.rejectedCount = lane.rejectedCount + 1;
    end
end

%% =========================================================================
% MEASUREMENT MODELS WITH EXTRINSICS
%% =========================================================================
function [h, H] = lidarMeasurementESKF(lane, cfg)
    R = q2R(lane.q);
    pL = lane.p + R*cfg.r_BL;
    RWL = R*cfg.R_BL;
    h = [pL(1), pL(2), atan2(RWL(2,1), RWL(1,1))];
    H = zeros(3,16);
    H(1,1) = 1; H(2,2) = 1;
    A = -R*skew3(cfg.r_BL);
    H(1:2,7:9) = A(1:2,:);

    base = h(3); epsa = 1e-7;
    for j = 1:3
        d = zeros(1,3); d(j) = epsa;
        Rp = R*q2R(qexp(d));
        RWLp = Rp*cfg.R_BL;
        H(3,6+j) = wrapPi(atan2(RWLp(2,1), RWLp(1,1)) - base)/epsa;
    end
end

function [h, H] = rangeMeasurementESKF(lane, cfg)
    R = q2R(lane.q);
    pS = lane.p + R*cfg.r_BR;
    dW = R*cfg.d_BR;
    den = dW(3);
    H = zeros(1,16);
    if den >= -0.2, h = nan; return; end
    num = -pS(3);
    h = num/den;
    H(3) = -1/den;
    Ar = -R*skew3(cfg.r_BR);
    Ad = -R*skew3(cfg.d_BR);
    H(7:9) = (-Ar(3,:)*den - num*Ad(3,:))/den^2;
end

%% =========================================================================
% ICP FUNCTIONS
%% =========================================================================
function [pose, rmse, overlap] = icpToMap_S2(scan, mappts, pose, cfg)
    rmse = inf; overlap = 0;
    for it = 1:cfg.icpIterations
        Q = transformPoints_S2(scan, pose);
        [idx, d] = knnsearch(mappts, Q);
        ids = find(d < cfg.icpMaxCorr); overlap = numel(ids);
        if overlap < 40, break; end
        [~, ord] = sort(d(ids));
        keep = max(40, floor(cfg.icpTrim*numel(ids)));
        ids = ids(ord(1:keep));
        delta = rigidFit_S2(Q(ids,:), mappts(idx(ids),:));
        delta(1:2) = max(-cfg.icpStepXY, min(cfg.icpStepXY, delta(1:2)));
        delta(3) = max(-cfg.icpStepYaw, min(cfg.icpStepYaw, delta(3)));
        pose = se2Compose_S2(delta, pose);
        rmse = sqrt(mean(d(ids).^2));
        if norm(delta(1:2)) < 1e-4 && abs(delta(3)) < 1e-4, break; end
    end
end

function delta = rigidFit_S2(src, dst)
    cs = mean(src,1); cd = mean(dst,1);
    X = src - cs; Y = dst - cd;
    [U,~,V] = svd(X'*Y);
    R = V*U';
    if det(R) < 0, V(:,end) = -V(:,end); R = V*U'; end
    t = cd' - R*cs';
    delta = [t(1), t(2), atan2(R(2,1), R(1,1))];
end

function P = voxelDown_S2(P, v)
    if isempty(P), return; end
    K = floor(P/v);
    [~, ia] = unique(K, 'rows', 'stable');
    P = P(ia,:);
end

function P = transformPoints_S2(P, pose)
    c = cos(pose(3)); s = sin(pose(3));
    P = P*[c, s; -s, c] + pose(1:2);
end

function c = se2Compose_S2(a, b)
    ca = cos(a(3)); sa = sin(a(3));
    c = [a(1) + ca*b(1) - sa*b(2), a(2) + sa*b(1) + ca*b(2), wrapPi(a(3)+b(3))];
end

function a = se2Inverse_S2(p)
    c = cos(p(3)); s = sin(p(3));
    a = [-c*p(1)-s*p(2), s*p(1)-c*p(2), wrapPi(-p(3))];
end

function d = se2Between_S2(a, b)
    d = se2Compose_S2(se2Inverse_S2(a), b);
end

function a = wrapPi(a), a = mod(a+pi, 2*pi)-pi; end

%% =========================================================================
% SCANCONTEXT + GLOBAL POSE GRAPH (S2)
%% =========================================================================
function [globalPose, loops, kfIdx] = buildGlobalPoseGraph_S2(scans, localPose, valid, cfg)
    accepted = find(valid);
    globalPose = localPose;
    loops = zeros(0,4);
    if ~cfg.enableScanContext || numel(accepted) < 3
        kfIdx = accepted; return;
    end

    kfIdx = accepted(1);
    for i = 2:numel(accepted)
        if accepted(i) - kfIdx(end) >= cfg.keyframeStride
            kfIdx(end+1,1) = accepted(i); %#ok<AGROW>
        end
    end
    if kfIdx(end) ~= accepted(end), kfIdx(end+1,1) = accepted(end); end %#ok<AGROW>

    K = numel(kfIdx);
    desc = cell(K,1); keys = zeros(K, cfg.scRings);
    for a = 1:K
        [desc{a}, keys(a,:)] = scanContextDescriptor_S2(scans{kfIdx(a)}, cfg);
    end

    edges = struct('i',{}, 'j',{}, 'z',{}, 'sig',{});
    for a = 2:K
        edges(end+1) = makeEdge_S2(a-1, a, se2Between_S2(localPose(kfIdx(a-1),:), localPose(kfIdx(a),:)), ...
            [0.055, 0.055, deg2rad(1.5)]); %#ok<AGROW>
    end

    exclude = max(6, ceil(cfg.scExcludeRecent/cfg.keyframeStride));
    last = -1e9;
    for a = exclude+1:K
        if size(loops,1) >= cfg.maxLoopClosures || a - last < 6, continue; end
        old = 1:a-exclude;
        [~, ord] = sort(vecnorm(keys(old,:) - keys(a,:), 2, 2));
        short = old(ord(1:min(6, numel(ord))));
        cand = [];
        for b = short
            [d, shift] = scanContextDistance_S2(desc{a}, desc{b});
            cand = [cand; d, b, shift]; %#ok<AGROW>
        end
        cand = sortrows(cand, 1);
        for c = 1:min(4, size(cand,1))
            d = cand(c,1); b = cand(c,2);
            if d > cfg.scThreshold, break; end
            if norm(localPose(kfIdx(a),1:2) - localPose(kfIdx(b),1:2)) > 1.25, continue; end
            init = se2Between_S2(localPose(kfIdx(b),:), localPose(kfIdx(a),:));
            [z, rmse, overlap] = icpPair_S2(scans{kfIdx(a)}, scans{kfIdx(b)}, init, cfg);
            consistency = se2Between_S2(init, z);
            if rmse < cfg.scVerifyRMSE && overlap > 110 && norm(consistency(1:2)) < 0.08 && abs(consistency(3)) < deg2rad(3)
                edges(end+1) = makeEdge_S2(b, a, z, [0.035, 0.035, deg2rad(1)]); %#ok<AGROW>
                loops(end+1,:) = [kfIdx(a), kfIdx(b), d, rmse]; %#ok<AGROW>
                last = a; break;
            end
        end
    end

    kfLocal = localPose(kfIdx,:);
    if isempty(loops), kfGlobal = kfLocal;
    else, kfGlobal = optimizeGraph_S2(kfLocal, edges); end
    globalPose = interpolateGlobal_S2(localPose, kfIdx, kfGlobal);
end

function e = makeEdge_S2(i, j, z, sig)
    e = struct('i', i, 'j', j, 'z', z, 'sig', sig);
end

function [desc, key] = scanContextDescriptor_S2(scan, cfg)
    r = vecnorm(scan, 2, 2);
    th = mod(atan2(scan(:,2), scan(:,1)), 2*pi);
    desc = zeros(cfg.scRings, cfg.scSectors);
    ri = min(cfg.scRings, max(1, floor(r/8*cfg.scRings)+1));
    si = min(cfg.scSectors, max(1, floor(th/(2*pi)*cfg.scSectors)+1));
    for n = 1:numel(r)
        desc(ri(n), si(n)) = max(desc(ri(n), si(n)), 1 - r(n)/8);
    end
    key = mean(desc, 2)';
end

function [best, bshift] = scanContextDistance_S2(a, b)
    best = 1; bshift = 0;
    for s = 0:size(a,2)-1
        bs = circshift(b, [0, s]);
        valid = vecnorm(a,2,1) > 1e-8 & vecnorm(bs,2,1) > 1e-8;
        if ~any(valid), continue; end
        dots = sum(a(:,valid) .* bs(:,valid), 1);
        den = vecnorm(a(:,valid),2,1) .* vecnorm(bs(:,valid),2,1);
        d = 1 - mean(dots ./ max(den, 1e-12));
        if d < best, best = d; bshift = s; end
    end
end

function [pose, rmse, overlap] = icpPair_S2(cur, ref, pose, cfg)
    rmse = inf; overlap = 0;
    for it = 1:20
        Q = transformPoints_S2(cur, pose);
        [idx, d] = knnsearch(ref, Q);
        ids = find(d < 0.25); overlap = numel(ids);
        if overlap < 50, break; end
        [~, ord] = sort(d(ids));
        ids = ids(ord(1:max(50, floor(0.75*numel(ids)))));
        delta = rigidFit_S2(Q(ids,:), ref(idx(ids),:));
        delta(1:2) = max(-0.10, min(0.10, delta(1:2)));
        delta(3) = max(-deg2rad(5), min(deg2rad(5), delta(3)));
        pose = se2Compose_S2(delta, pose);
        rmse = sqrt(mean(d(ids).^2));
        if norm(delta(1:2)) < 1e-4 && abs(delta(3)) < 1e-4, break; end
    end
end

function P = optimizeGraph_S2(P0, edges)
    K = size(P0,1);
    x0 = reshape(P0(2:end,:)', [], 1);
    anchor = P0(1,:);
    fun = @(x) graphResidual_S2(x, K, edges, anchor);
    opts = optimoptions('lsqnonlin', 'Display', 'off', 'MaxIterations', 25, ...
                        'MaxFunctionEvaluations', 3000, 'FunctionTolerance', 1e-7, 'StepTolerance', 1e-7);
    try
        x = lsqnonlin(fun, x0, [], [], opts);
        P = P0;
        P(2:end,:) = reshape(x, 3, [])';
        P(:,3) = wrapPi(P(:,3));
    catch ME
        warning('Pose graph optimization skipped: %s', ME.message);
        P = P0;
    end
end

function r = graphResidual_S2(x, K, edges, anchor)
    P = zeros(K,3); P(1,:) = anchor;
    P(2:end,:) = reshape(x, 3, [])';
    r = zeros(3*numel(edges), 1);
    for n = 1:numel(edges)
        pred = se2Between_S2(P(edges(n).i,:), P(edges(n).j,:));
        e = se2Between_S2(edges(n).z, pred);
        e(3) = wrapPi(e(3));
        q = e ./ edges(n).sig;
        a = abs(q); big = a > 1.5;
        q(big) = sign(q(big)) .* sqrt(2*1.5*a(big) - 1.5^2);
        r(3*n-2:3*n) = q';
    end
end

function out = interpolateGlobal_S2(local, kf, kg)
    C = zeros(numel(kf), 3);
    for a = 1:numel(kf)
        C(a,:) = se2Compose_S2(kg(a,:), se2Inverse_S2(local(kf(a),:)));
    end
    out = local;
    for a = 1:numel(kf)-1
        i0 = kf(a); i1 = kf(a+1);
        for i = i0:i1
            u = (i - i0) / max(1, i1 - i0);
            ci = (1-u)*C(a,1:2) + u*C(a+1,1:2);
            cy = wrapPi(C(a,3) + u*wrapPi(C(a+1,3) - C(a,3)));
            out(i,:) = se2Compose_S2([ci, cy], local(i,:));
        end
    end
end

%% =========================================================================
% LANE SELECTOR (NIS health scoring, Falchi et al. 2024)
%% =========================================================================
function [bestLaneId, healthScores, degraded] = laneSelector_S2( ...
    lanes, cfg, tNow, currentLane, lastSwitchTime)

    nLanes = numel(lanes);
    healthScores = inf(nLanes,1); % lower is better
    eligible = false(nLanes,1);

    for i = 1:nLanes
        lane = lanes(i);
        xyCov = trace(lane.P(1:2,1:2));
        velCov = trace(lane.P(4:5,4:5));
        freshXY = (tNow - lane.lastHorizontalAidTime) <= cfg.horizontalAidTimeout;
        finiteState = all(isfinite([lane.p; lane.v; lane.q(:); diag(lane.P)]));
        eligible(i) = freshXY && finiteState && ...
            xyCov <= cfg.maxXYCovariance && ...
            velCov <= cfg.maxVelXYCovariance;

        % Compare each measurement family on its own normalized NIS scale,
        % then average the family scores. This avoids a high-rate sensor or
        % a 1-DOF measurement dominating the lane score.
        typeScores = [];
        typeNames = {'vio','lidar','range','baro','gravity'};
        for tt = 1:numel(typeNames)
            ids = find(strcmp(lane.nisType, typeNames{tt}));
            if isempty(ids), continue; end
            nIds = numel(ids);
            ids = ids(max(1,nIds-cfg.laneNisWindow+1):nIds);
            vals = lane.nisNormalized(ids);
            vals = vals(isfinite(vals));
            if numel(vals) >= cfg.minLaneObservations
                typeScores(end+1) = mean(min(vals,5)); %#ok<AGROW>
            end
        end
        if isempty(typeScores)
            innovationScore = 0.75;
        else
            innovationScore = mean(typeScores);
        end

        covariancePenalty = cfg.laneCovarianceWeight * ...
            (xyCov/max(cfg.maxXYCovariance,eps) + ...
             velCov/max(cfg.maxVelXYCovariance,eps));
        healthScores(i) = innovationScore + covariancePenalty + ...
            cfg.lanePriorityPenalty(min(i,numel(cfg.lanePriorityPenalty)));
    end

    degraded = ~any(eligible);
    if degraded
        % Never switch to the IMU+vertical-only lane merely because its
        % innovations are small. Retain the last horizontal solution.
        bestLaneId = currentLane;
        return;
    end

    currentEligible = currentLane >= 1 && currentLane <= nLanes && eligible(currentLane);

    % Consistency protects against nuisance switching while the active lane
    % is healthy. If the active lane itself is no longer eligible, do not use
    % its diverged state to veto a valid failover candidate.
    if currentEligible
        for i = 1:nLanes
            if ~eligible(i) || i == currentLane, continue; end
            dp = norm(lanes(i).p - lanes(currentLane).p);
            dq = norm(qlog(qmul(qconj(lanes(currentLane).q), lanes(i).q)));
            if dp > cfg.maxLanePositionJump || dq > cfg.maxLaneAttitudeJump
                eligible(i) = false;
            end
        end
    end

    [candidateScore, candidate] = min(healthScores + (~eligible)*1e9);
    if ~eligible(candidate)
        bestLaneId = currentLane;
        degraded = true;
        return;
    end

    if currentEligible
        if (tNow - lastSwitchTime) < cfg.laneMinDwellTime
            bestLaneId = currentLane;
            return;
        end
        if healthScores(currentLane) <= candidateScore + cfg.laneSwitchMargin
            bestLaneId = currentLane;
            return;
        end
    end
    bestLaneId = candidate;
end

%% =========================================================================
% QUATERNION UTILITIES (Forster et al. 2017)
%% =========================================================================
function q = qnormalize(q)
    q = q(:)';
    q = q/norm(q);
    if q(1) < 0, q = -q; end
end

function q = qconj(q)
    q = [q(1), -q(2:4)];
end

function q = qmul(a, b)
    a = a(:)';
    b = b(:)';
    q = [a(1)*b(1) - dot(a(2:4), b(2:4)), ...
         a(1)*b(2:4) + b(1)*a(2:4) + cross(a(2:4), b(2:4))];
    q = qnormalize(q);
end

function q = qexp(v)
    v = v(:)';
    a = norm(v);
    if a < 1e-10
        q = qnormalize([1, 0.5*v]);
    else
        q = [cos(a/2), sin(a/2)*v/a];
    end
end

function v = qlog(q)
    q = qnormalize(q);
    nv = norm(q(2:4));
    if nv < 1e-10
        v = 2*q(2:4);
    else
        a = 2*atan2(nv, q(1));
        if a > pi, a = a - 2*pi; end
        v = a*q(2:4)/nv;
    end
end

function R = q2R(q)
    q = qnormalize(q);
    w = q(1); x = q(2); y = q(3); z = q(4);
    R = [1-2*(y^2+z^2), 2*(x*y-z*w), 2*(x*z+y*w);
         2*(x*y+z*w), 1-2*(x^2+z^2), 2*(y*z-x*w);
         2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x^2+y^2)];
end

function q = rpy2q(r, p, y)
    cr = cos(r/2); sr = sin(r/2);
    cp = cos(p/2); sp = sin(p/2);
    cy = cos(y/2); sy = sin(y/2);
    q = qnormalize([cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]);
end

function q = R2q(R)
    tr = trace(R);
    if tr > 0
        S = sqrt(tr+1)*2;
        q = [0.25*S, (R(3,2)-R(2,3))/S, (R(1,3)-R(3,1))/S, (R(2,1)-R(1,2))/S];
    else
        [~, i] = max(diag(R));
        if i == 1
            S = sqrt(1+R(1,1)-R(2,2)-R(3,3))*2;
            q = [(R(3,2)-R(2,3))/S, 0.25*S, (R(1,2)+R(2,1))/S, (R(1,3)+R(3,1))/S];
        elseif i == 2
            S = sqrt(1+R(2,2)-R(1,1)-R(3,3))*2;
            q = [(R(1,3)-R(3,1))/S, (R(1,2)+R(2,1))/S, 0.25*S, (R(2,3)+R(3,2))/S];
        else
            S = sqrt(1+R(3,3)-R(1,1)-R(2,2))*2;
            q = [(R(2,1)-R(1,2))/S, (R(1,3)+R(3,1))/S, (R(2,3)+R(3,2))/S, 0.25*S];
        end
    end
    q = qnormalize(q);
end

function rpy = q2rpy(q)
    R = q2R(q);
    p = asin(max(-1, min(1, -R(3,1))));
    r = atan2(R(3,2), R(3,3));
    y = atan2(R(2,1), R(1,1));
    rpy = [r, p, y];
end

function S = skew3(v)
    v = v(:);
    S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end

%% =========================================================================
% TRIAL RUNNER
%% =========================================================================
function out = runTrial_S2(seed, scenario, cfg, makePlots)
    rng(seed, 'twister');
    label = scenario.name;
    fprintf('--- %s TRIAL (seed %d) ---\n', label, seed);

    [gt, meas, failureLog] = simulateSensors_S2(seed, cfg, scenario);

    for w = 1:size(scenario.vioOutageWindows,1)
        tw = scenario.vioOutageWindows(w,:);
        meas.vioValid = meas.vioValid & ...
            ~(meas.tVio >= tw(1) & meas.tVio <= tw(2));
    end
    for w = 1:size(scenario.rangeOutageWindows,1)
        tw = scenario.rangeOutageWindows(w,:);
        meas.rangeValid = meas.rangeValid & ...
            ~(meas.tRange >= tw(1) & meas.tRange <= tw(2));
    end

    % Deterministic, scenario-specific LiDAR dropout and false-scan injection.
    rng(seed + 100003, 'twister');
    meas.lidarValid = rand(numel(meas.tLidar),1) >= scenario.lidarDropProbability;
    meas.lidarValid(1) = true;
    meas.lidarFalse = rand(numel(meas.tLidar),1) < scenario.lidarFalsePoseProbability;
    meas.lidarFalse(1) = false;
    for i = find(meas.lidarFalse(:))'
        falsePose = [scenario.lidarFalseTranslationSigma*randn(1,2), ...
                     scenario.lidarFalseYawSigma*randn];
        meas.scans{i} = transformPoints_S2(meas.scans{i}, falsePose);
    end

    [est, localLidar] = runMultiLaneESKF(gt, meas, cfg, failureLog);
    [globalLidar, loops, keyframes] = buildGlobalPoseGraph_S2( ...
        meas.scans, localLidar.pose, localLidar.valid, cfg);

    L = numel(meas.tLidar); lidarTruth = zeros(L,3);
    for i = 1:L
        k = meas.iLidar(i); R = q2R(gt.q(k,:));
        pL = gt.p(k,:)' + R*cfg.r_BL; RWL = R*cfg.R_BL;
        lidarTruth(i,:) = [pL(1), pL(2), atan2(RWL(2,1), RWL(1,1))];
    end
    localErr = vecnorm(localLidar.pose(:,1:2) - lidarTruth(:,1:2), 2, 2);
    globalErr = vecnorm(globalLidar(:,1:2) - lidarTruth(:,1:2), 2, 2);
    localErr(~localLidar.valid) = nan;
    globalErr(~localLidar.valid) = nan;

    ss = gt.t >= 5;
    summary = struct();
    summary.seed = seed;
    summary.scenario = label;
    summary.stress = ~strcmpi(label,'NOMINAL');
    summary.fusedMaxAfter5_m = max(est.posErr(ss));
    summary.fusedRMSEAfter5_m = sqrt(mean(est.posErr(ss).^2));
    summary.fusedFinal_m = est.posErr(end);
    summary.attMaxAfter5_deg = rad2deg(max(est.attErr(ss)));
    summary.lidarLocalMax_m = max(localErr,[],'omitnan');
    summary.lidarLocalRMSE_m = sqrt(mean(localErr.^2,'omitnan'));
    summary.lidarGlobalMax_m = max(globalErr,[],'omitnan');
    summary.lidarHealthyFraction = mean(localLidar.valid);
    summary.verifiedLoops = size(loops,1);
    summary.localReturnError_m = norm(se2Between_S2(localLidar.pose(1,:),localLidar.pose(end,:)));
    summary.globalReturnError_m = norm(se2Between_S2(globalLidar(1,:),globalLidar(end,:)));
    summary.counts = est.counts;
    summary.unhealthySamples = est.unhealthySamples;
    summary.degradedDuration_s = nnz(est.degraded)/cfg.imuRate;
    summary.finalLane = est.laneHist(end);
    summary.rtlActive = est.rtlActive(end);
    summary.navigationPass = summary.fusedMaxAfter5_m < cfg.requirement_m && ...
        summary.attMaxAfter5_deg < cfg.attRequirement_deg;
    summary.lidarPass = summary.lidarHealthyFraction > ...
        max(0.50, 0.80 - scenario.lidarDropProbability);
    summary.pass = summary.navigationPass && summary.lidarPass;

    fprintf('  Fused max after 5 s : %6.2f cm\n',100*summary.fusedMaxAfter5_m);
    fprintf('  Fused RMSE after 5 s: %6.2f cm\n',100*summary.fusedRMSEAfter5_m);
    fprintf('  Attitude max        : %6.3f deg\n',summary.attMaxAfter5_deg);
    fprintf('  Local LiDAR max     : %6.2f cm\n',100*summary.lidarLocalMax_m);
    fprintf('  LiDAR healthy       : %6.2f %%\n',100*summary.lidarHealthyFraction);
    fprintf('  Lane switches       : %d\n',summary.counts.laneSwitches);
    fprintf('  Degraded duration   : %.2f s\n',summary.degradedDuration_s);
    fprintf('  RESULT              : %s\n\n', ternary_S2(summary.pass,'PASS','FAIL'));

    if makePlots
        plotTrial_S2(gt, est, lidarTruth, localLidar, globalLidar, ...
            summary, label, keyframes, cfg, failureLog);
    end

    out.summary = summary;
    out.est = est;
    out.localLidar = localLidar;
    out.globalLidar = globalLidar;
    out.loops = loops;
    out.keyframes = keyframes;
    out.failureLog = failureLog;
    out.scenario = scenario;
    out.config = cfg;
end

function out = ternary_S2(condition, trueText, falseText)
    if condition, out = trueText; else, out = falseText; end
end

%% =========================================================================
% PRINT RESULT
%% =========================================================================
function printResult_S2(result, cfg)
    s = result.summary;
    fprintf('\n============================================================\n');
    fprintf('  S2 UNIFIED MULTI-LANE ESKF RESULTS (seed %d)\n', s.seed);
    fprintf('============================================================\n');
    fprintf('  Fused max after 5 s : %7.3f m (%6.2f cm)\n', s.fusedMaxAfter5_m, 100*s.fusedMaxAfter5_m);
    fprintf('  Fused RMSE after 5 s: %7.3f m (%6.2f cm)\n', s.fusedRMSEAfter5_m, 100*s.fusedRMSEAfter5_m);
    fprintf('  Fused final error   : %7.3f m (%6.2f cm)\n', s.fusedFinal_m, 100*s.fusedFinal_m);
    fprintf('  Attitude max        : %7.3f deg\n', s.attMaxAfter5_deg);
    fprintf('  Local LiDAR max     : %7.3f m (%6.2f cm)\n', s.lidarLocalMax_m, 100*s.lidarLocalMax_m);
    fprintf('  LiDAR healthy frac  : %7.2f %%\n', 100*s.lidarHealthyFraction);
    fprintf('  Verified loops      : %d\n', s.verifiedLoops);
    fprintf('  Final active lane   : %d (%s)\n', s.finalLane, laneName_S2(s.finalLane));
    fprintf('  RTL active          : %s\n', mat2str(s.rtlActive));
    fprintf('  VIO acc/rej         : %d / %d\n', s.counts.vioAcc, s.counts.vioRej);
    fprintf('  LiDAR acc/rej       : %d / %d\n', s.counts.lidarAcc, s.counts.lidarRej);
    fprintf('  Range acc/rej       : %d / %d\n', s.counts.rangeAcc, s.counts.rangeRej);
    fprintf('  Baro acc/rej        : %d / %d\n', s.counts.baroAcc, s.counts.baroRej);
    fprintf('  Unhealthy samples   : %d\n', s.unhealthySamples);
    fprintf('  Lane switches       : %d\n', s.counts.laneSwitches);
    fprintf('  Degraded duration   : %.2f s\n', s.degradedDuration_s);
    if s.pass
        fprintf('  RESULT: *** PASS ***\n');
    else
        fprintf('  RESULT: *** FAIL ***\n');
    end
    fprintf('============================================================\n');
end

function name = laneName_S2(id)
    names = {'Lane 0: Primary All', 'Lane 1: Backup All', 'Lane 2: Cam-VO Only', 'Lane 3: IMU+Baro Only'};
    if id >= 1 && id <= 4, name = names{id}; else, name = 'Unknown'; end
end

%% =========================================================================
% SUMMARY FUNCTIONS
%% =========================================================================
function row = emptySummaryRow_S2()
    row = struct('seed',0, 'fusedMaxAfter5_m',0, 'fusedRMSEAfter5_m',0, ...
                 'fusedFinal_m',0, 'attMaxAfter5_deg',0, 'finalLane',0, ...
                 'rtlActive',0, 'unhealthySamples',0, 'pass',false);
end

function row = summaryRow_S2(trial)
    row = emptySummaryRow_S2();
    s = trial.summary;
    row.seed = s.seed;
    row.fusedMaxAfter5_m = s.fusedMaxAfter5_m;
    row.fusedRMSEAfter5_m = s.fusedRMSEAfter5_m;
    row.fusedFinal_m = s.fusedFinal_m;
    row.attMaxAfter5_deg = s.attMaxAfter5_deg;
    row.finalLane = s.finalLane;
    row.rtlActive = s.rtlActive;
    row.unhealthySamples = s.unhealthySamples;
    row.pass = s.pass;
end

function summary = summarizeRows_S2(rows, cfg)
    summary.fusedMax5_worst = max([rows.fusedMaxAfter5_m]);
    summary.fusedRMSE5_worst = max([rows.fusedRMSEAfter5_m]);
    summary.fusedFinal_worst = max([rows.fusedFinal_m]);
    summary.attMax_worst = max([rows.attMaxAfter5_deg]);
    summary.pass_count = sum([rows.pass]);
end

%% =========================================================================
% STRESS REGRESSION
%% =========================================================================
function regression = runStressRegression_S2(cfg, baseSeed)
    fprintf('\n=== STRESS REGRESSION ===\n');
    cases = repmat(defaultScenario_S2('nominal'),4,1);

    cases(1).name = 'LIDAR_20PCT_DROPOUT';
    cases(1).lidarDropProbability = 0.20;

    cases(2).name = 'LIDAR_2PCT_FALSE_SCAN';
    cases(2).lidarFalsePoseProbability = 0.02;

    cases(3).name = 'LIDAR_DROPOUT_AND_FALSE';
    cases(3).lidarDropProbability = 0.20;
    cases(3).lidarFalsePoseProbability = 0.02;

    cases(4).name = 'VIO_AND_RANGE_OUTAGE';
    cases(4).vioOutageWindows = [25 32];
    cases(4).rangeOutageWindows = [18 38];

    regression = struct('scenario',{},'summaries',{},'passCount',{},'worstMax_m',{});
    for ci = 1:numel(cases)
        fprintf('\n%s\n', cases(ci).name);
        summaries = repmat(emptySummaryRow_S2(),3,1);
        for jj = 1:3
            seed = baseSeed + 200 + jj - 1;
            trial = runTrial_S2(seed, cases(ci), cfg, false);
            summaries(jj) = summaryRow_S2(trial);
            fprintf('  seed=%d | max5=%.2f cm | lane=%d | switches=%d | pass=%d\n', ...
                seed,100*trial.summary.fusedMaxAfter5_m,trial.summary.finalLane, ...
                trial.summary.counts.laneSwitches,trial.summary.pass);
        end
        regression(ci).scenario = cases(ci).name;
        regression(ci).summaries = summaries;
        regression(ci).passCount = sum([summaries.pass]);
        regression(ci).worstMax_m = max([summaries.fusedMaxAfter5_m]);
        fprintf('  Worst = %.2f cm | passed = %d/3\n', ...
            100*regression(ci).worstMax_m,regression(ci).passCount);
    end
end

%% =========================================================================
% TABBED DEBUG DASHBOARD (ONE FIGURE)
%% =========================================================================
function plotTrial_S2(gt, est, lidarTruth, localLidar, globalLidar, ...
    summary, label, keyframes, cfg, failureLog)

    t = gt.t; N = numel(t);
    eulerEst = zeros(N,3); eulerTrue = zeros(N,3);
    for k = 1:N
        eulerEst(k,:) = q2rpy(est.q(k,:));
        eulerTrue(k,:) = q2rpy(gt.q(k,:));
    end
    attErrDeg = rad2deg(wrapPi(eulerEst - eulerTrue));

    fig = figure('Name',['Stage S2 Dashboard - ' label], ...
        'Color','w','Position',[50 50 1450 850]);
    tabs = uitabgroup(fig);

    tab = uitab(tabs,'Title','Trajectory');
    tl = tiledlayout(tab,1,2,'Padding','compact','TileSpacing','compact');
    ax = nexttile(tl); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    plot3(ax,gt.p(:,1),gt.p(:,2),gt.p(:,3),'k-','LineWidth',1.8,'DisplayName','Truth');
    plot3(ax,est.p(:,1),est.p(:,2),est.p(:,3),'-','LineWidth',1.3,'DisplayName','ESKF');
    plot3(ax,lidarTruth(:,1),lidarTruth(:,2),gt.p(measIndexForPlot_S2(size(lidarTruth,1),N),3),'.','DisplayName','LiDAR truth');
    xlabel(ax,'x [m]'); ylabel(ax,'y [m]'); zlabel(ax,'z [m]');
    title(ax,sprintf('%s 6-DOF trajectory',label)); axis(ax,'equal'); view(ax,45,25); legend(ax,'Location','best');

    ax = nexttile(tl); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    rectangle(ax,'Position',[0 0 cfg.room(1) cfg.room(2)],'EdgeColor','k','LineWidth',1.5);
    for ii=1:size(cfg.obstacles,1)
        rectangle(ax,'Position',cfg.obstacles(ii,:),'FaceColor',[0.85 0.85 0.85],'EdgeColor','k');
    end
    plot(ax,gt.p(:,1),gt.p(:,2),'k-','LineWidth',1.5,'DisplayName','Truth');
    plot(ax,est.p(:,1),est.p(:,2),'-','LineWidth',1.2,'DisplayName','ESKF');
    plot(ax,globalLidar(:,1),globalLidar(:,2),':','LineWidth',1.2,'DisplayName','Global map');
    xlabel(ax,'x [m]'); ylabel(ax,'y [m]'); title(ax,'Top view'); legend(ax,'Location','best');

    tab = uitab(tabs,'Title','Errors');
    tl = tiledlayout(tab,2,2,'Padding','compact','TileSpacing','compact');
    ax=nexttile(tl); plot(ax,t,100*est.posErr,'LineWidth',1.2); grid(ax,'on'); hold(ax,'on');
    yline(ax,100*cfg.requirement_m,'--r','10 cm'); xlabel(ax,'Time [s]'); ylabel(ax,'3-D error [cm]');
    title(ax,sprintf('Max after 5 s = %.2f cm',100*summary.fusedMaxAfter5_m));
    labels={'Roll','Pitch','Yaw'};
    for j=1:3
        ax=nexttile(tl); plot(ax,t,attErrDeg(:,j),'LineWidth',1.1); grid(ax,'on'); hold(ax,'on');
        yline(ax,cfg.attRequirement_deg,'--r'); yline(ax,-cfg.attRequirement_deg,'--r');
        xlabel(ax,'Time [s]'); ylabel(ax,'Error [deg]'); title(ax,[labels{j} ' error']);
    end

    tab = uitab(tabs,'Title','Bias & covariance');
    tl = tiledlayout(tab,2,2,'Padding','compact','TileSpacing','compact');
    ax=nexttile(tl); plot(ax,t,est.ba,'LineWidth',1.0); grid(ax,'on'); xlabel(ax,'Time [s]'); ylabel(ax,'m/s^2'); title(ax,'Accelerometer bias');
    ax=nexttile(tl); plot(ax,t,rad2deg(est.bg),'LineWidth',1.0); grid(ax,'on'); xlabel(ax,'Time [s]'); ylabel(ax,'deg/s'); title(ax,'Gyro bias');
    ax=nexttile(tl); plot(ax,t,est.bbaro,'LineWidth',1.0); grid(ax,'on'); xlabel(ax,'Time [s]'); ylabel(ax,'m'); title(ax,'Barometer bias');
    ax=nexttile(tl); semilogy(ax,t,max(est.xyCovariance,eps),'LineWidth',1.0); grid(ax,'on'); xlabel(ax,'Time [s]'); ylabel(ax,'trace P_{xy}'); title(ax,'Lane XY covariance');

    tab = uitab(tabs,'Title','Lane health');
    tl = tiledlayout(tab,2,2,'Padding','compact','TileSpacing','compact');
    ax=nexttile(tl); stairs(ax,t,est.laneHist,'LineWidth',1.2); grid(ax,'on'); ylim(ax,[0.5 4.5]); yticks(ax,1:4); xlabel(ax,'Time [s]'); ylabel(ax,'Lane'); title(ax,sprintf('Switches: %d',summary.counts.laneSwitches));
    ax=nexttile(tl); plot(ax,t,est.laneScores,'LineWidth',1.0); grid(ax,'on'); xlabel(ax,'Time [s]'); ylabel(ax,'Score (lower better)'); title(ax,'Lane health scores');
    ax=nexttile(tl); area(ax,t,double(est.degraded)); grid(ax,'on'); ylim(ax,[-0.1 1.1]); xlabel(ax,'Time [s]'); ylabel(ax,'Degraded'); title(ax,sprintf('Degraded %.2f s',summary.degradedDuration_s));
    ax=nexttile(tl); area(ax,t,double(est.rtlActive)); grid(ax,'on'); ylim(ax,[-0.1 1.1]); xlabel(ax,'Time [s]'); ylabel(ax,'RTL'); title(ax,'RTL state');

    tab = uitab(tabs,'Title','LiDAR / pose graph');
    tl = tiledlayout(tab,2,2,'Padding','compact','TileSpacing','compact');
    ax=nexttile(tl); plot(ax,localLidar.diag(:,1),'LineWidth',1.0); grid(ax,'on'); ylabel(ax,'RMSE [m]'); xlabel(ax,'Scan'); title(ax,'ICP RMSE');
    ax=nexttile(tl); plot(ax,localLidar.diag(:,2),'LineWidth',1.0); grid(ax,'on'); ylabel(ax,'Points'); xlabel(ax,'Scan'); title(ax,'ICP overlap');
    ax=nexttile(tl); stairs(ax,double(localLidar.valid),'LineWidth',1.0); grid(ax,'on'); ylim(ax,[-0.1 1.1]); xlabel(ax,'Scan'); title(ax,sprintf('Accepted %.1f%%',100*mean(localLidar.valid)));
    ax=nexttile(tl); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    plot(ax,localLidar.pose(:,1),localLidar.pose(:,2),'-','DisplayName','Local');
    plot(ax,globalLidar(:,1),globalLidar(:,2),':','DisplayName','Global');
    if ~isempty(keyframes), scatter(ax,globalLidar(keyframes,1),globalLidar(keyframes,2),12,'filled','DisplayName','Keyframes'); end
    xlabel(ax,'x [m]'); ylabel(ax,'y [m]'); title(ax,sprintf('Loops: %d',summary.verifiedLoops)); legend(ax,'Location','best');

    fig.Name = sprintf(['Stage S2 Dashboard - %s | seed %d | max %.2f cm | ' ...
        'lane %d | LiDAR fail %s | camera fail %s'], ...
        label,summary.seed,100*summary.fusedMaxAfter5_m,summary.finalLane, ...
        timeText_S2(failureLog.lidarFailTime),timeText_S2(failureLog.camFailTime));
end

function idx = measIndexForPlot_S2(M,N)
    idx = round(linspace(1,N,M));
end

function s = timeText_S2(t)
    if isfinite(t), s = sprintf('%.1f s',t); else, s = 'none'; end
end

end

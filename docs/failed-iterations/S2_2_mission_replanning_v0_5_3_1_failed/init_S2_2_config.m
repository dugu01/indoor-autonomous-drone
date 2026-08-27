function cfg = init_S2_2_config()
% INIT_S2_2_CONFIG Stage S2.2 v0.5.3 autonomous mission lifecycle integration.

cfg.stage='S2.2';
cfg.version='v0.5.3.1';
cfg.methodName='OMR-IDS-MS-6DOF-LIFECYCLE-ROBUST';

cfg.room=[6.0 6.0 2.5];
cfg.gridResolution=0.10;
cfg.dt=0.02;
cfg.maxTime_s=100.0;
cfg.gW=[0;0;-9.81];

% F450 simulation parameters. These are nominal simulation values and must
% be calibrated against the final airframe before hardware-in-the-loop use.
cfg.mass_kg=1.50;
cfg.inertia_kgm2=diag([0.030 0.030 0.055]);
cfg.linearDrag=0.10;
cfg.angularDrag=0.02;
cfg.thrustToWeight=2.40;
cfg.maxMoment_Nm=[0.90;0.90;0.35];
cfg.motorArmRadius=0.225;
cfg.propRadius=0.127;
cfg.collisionRadius=cfg.motorArmRadius+cfg.propRadius;
cfg.localizationMargin=0.10;
cfg.controlMargin=0.05;
cfg.baseInflationRadius=cfg.collisionRadius+cfg.localizationMargin+cfg.controlMargin;
cfg.inflationRadius=cfg.baseInflationRadius;
cfg.uncertaintySigmaGain=2.0;
cfg.maxInflationRadius=0.72;
cfg.inflationReplanThreshold_m=0.035;

cfg.altitudeNominal_m=1.15;
cfg.goalTolerance_m=0.18;
cfg.altitudeTolerance_m=0.12;
cfg.maxSpeedXY_mps=0.32;
cfg.maxAccelXY_mps2=0.65;
cfg.maxDecelXY_mps2=0.80;
cfg.maxJerkXY_mps3=2.20;

% Seventh-order polynomial trajectory generation inherited from v0.3.
cfg.trajectorySampleDt_s=0.02;
cfg.trajectoryNominalSpeedFraction=0.68;
cfg.trajectoryInternalSpeedFraction=0.58;
cfg.trajectoryMinSegmentTime_s=0.90;
cfg.trajectoryMaxScaleIterations=12;
cfg.trajectoryScaleSafety=1.03;
cfg.trajectoryClockMaxError_m=0.55;
cfg.trajectoryKp=1.60;
cfg.trajectoryKd=1.10;
cfg.maxTrajectoryTrackingError_m=0.80;
cfg.maxReferenceContinuityJump=[1e-7 1e-6 1e-5 1e-4];
cfg.maxReplanStateJump=[1e-7 1e-5 1e-4];

% Near-hover geometric position/attitude controller.
cfg.positionKp=[2.0;2.0;3.2];
cfg.velocityKd=[1.5;1.5;2.0];
cfg.attitudeKp=[5.0;5.0;2.2];
cfg.rateKd=[0.55;0.55;0.25];
cfg.maxTilt_rad=deg2rad(24);
cfg.maxCommandAccel_mps2=3.0;
cfg.maxHorizontalCommandAccel_mps2=0.65;
cfg.maxVerticalCommandAccel_mps2=1.50;
cfg.maxControllerJerk_mps3=6.0;
cfg.maxPositionTrackingError_m=0.45;
cfg.maxAltitudeError_m=0.18;
cfg.maxAttitudeError_deg=5.0;
cfg.maxEstimatorPositionError_m=0.10;
cfg.maxEstimatorFailsafeBound_m=0.25;
cfg.maxEstimatorAttitudeError_deg=2.0;
cfg.maxExecutedSpeed_mps=0.45;
cfg.maxExecutedAccel_mps2=2.5;
cfg.maxExecutedJerk_mps3=25.0;
% Soft executed-speed envelope. The controller starts removing tangential
% acceleration before the hard validation limit is reached. This prevents a
% large REJOIN position error from accelerating the 6-DOF vehicle beyond the
% allowed executed speed even though the reference velocity is bounded.
cfg.executedSpeedSoftLimit_mps=0.34;
cfg.speedGuardGain=2.5;
cfg.speedGuardBrake_mps2=0.30;

% Dynamic-obstacle safety inherited from v0.3.
cfg.predictionHorizon_s=3.0;
cfg.sensorControlDelay_s=0.30;
cfg.dynamicBuffer_m=0.12;
cfg.holdClearTime_s=0.40;
cfg.safetyVelocityAccelLimit_mps2=0.55;
cfg.safetyVelocityJerkLimit_mps3=2.0;
cfg.rejoinTolerance_m=0.30;
cfg.dynamicPositionNoise_m=0.020;
cfg.trackerAlpha=0.72;
cfg.trackerBeta=0.05;
cfg.trackerVelocityFilterAlpha=0.10;
cfg.stoppedSpeedThreshold_mps=0.10;
cfg.stoppedPersistence_s=1.50;
cfg.obstacleNoDataStopTimeout_s=0.50;
cfg.obstacleNoDataFailsafeTimeout_s=5.0;
cfg.stallRecoveryTime_s=1.0;
% Multi-seed liveness watchdog. A prolonged REJOIN that does not reduce
% distance to the goal is converted into a brake-and-replan recovery.
cfg.rejoinProgressTimeout_s=2.0;
cfg.rejoinProgressMinimum_m=0.04;
cfg.rejoinHardTimeout_s=6.0;
cfg.maxProgressRecoveries=3;
% When a newly promoted/static obstacle invalidates a trajectory while the
% vehicle is moving, brake first and regenerate from a near-hover state.
cfg.replanBrakeSpeed_mps=0.05;
cfg.replanBrakeAccel_mps2=0.12;
cfg.replanBrakeTimeout_s=4.0;
cfg.replanRetryPeriod_s=0.25;
cfg.rejoinVelocityTolerance_mps=0.12;
cfg.estAccelerationFilterAlpha=0.08;
cfg.maxReplanStartAccel_mps2=0.40;
cfg.minDynamicPhysicalClearance_m=0.0;

% Online four-lane S2.1-style ESKF configuration.
cfg.accelND=0.003;
cfg.gyroND=deg2rad(0.025);
cfg.accelBiasRW=2e-4;
cfg.gyroBiasRW=deg2rad(0.002);
cfg.baroBiasRW=0.002;
cfg.vioPosSigma=0.015;
cfg.vioVelSigma=0.025;
cfg.vioAttSigma=deg2rad(0.45);
cfg.lidarSigmaXY=0.025;
cfg.lidarSigmaYaw=deg2rad(0.70);
cfg.rangeSigma=0.012;
cfg.baroSigma=0.060;
cfg.gateVIO9=27.877;
cfg.gateLidar3=16.266;
cfg.gate1=10.828;
cfg.laneNisWindow=10;
cfg.horizontalAidTimeout=1.0;
cfg.verticalAidTimeout=1.0;
cfg.attitudeAidTimeout=1.0;
cfg.maxXYCovariance=0.10^2;
cfg.maxZCovariance=0.12^2;
cfg.laneSwitchMargin=0.20;
cfg.laneConfirmTime=0.15;
cfg.laneFastConfirmTime=0.04;
cfg.laneFastSwitchMargin=0.02;
cfg.laneMinDwellTime=1.0;
cfg.maxLanePositionJump=0.75;
cfg.maxLaneAttitudeJump=deg2rad(12);
cfg.outputBlendTime=0.30;
% Confirmed IMU-fault switches use a shorter causal blend so the faulty lane
% is not retained in the control output for the full nominal transition.
cfg.outputBlendTimeFault_s=0.08;
cfg.imuAttributionWindow=4;
cfg.imuAttributionRecentWeight=0.80;
cfg.degradedRTLDelay=2.0;
cfg.lanePriorityPenalty=[0.00 0.03 0.12 0.12];
cfg.imuDisagreementAccel=0.22;
cfg.imuDisagreementGyro=deg2rad(1.2);
cfg.imuDisagreementSamples=4;
cfg.imuGroupScoreMargin=0.06;

% Sensor simulation rates/noise.
cfg.vioPeriodSteps=max(1,round((1/25)/cfg.dt));
cfg.lidarPeriodSteps=max(1,round((1/5)/cfg.dt));
cfg.rangePeriodSteps=max(1,round((1/25)/cfg.dt));
cfg.baroPeriodSteps=max(1,round((1/10)/cfg.dt));
cfg.imuAccelNoiseSigma=0.010;
cfg.imuGyroNoiseSigma=deg2rad(0.05);
cfg.dynamicSensorNoiseSigma=0.020;

cfg.knownObstacles=[1.0 1.0 0.5 0.5;4.0 3.5 0.5 0.5];

% Autonomous mission lifecycle (v0.5).
cfg.groundHeight_m=0.03;
cfg.preflightCheckTime_s=0.60;
cfg.armDelay_s=0.50;
cfg.takeoffDuration_s=4.50;
cfg.initialHoverTime_s=1.00;
cfg.waitForGoalTime_s=0.60;
cfg.goalHoverTime_s=1.00;
cfg.rtlArrivalHoverTime_s=0.80;
cfg.landingDuration_s=5.50;
cfg.disarmDelay_s=0.60;
cfg.emergencyHoldTime_s=0.60;
% Position-loss response: stop the mission immediately, damp the
% short-term inertial horizontal velocity, then descend when the
% configured no-aid delay requests the failsafe. No stale XY position
% target is used.
cfg.emergencyVelocityDampingEnabled=true;
cfg.takeoffAltitudeTolerance_m=0.08;
cfg.landingAltitudeTolerance_m=0.05;
cfg.landedVerticalSpeed_mps=0.08;
cfg.landContactConfirmTime_s=0.20;
cfg.takeoffConfirmTime_s=0.30;
cfg.goalArrivalConfirmTime_s=0.30;
cfg.rtlArrivalConfirmTime_s=0.30;
cfg.arrivalSpeedTolerance_mps=0.15;
cfg.landingZoneExtraMargin_m=0.08;
cfg.maxVerticalReferenceSpeed_mps=0.60;
cfg.maxVerticalReferenceAccel_mps2=0.65;
cfg.maxVerticalReferenceJerk_mps3=1.50;
cfg.maxExecutedVerticalSpeed_mps=0.75;
cfg.maxExecutedVerticalAccel_mps2=2.50;
cfg.maxExecutedVerticalJerk_mps3=25.0;
cfg.maxLifecycleTime_s=165.0;
cfg.rtlReplanTimeout_s=5.0;
% If a safe grid route exists but repeated smooth-trajectory attempts fail,
% execute a conservative stop-at-corner grid fallback rather than landing at
% the current location.
cfg.gridFallbackRetryLimit=8;
cfg.gridFallbackSpeed_mps=0.18;
cfg.gridFallbackWaypointTolerance_m=0.10;
cfg.gridFallbackFinalSpeedTolerance_mps=0.08;
cfg.gridFallbackAccelLimit_mps2=0.45;
cfg.gridFallbackJerkLimit_mps3=1.50;
% Horizontal-aid loss: brake immediately while inertial velocity remains
% trustworthy, then level the vehicle and complete a shorter vertical land.
cfg.lifecycleXYLossDetectionAge_s=0.20;
cfg.lifecycleXYRecoveryAge_s=0.15;
cfg.lifecycleXYLossEmergencyDelay_s=0.20;
cfg.xyLossVelocityTrustTime_s=0.85;
cfg.xyLossEmergencyHoldTime_s=0.05;
cfg.emergencyLandingDuration_s=4.40;
cfg.emergencyVelocityDampingGain=2.80;
cfg.xyLossOpenLoopBrakeAccel_mps2=0.50;
cfg.xyLossBrakeMinTime_s=0.25;
cfg.xyLossBrakeMaxTime_s=0.90;
cfg.xyLossBrakeRampAllowance_s=0.18;
cfg.xyLossBrakeExitSpeed_mps=0.055;
cfg.preflightMinHorizontalAids=1;
cfg.preflightRequireVerticalAid=true;
cfg.preflightMinAcceptedUpdates=3;
cfg.preflightMaxXYCovariance=cfg.maxXYCovariance;
cfg.preflightMaxZVariance=cfg.maxZCovariance;
cfg.homePosition=[3.0 0.8];
cfg.alternateLandingZones=[5.1 0.9;0.9 5.1;0.9 3.0];
cfg.landingSelectionMaxCandidates=8;
cfg.missionStateTimeout_s=45.0;
cfg.resultsRoot=fullfile(pwd,'results','S2_2_mission_replanning');
end

function cfg = init_S2_2_config()
% INIT_S2_2_CONFIG  Stage S2.2 v0.2 incremental/dynamic planning config.

cfg.stage = 'S2.2';
cfg.version = 'v0.2';
cfg.methodName = 'OMR-IDS Planner';
% OMR-IDS = Obstacle-Mapped Replanning with Incremental/Dynamic Safety.

cfg.room = [6.0 6.0 2.5];
cfg.gridResolution = 0.10;
cfg.dt = 0.10;
cfg.maxTime_s = 100.0;

% F450 safety geometry carried forward from S2.1.
cfg.motorArmRadius = 0.225;
cfg.propRadius = 0.127;
cfg.collisionRadius = cfg.motorArmRadius + cfg.propRadius;
cfg.localizationMargin = 0.10;
cfg.controlMargin = 0.05;
cfg.inflationRadius = cfg.collisionRadius + cfg.localizationMargin + cfg.controlMargin;

cfg.altitudeNominal_m = 1.15;
cfg.goalTolerance_m = 0.15;
cfg.waypointTolerance_m = 0.12;
cfg.maxSpeedXY_mps = 0.35;
cfg.maxAccelXY_mps2 = 0.70;
cfg.maxDecelXY_mps2 = 0.80;

% Predictive dynamic-obstacle safety.
cfg.predictionHorizon_s = 3.0;
cfg.sensorControlDelay_s = 0.30;
cfg.dynamicBuffer_m = 0.12;
cfg.holdClearTime_s = 0.40;
cfg.dynamicPositionNoise_m = 0.020;
cfg.trackerAlpha = 0.72;
cfg.trackerBeta = 0.12;
cfg.stoppedSpeedThreshold_mps = 0.10;
cfg.stoppedPersistence_s = 1.50;

% No-data behaviour aligned with the conservative PX4 concept.
cfg.noDataStopTimeout_s = 0.50;
cfg.noDataFailsafeTimeout_s = 5.0;

% Consistency/fallback logic.
cfg.stallRecoveryTime_s = 0.80;
cfg.maxTrackingErrorForPass_m = 0.80;
cfg.minDynamicPhysicalClearance_m = 0.0;

cfg.knownObstacles = [1.0 1.0 0.5 0.5; 4.0 3.5 0.5 0.5];
cfg.resultsRoot = fullfile(pwd,'results','S2_2_mission_replanning');
end

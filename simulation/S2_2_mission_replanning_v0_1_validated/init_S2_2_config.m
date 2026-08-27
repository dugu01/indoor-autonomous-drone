function cfg = init_S2_2_config()
% INIT_S2_2_CONFIG  Configuration for Stage S2.2 v0.1.

cfg.stage = 'S2.2';
cfg.version = 'v0.1';
cfg.methodName = 'OMR-FailSafe Planner';

cfg.room = [6.0 6.0 2.5];
cfg.gridResolution = 0.05;
cfg.dt = 0.10;
cfg.maxTime_s = 90.0;

% F450 safety geometry carried forward from S2.1.
cfg.motorArmRadius = 0.225;
cfg.propRadius = 0.127;
cfg.collisionRadius = cfg.motorArmRadius + cfg.propRadius;
cfg.localizationMargin = 0.10;
cfg.controlMargin = 0.05;
cfg.inflationRadius = cfg.collisionRadius + cfg.localizationMargin + cfg.controlMargin;

cfg.altitudeNominal_m = 1.15;
cfg.startAltitude_m = 1.15;
cfg.goalTolerance_m = 0.15;
cfg.waypointTolerance_m = 0.12;
cfg.lookaheadDistance_m = 0.75;
cfg.hoverTimeBeforeReplan_s = 0.50;
cfg.obstaclePersistenceHits = 1;

cfg.maxSpeedXY_mps = 0.25;
cfg.maxAccelXY_mps2 = 0.60;
cfg.maxTrackingErrorForPass_m = 0.80;

cfg.knownObstacles = [1.0 1.0 0.5 0.5; 4.0 3.5 0.5 0.5];

cfg.resultsRoot = fullfile(pwd,'results','S2_2_mission_replanning');
end

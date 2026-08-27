function cfg = init_S2_3_config()
% INIT_S2_3_CONFIG Stage S2.3 integrated online-mapping configuration.
% Inherit the validated S2.2 numerical plant, ESKF, controller, planner and
% lifecycle thresholds. New fields configure perception and mapping only.

cfg=init_S2_2_config();
cfg.stage='S2.3';
cfg.version='v1.0.0-candidate';
cfg.methodName='POM-IDS-MS-6DOF-LIFECYCLE';

% Raw obstacle-perception simulation. Estimator LiDAR pose aiding remains a
% separate S2.2 channel and is not used as an obstacle scan.
cfg.perceptionLidarPeriodSteps=max(1,round((1/8)/cfg.dt));
cfg.depthPeriodSteps=max(1,round((1/10)/cfg.dt));
cfg.perceptionLidarRayCount=120;
cfg.perceptionLidarMaxRange_m=6.5;
cfg.perceptionLidarMinRange_m=0.12;
cfg.perceptionLidarRangeSigma_m=0.012;
cfg.depthAzimuthRayCount=17;
cfg.depthElevationRayCount=7;
cfg.depthHFOV_rad=deg2rad(87);
cfg.depthVFOV_rad=deg2rad(58);
cfg.depthMaxRange_m=4.5;
cfg.depthMinRange_m=0.18;
cfg.depthRangeSigma_m=0.018;
cfg.raycastStep_m=0.035;
cfg.r_B_lidar=[0;0;0.03];
cfg.r_B_depth=[0.08;0;0.02];
cfg.R_B_lidar=eye(3);
cfg.R_B_depth=eye(3);

% Layered probabilistic occupancy map. A custom base-MATLAB representation
% avoids coupling the stage to occupancyMap3D toolbox availability.
cfg.mapResolutionXY_m=cfg.gridResolution;
cfg.mapResolutionZ_m=0.20;
cfg.mapMinZ_m=0.0;
cfg.mapMaxZ_m=cfg.room(3);
cfg.mapLogOddsPrior=0.0;
cfg.mapLogOddsHit=0.90;
cfg.mapLogOddsMiss=-0.45;
cfg.mapLogOddsMin=-4.0;
cfg.mapLogOddsMax=4.0;
cfg.mapOccupiedProbability=0.65;
cfg.mapFreeProbability=0.20;
cfg.mapMinFreeObservations=2;
cfg.mapMinOccupiedObservations=2;
cfg.mapDynamicHit=1.20;
cfg.mapDynamicDecayPerSecond=0.75;
cfg.mapDynamicOccupiedProbability=0.60;
cfg.mapDynamicPromotionTime_s=1.50;
cfg.mapDynamicPromotionHits=5;
cfg.mapChangedProbabilityThreshold=0.08;
cfg.mapPoseCovarianceReject_m=0.22;
cfg.mapMaxPacketAge_s=0.30;
cfg.mapPerceptionHoldTimeout_s=0.55;
cfg.mapPreflightMinAcceptedPackets=2;
cfg.mapPerceptionFailsafeTimeout_s=4.0;
cfg.mapInitialScanTime_s=2.40;
cfg.mapScanHoldTime_s=2.60;
% The seed-0 recorded replay localized the 2 deg attitude violation to
% repeated scan states at 55 deg/s. Use a 35 deg/s continuous scan; this
% remains sufficient to sweep approximately 91 deg during a 2.6 s hold.
cfg.mapScanYawRate_radps=deg2rad(35);
cfg.mapMaxExtensionAttempts=12;
cfg.mapMaxNoProgressScans=3;
cfg.mapMinFrontierProgress_m=0.20;
cfg.mapFrontierUnknownNeighborCount=1;
cfg.mapStopExtraMargin_m=0.08;
cfg.mapLandingFreshness_s=180.0;
cfg.mapTakeoffRadius_m=cfg.collisionRadius+cfg.controlMargin;
cfg.mapUnknownIsOccupied=true;
cfg.mapStoreSnapshotPeriod_s=1.0;

% Truth-independent mapping acceptance metrics.
cfg.mapMaxFalseFreeRate=0.005;
cfg.mapMinOccupiedRecall=0.95;
cfg.mapBoundaryP95Limit_m=max(0.10,2*cfg.mapResolutionXY_m);

% Give unknown-environment missions sufficient time for scan/advance cycles.
cfg.maxLifecycleTime_s=220.0;
cfg.missionStateTimeout_s=70.0;
cfg.resultsRoot=fullfile(pwd,'results','S2_3_online_mapping');
end

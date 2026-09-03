function cfg = init_S2_4_E_config()
% INIT_S2_4_E_CONFIG Coupled active-exploration request configuration.
%
% The S2.3 estimator, mapper, planner, trajectory generator, controller and
% plant remain inherited. S2.4 produces a mission-level request only; it
% never produces thrust, attitude, body-rate or velocity commands.

cfg = init_S2_3_config();
cfg.stage = 'S2.4-E';
cfg.version = 'v0.3.6-adversarial-competing-corridors-candidate';
cfg.methodName = 'UNCERTAINTY-FRONTIER-NBV-MISSION-REQUEST';

cfg.activeExploration = init_S2_4_AD_config(cfg);
cfg.activeExploration.shadowOnly = false;
cfg.activeExploration.requestOutputEnabled = true;
cfg.activeExploration.commandOutputEnabled = false;
cfg.activeExploration.requestValidity_s = 1.00;
cfg.activeExploration.requireMapRevalidation = true;
cfg.activeExploration.maxViewpointExecutions = 12;
cfg.activeExploration.scanHoldTime_s = cfg.mapScanHoldTime_s;
cfg.activeExploration.scanYawRate_radps = cfg.mapScanYawRate_radps;
cfg.activeExploration.enableOutbound = true;
cfg.activeExploration.enableRTL = false;

% The first coupled milestone needs additional time for move-scan-replan
% cycles but retains all inherited controller and safety limits.
cfg.maxLifecycleTime_s = max(cfg.maxLifecycleTime_s,260.0);
cfg.missionStateTimeout_s = max(cfg.missionStateTimeout_s,90.0);
end

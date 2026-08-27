function results = run_S2_2_mission_replanning(seed,scenarioName,makePlots,makeAnimation)
% RUN_S2_2_MISSION_REPLANNING Stage S2.2 v0.4 estimator/6-DOF integration.
if nargin<1||isempty(seed),seed=0;end
if nargin<2||isempty(scenarioName),scenarioName='nominal_6dof';end
if nargin<3||isempty(makePlots),makePlots=true;end
if nargin<4||isempty(makeAnimation),makeAnimation=false;end
rng(seed,'twister');scriptDir=fileparts(mfilename('fullpath'));addpath(scriptDir,'-begin');cfg=init_S2_2_config();cfg.seed=seed;
versionFolder=lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_'));cfg.resultsRoot=fullfile(fileparts(scriptDir),'results','S2_2_mission_replanning',versionFolder);
if ~exist(cfg.resultsRoot,'dir'),mkdir(cfg.resultsRoot);end
scenario=scenario_S2_2(scenarioName);label=lower(regexprep(scenario.name,'[^A-Za-z0-9]+','_'));
resultsDir=fullfile(cfg.resultsRoot,label,sprintf('seed_%03d',seed));if ~exist(resultsDir,'dir'),mkdir(resultsDir);end
fprintf('\n============================================================\n');
fprintf(' STAGE S2.2 v0.4 ESTIMATOR-IN-THE-LOOP 6-DOF REPLANNING\n');
fprintf(' seed=%d | scenario=%s | plots=%d | animation=%d\n',seed,scenario.name,makePlots,makeAnimation);
fprintf(' Results: %s\n',resultsDir);fprintf('============================================================\n');
[log,summary,maps]=mission_manager_S2_2(cfg,scenario);summary.seed=seed;summary.scenario=scenario.name;summary.outputDir=resultsDir;
fprintf('Goal / failsafe / RTL  : %d / %d / %d\n',summary.goalReached,summary.failsafeTriggered,summary.rtlRequested);
fprintf('Replans/promotions/infl: %d / %d / %d\n',summary.replanCount,summary.promotionCount,summary.inflationReplanCount);
fprintf('Dynamic avoid/holds    : %d / %d\n',summary.dynamicAvoidSteps,summary.dynamicHoldCount);
fprintf('Obstacle no-data holds : %d\n',summary.obstacleNoDataHoldCount);
fprintf('Replan brake / retries : %d / %d\n',summary.replanBrakeCount,summary.replanRetryCount);
fprintf('Lane final / switches  : %d / %d\n',summary.activeLaneFinal,summary.laneSwitches);
fprintf('D* repair / A* scratch : %d / %d expanded nodes\n',summary.dstarRepairExpanded,summary.astarScratchExpanded);
fprintf('Clearance obs/wall/dyn : %.3f / %.3f / %.3f m\n',summary.minObstacleClearance_m,summary.minWallClearance_m,summary.minDynamicClearance_m);
fprintf('Reference v/a/j        : %.3f / %.3f / %.3f\n',summary.maxReferenceSpeed_mps,summary.maxReferenceAccel_mps2,summary.maxReferenceJerk_mps3);
fprintf('Executed v/a/j         : %.3f / %.3f / %.3f\n',summary.maxExecutedSpeed_mps,summary.maxExecutedAccel_mps2,summary.maxExecutedJerk_mps3);
fprintf('Estimator pos/att max  : %.3f m / %.3f deg\n',summary.maxEstimatorPositionError_m,summary.maxEstimatorAttitudeError_deg);
fprintf('Track/override/alt     : %.3f / %.3f / %.3f m\n',summary.maxTrackingError_m,summary.maxSafetyOverrideDeviation_m,summary.maxAltitudeError_m);
fprintf('Tilt / inflation max   : %.3f deg / %.3f m\n',summary.maxTilt_deg,summary.maxInflationRadius_m);
fprintf('Trajectories/fallbacks : %d / %d | max scale %.2f\n',summary.trajectoryGenerationCount,summary.trajectoryFallbackCount,summary.maxTrajectoryTimeScale);
fprintf('Core S/D/RK/EK/C/E/U   : %d / %d / %d / %d / %d / %d / %d\n', ...
    summary.staticPass,summary.dynamicPass,summary.referenceKinematicPass,summary.executedKinematicPass, ...
    summary.controllerPass,summary.estimatorPositionPass&&summary.estimatorAttitudePass,summary.uncertaintyPass);
fprintf('Mission/failsafe/event : %d / %d / %d\n',summary.missionOutcomePass,summary.failsafeExpectationPass,summary.eventPass);
fprintf('Events R/D/P/N/L/RTL   : %d / %d / %d / %d / %d / %d\n',summary.replanEventPass,summary.dynamicEventPass, ...
    summary.promotionEventPass,summary.noDataEventPass,summary.laneSwitchEventPass,summary.rtlEventPass);
fprintf('Collision / geofence   : %d / %d\n',summary.collisionCount,summary.geofenceViolationCount);
fprintf('RESULT                 : %s\n\n',ternary(summary.pass,'PASS','FAIL'));
plotFiles={};if makePlots,plotFiles=plot_S2_2_dashboard(cfg,scenario,log,summary,maps,resultsDir);end
animationFile='';if makeAnimation,animationFile=animate_S2_2_flight(cfg,scenario,log,summary,maps,resultsDir);end
save(fullfile(resultsDir,'S2_2_v0_4_trial_data.mat'),'cfg','scenario','log','summary','maps','-v7.3');write_summary(summary,resultsDir);
results=struct('summary',summary,'log',log,'maps',maps,'plotFiles',{plotFiles},'animationFile',animationFile,'outputDir',resultsDir);
end
function write_summary(s,d)
f=fopen(fullfile(d,'summary_S2_2_v0_4.txt'),'w');if f<0,return;end;c=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'Stage S2.2 v0.4 estimator-in-the-loop 6-DOF replanning\nScenario: %s | seed %d\n',s.scenario,s.seed);
fprintf(f,'Goal %d | failsafe %d | RTL %d | PASS %d\n',s.goalReached,s.failsafeTriggered,s.rtlRequested,s.pass);
fprintf(f,'Replans %d | promotions %d | inflation replans %d | dynamic avoid %d\n',s.replanCount,s.promotionCount,s.inflationReplanCount,s.dynamicAvoidSteps);
fprintf(f,'Replan brake %d | retries %d\n',s.replanBrakeCount,s.replanRetryCount);
fprintf(f,'Lane final %d | switches %d\n',s.activeLaneFinal,s.laneSwitches);
fprintf(f,'Estimator max position %.8f | attitude deg %.8f\n',s.maxEstimatorPositionError_m,s.maxEstimatorAttitudeError_deg);
fprintf(f,'Reference v/a/j %.8f %.8f %.8f\n',s.maxReferenceSpeed_mps,s.maxReferenceAccel_mps2,s.maxReferenceJerk_mps3);
fprintf(f,'Executed v/a/j %.8f %.8f %.8f\n',s.maxExecutedSpeed_mps,s.maxExecutedAccel_mps2,s.maxExecutedJerk_mps3);
fprintf(f,'Track %.8f | override %.8f | altitude %.8f | tilt %.8f deg\n',s.maxTrackingError_m,s.maxSafetyOverrideDeviation_m,s.maxAltitudeError_m,s.maxTilt_deg);
fprintf(f,'Clearance obstacle/wall/dynamic %.8f %.8f %.8f\n',s.minObstacleClearance_m,s.minWallClearance_m,s.minDynamicClearance_m);
end
function o=ternary(c,a,b),if c,o=a;else,o=b;end,end

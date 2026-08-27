function results = run_S2_2_mission_replanning(seed,scenarioName,makePlots,makeAnimation)
% RUN_S2_2_MISSION_REPLANNING Stage S2.2 v0.3 smooth dynamic replanning.
if nargin<1||isempty(seed),seed=0;end
if nargin<2||isempty(scenarioName),scenarioName='dynamic_crossing_yield';end
if nargin<3||isempty(makePlots),makePlots=true;end
if nargin<4||isempty(makeAnimation),makeAnimation=false;end
rng(seed,'twister');scriptDir=fileparts(mfilename('fullpath'));addpath(scriptDir,'-begin');cfg=init_S2_2_config();cfg.seed=seed;
versionFolder=lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_'));
cfg.resultsRoot=fullfile(fileparts(scriptDir),'results','S2_2_mission_replanning',versionFolder);
if ~exist(cfg.resultsRoot,'dir'),mkdir(cfg.resultsRoot);end
scenario=scenario_S2_2(scenarioName);label=lower(regexprep(scenario.name,'[^A-Za-z0-9]+','_'));
resultsDir=fullfile(cfg.resultsRoot,label,sprintf('seed_%03d',seed));if ~exist(resultsDir,'dir'),mkdir(resultsDir);end
fprintf('\n============================================================\n');fprintf(' STAGE S2.2 v0.3 SMOOTH DYNAMICALLY FEASIBLE REPLANNING\n');
fprintf(' seed=%d | scenario=%s | plots=%d | animation=%d\n',seed,scenario.name,makePlots,makeAnimation);
fprintf(' Results: %s\n',resultsDir);fprintf('============================================================\n');
[log,summary,maps]=mission_manager_S2_2(cfg,scenario);summary.seed=seed;summary.scenario=scenario.name;summary.outputDir=resultsDir;
fprintf('Goal / failsafe        : %d / %d\n',summary.goalReached,summary.failsafeTriggered);
fprintf('Replans / promotions   : %d / %d\n',summary.replanCount,summary.promotionCount);
fprintf('Dynamic avoid / holds  : %d / %d\n',summary.dynamicAvoidSteps,summary.dynamicHoldCount);
fprintf('No-data holds          : %d\n',summary.noDataHoldCount);
fprintf('D* repair / A* scratch : %d / %d expanded nodes\n',summary.dstarRepairExpanded,summary.astarScratchExpanded);
fprintf('Clearance obs/wall/dyn : %.3f / %.3f / %.3f m\n',summary.minObstacleClearance_m,summary.minWallClearance_m,summary.minDynamicClearance_m);
fprintf('Speed / accel / jerk   : %.3f / %.3f / %.3f\n',summary.maxSpeed_mps,summary.maxAccel_mps2,summary.maxJerk_mps3);
fprintf('Trajectories/fallbacks : %d / %d | max scale %.2f\n',summary.trajectoryGenerationCount,summary.trajectoryFallbackCount,summary.maxTrajectoryTimeScale);
fprintf('Track / override error : %.3f / %.3f m\n', ...
    summary.maxTrackingError_m,summary.maxSafetyOverrideDeviation_m);
fprintf('Rejoins / time         : %d / %.2f s\n',summary.rejoinCount,summary.timeToGoal_s);
fprintf('Reference C0/C1/C2/C3: %.3e / %.3e / %.3e / %.3e\n', ...
    summary.maxReferenceContinuityJump(1),summary.maxReferenceContinuityJump(2), ...
    summary.maxReferenceContinuityJump(3),summary.maxReferenceContinuityJump(4));
fprintf('Replan p/v/a jump     : %.3e / %.3e / %.3e\n', ...
    summary.maxReplanStateJump(1),summary.maxReplanStateJump(2),summary.maxReplanStateJump(3));
fprintf('Pass S/D/K/Ref/R/T    : %d / %d / %d / %d / %d / %d\n', ...
    summary.staticPass,summary.dynamicPass,summary.kinematicPass, ...
    summary.referenceContinuityPass,summary.replanContinuityPass,summary.trackingPass);
fprintf('Mission/failsafe/event : %d / %d / %d\n', ...
    summary.missionOutcomePass,summary.failsafeExpectationPass,summary.eventPass);
fprintf('Events I/D/N/P/TS/SE  : %d / %d / %d / %d / %d / %d\n', ...
    summary.incrementalEventPass,summary.dynamicEventPass,summary.noDataEventPass, ...
    summary.promotionEventPass,summary.timeRescalePass,summary.searchEfficiencyPass);
fprintf('Collision / geofence   : %d / %d\n',summary.collisionCount,summary.geofenceViolationCount);
fprintf('RESULT                 : %s\n\n',ternary(summary.pass,'PASS','FAIL'));
plotFiles={};if makePlots,plotFiles=plot_S2_2_dashboard(cfg,scenario,log,summary,maps,resultsDir);end
animationFile='';if makeAnimation,warning('S2_2:AnimationPending','v0.3 animation will be enabled after MATLAB logic validation.');end
save(fullfile(resultsDir,'S2_2_v0_3_trial_data.mat'),'cfg','scenario','log','summary','maps','-v7.3');write_summary(summary,resultsDir);
results=struct('summary',summary,'log',log,'maps',maps,'plotFiles',{plotFiles},'animationFile',animationFile,'outputDir',resultsDir);
end
function write_summary(s,d)
f=fopen(fullfile(d,'summary_S2_2_v0_3.txt'),'w');if f<0,return;end;c=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'Stage S2.2 v0.3 smooth dynamically feasible replanning\nScenario: %s | seed %d\n',s.scenario,s.seed);
fprintf(f,'Goal %d | failsafe %d | PASS %d\n',s.goalReached,s.failsafeTriggered,s.pass);
fprintf(f,'Replans %d | promotions %d | dynamic avoid %d | no-data holds %d\n',s.replanCount,s.promotionCount,s.dynamicAvoidSteps,s.noDataHoldCount);
fprintf(f,'Max speed/accel/jerk %.8f / %.8f / %.8f\n',s.maxSpeed_mps,s.maxAccel_mps2,s.maxJerk_mps3);
fprintf(f,'Trajectory generations %d | fallbacks %d | max scale %.8f\n',s.trajectoryGenerationCount,s.trajectoryFallbackCount,s.maxTrajectoryTimeScale);
fprintf(f,'Nominal tracking error %.8f | safety-override deviation %.8f | rejoins %d\n', ...
    s.maxTrackingError_m,s.maxSafetyOverrideDeviation_m,s.rejoinCount);
fprintf(f,'Reference C0-C3 jumps %.3e %.3e %.3e %.3e\n',s.maxReferenceContinuityJump);
fprintf(f,'Replan p-v-a jumps %.3e %.3e %.3e\n',s.maxReplanStateJump);
fprintf(f,'Core gates S D K Ref R T: %d %d %d %d %d %d\n',s.staticPass,s.dynamicPass,s.kinematicPass,s.referenceContinuityPass,s.replanContinuityPass,s.trackingPass);
fprintf(f,'Mission/failsafe/event gates: %d %d %d\n',s.missionOutcomePass,s.failsafeExpectationPass,s.eventPass);
fprintf(f,'Event gates I D N P TS SE: %d %d %d %d %d %d\n',s.incrementalEventPass,s.dynamicEventPass,s.noDataEventPass,s.promotionEventPass,s.timeRescalePass,s.searchEfficiencyPass);
end
function o=ternary(c,a,b),if c,o=a;else,o=b;end,end

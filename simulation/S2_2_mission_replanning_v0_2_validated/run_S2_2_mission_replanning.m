function results = run_S2_2_mission_replanning(seed,scenarioName,makePlots,makeAnimation)
% RUN_S2_2_MISSION_REPLANNING  Stage S2.2 v0.2 incremental/dynamic planning.
if nargin<1||isempty(seed),seed=0;end
if nargin<2||isempty(scenarioName),scenarioName='dynamic_crossing_yield';end
if nargin<3||isempty(makePlots),makePlots=true;end
if nargin<4||isempty(makeAnimation),makeAnimation=false;end
rng(seed,'twister');scriptDir=fileparts(mfilename('fullpath'));addpath(scriptDir,'-begin');

cfg=init_S2_2_config();
cfg.seed=seed;
versionFolder=lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_'));
% Canonical stage/version hierarchy:
% simulation/results/S2_2_mission_replanning/v0_2/<scenario>/seed_000/
cfg.resultsRoot=fullfile(fileparts(scriptDir),'results','S2_2_mission_replanning',versionFolder);
if ~exist(cfg.resultsRoot,'dir'),mkdir(cfg.resultsRoot);end

scenario=scenario_S2_2(scenarioName);label=lower(regexprep(scenario.name,'[^A-Za-z0-9]+','_'));
resultsDir=fullfile(cfg.resultsRoot,label,sprintf('seed_%03d',seed));if ~exist(resultsDir,'dir'),mkdir(resultsDir);end
fprintf('\n============================================================\n');
fprintf(' STAGE S2.2 v0.2 INCREMENTAL + DYNAMIC REPLANNING\n');
fprintf(' seed=%d | scenario=%s | plots=%d | animation=%d\n',seed,scenario.name,makePlots,makeAnimation);
fprintf(' Results: %s\n',resultsDir);
fprintf('============================================================\n');
[log,summary,maps]=mission_manager_S2_2(cfg,scenario);summary.seed=seed;summary.scenario=scenario.name;summary.outputDir=resultsDir;
fprintf('Goal / failsafe        : %d / %d\n',summary.goalReached,summary.failsafeTriggered);
fprintf('Replans / promotions   : %d / %d\n',summary.replanCount,summary.promotionCount);
fprintf('Dynamic avoid / holds  : %d / %d\n',summary.dynamicAvoidSteps,summary.dynamicHoldCount);
fprintf('No-data holds          : %d\n',summary.noDataHoldCount);
fprintf('D* repair / A* scratch : %d / %d expanded nodes\n',summary.dstarRepairExpanded,summary.astarScratchExpanded);
fprintf('Clearance obs/wall/dyn : %.3f / %.3f / %.3f m\n',summary.minObstacleClearance_m,summary.minWallClearance_m,summary.minDynamicClearance_m);
fprintf('Collision / geofence   : %d / %d\n',summary.collisionCount,summary.geofenceViolationCount);
fprintf('Path / time            : %.2f m / %.2f s\n',summary.pathLength_m,summary.timeToGoal_s);
fprintf('RESULT                 : %s\n\n',ternary(summary.pass,'PASS','FAIL'));
plotFiles={};if makePlots,plotFiles=plot_S2_2_dashboard(cfg,scenario,log,summary,maps,resultsDir);end
animationFile='';if makeAnimation
    warning('S2_2:AnimationPending','v0.2 dynamic animation is not yet enabled; dashboard and MAT logs are saved.');
end
save(fullfile(resultsDir,'S2_2_v0_2_trial_data.mat'),'cfg','scenario','log','summary','maps','-v7.3');
write_summary(summary,resultsDir);
results=struct('summary',summary,'log',log,'maps',maps,'plotFiles',{plotFiles},'animationFile',animationFile,'outputDir',resultsDir);
end

function write_summary(s,d)
f=fopen(fullfile(d,'summary_S2_2_v0_2.txt'),'w');if f<0,return;end;c=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'Stage S2.2 v0.2 incremental/dynamic planning\nScenario: %s | seed %d\n',s.scenario,s.seed);
fprintf(f,'Goal %d | failsafe %d | PASS %d\n',s.goalReached,s.failsafeTriggered,s.pass);
fprintf(f,'Replans %d | promotions %d | dynamic avoid steps %d | no-data holds %d\n',s.replanCount,s.promotionCount,s.dynamicAvoidSteps,s.noDataHoldCount);
fprintf(f,'D* repair expansions %d | A* scratch expansions %d\n',s.dstarRepairExpanded,s.astarScratchExpanded);
fprintf(f,'Min obstacle/wall/dynamic clearance %.6f / %.6f / %.6f m\n',s.minObstacleClearance_m,s.minWallClearance_m,s.minDynamicClearance_m);
end
function o=ternary(c,a,b),if c,o=a;else,o=b;end,end

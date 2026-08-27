function results = run_S2_2_mission_replanning(seed, scenarioName, makePlots, makeAnimation)
% RUN_S2_2_MISSION_REPLANNING
% Stage S2.2 v0.1: mission management, A* planning, online obstacle
% detection, fail-safe hover and replanning.
%
% This is a MATLAB integration reference for the Python-tested S2.2 v0.1
% OMR-FailSafe logic. It is deliberately separate from Stage S2.1.
%
% Example:
%   results = run_S2_2_mission_replanning(0,'unknown_obstacle_appears',true,false);

if nargin < 1 || isempty(seed), seed = 0; end
if nargin < 2 || isempty(scenarioName), scenarioName = 'unknown_obstacle_appears'; end
if nargin < 3 || isempty(makePlots), makePlots = true; end
if nargin < 4 || isempty(makeAnimation), makeAnimation = false; end

rng(seed,'twister');
scriptDir = fileparts(mfilename('fullpath'));
addpath(scriptDir,'-begin');

cfg = init_S2_2_config();
cfg.seed = seed;
cfg.resultsRoot = fullfile(fileparts(scriptDir),'results','S2_2_mission_replanning');
if ~exist(cfg.resultsRoot,'dir'), mkdir(cfg.resultsRoot); end

scenario = scenario_S2_2(scenarioName);
runLabel = lower(regexprep(scenario.name,'[^A-Za-z0-9]+','_'));
resultsDir = fullfile(cfg.resultsRoot,runLabel,sprintf('seed_%03d',seed));
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

fprintf('\n============================================================\n');
fprintf(' STAGE S2.2 v0.1 OMR-FAILSAFE MISSION REPLANNING\n');
fprintf(' seed=%d | scenario=%s | plots=%d | animation=%d\n',seed,scenario.name,makePlots,makeAnimation);
fprintf('============================================================\n');
fprintf('Results root: %s\n\n',resultsDir);

[log, summary, maps] = mission_manager_S2_2(cfg, scenario);
summary.seed = seed;
summary.scenario = scenario.name;
summary.outputDir = resultsDir;

fprintf('Scenario              : %s\n',scenario.name);
fprintf('Goal reached          : %d\n',summary.goalReached);
fprintf('Replans / hover stops : %d / %d\n',summary.replanCount,summary.hoverStopCount);
fprintf('Failsafe triggered    : %d\n',summary.failsafeTriggered);
fprintf('Collisions / geofence : %d / %d\n',summary.collisionCount,summary.geofenceViolationCount);
fprintf('Min clearance obs/wall: %.3f / %.3f m\n',summary.minObstacleClearance_m,summary.minWallClearance_m);
fprintf('Path length / time    : %.2f m / %.2f s\n',summary.pathLength_m,summary.timeToGoal_s);
fprintf('RESULT                : %s\n\n', ternary_S2_2(summary.pass,'PASS','FAIL'));

plotFiles = {};
if makePlots
    plotFiles = plot_S2_2_dashboard(cfg, scenario, log, summary, maps, resultsDir);
end

animationFile = '';
if makeAnimation
    animationFile = animate_S2_2_flight(cfg, scenario, log, summary, maps, resultsDir);
end

write_summary_S2_2(summary, resultsDir);
save(fullfile(resultsDir,'S2_2_trial_data.mat'),'cfg','scenario','log','summary','maps','-v7.3');

results = struct('summary',summary,'log',log,'maps',maps, ...
    'plotFiles',{plotFiles},'animationFile',animationFile,'outputDir',resultsDir);

fprintf('================ STAGE S2.2 v0.1 RESULT ================\n');
if summary.pass
    fprintf('*** PASS *** mission replanning met v0.1 safety criteria.\n');
else
    fprintf('*** FAIL *** inspect collision, geofence, replanning and failsafe logs.\n');
end
fprintf('S2.1 remains frozen; this is S2.2 planning/mission logic only.\n');
fprintf('=========================================================\n\n');
end

function write_summary_S2_2(s, resultsDir)
f = fopen(fullfile(resultsDir,'summary_S2_2.txt'),'w');
if f < 0
    warning('S2_2:SummaryWrite','Could not write summary file.');
    return;
end
c = onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'Stage S2.2 v0.1 OMR-FailSafe mission replanning\n');
fprintf(f,'Scenario: %s | seed %d\n',s.scenario,s.seed);
fprintf(f,'Goal reached: %d | PASS: %d\n',s.goalReached,s.pass);
fprintf(f,'Replans: %d | hover stops: %d | failsafe: %d\n',s.replanCount,s.hoverStopCount,s.failsafeTriggered);
fprintf(f,'Collisions: %d | geofence violations: %d\n',s.collisionCount,s.geofenceViolationCount);
fprintf(f,'Min obstacle clearance: %.6f m\n',s.minObstacleClearance_m);
fprintf(f,'Min wall clearance: %.6f m\n',s.minWallClearance_m);
fprintf(f,'Path length: %.6f m | time to goal: %.3f s\n',s.pathLength_m,s.timeToGoal_s);
end

function out = ternary_S2_2(cond,a,b)
if cond, out = a; else, out = b; end
end

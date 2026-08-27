function results = run_S2_4_coupled(seed,scenarioName,makePlots,makeAnimation)
% RUN_S2_4_COUPLED First command-enabled mission-manager coupling candidate.
%
% S2.4 outputs mission requests only. The inherited S2.3 mission manager
% copy, planner, trajectory generator, controller and plant execute them.
if nargin<1||isempty(seed),seed=0;end
if nargin<2||isempty(scenarioName),scenarioName='active_goal_requires_scan';end
if nargin<3||isempty(makePlots),makePlots=false;end
if nargin<4||isempty(makeAnimation),makeAnimation=false;end
rng(seed,'twister');

missionDir=fileparts(mfilename('fullpath'));
coupledRoot=fileparts(missionDir);
projectRoot=fileparts(coupledRoot);
parentRoot=fullfile(projectRoot,'frozen_parent','S2_3_online_mapping_v1_0_0_validated');
shadowRoot=fullfile(projectRoot,'s2_4_shadow');
scenarioRoot=fullfile(coupledRoot,'scenarios');
executionRoot=fullfile(coupledRoot,'execution');
required={parentRoot,shadowRoot,scenarioRoot,executionRoot};
for k=1:numel(required)
    if exist(required{k},'dir')~=7,error('S2_4:PathMissing','Missing path: %s',required{k});end
end
% Preserve the caller's complete MATLAB path. The runner may temporarily
% promote its dependencies, but it must not remove paths that were already
% configured by the caller. This makes repeated and multiseed runs safe.
callerPath=path;
pathGuard=onCleanup(@()restoreCallerPath(callerPath)); %#ok<NASGU>

addpath(parentRoot,'-begin');
addpath(shadowRoot,'-begin');
addpath(executionRoot,'-begin');
addpath(scenarioRoot,'-begin');
addpath(missionDir,'-begin');

cfg=init_S2_4_E_config();cfg.seed=seed;
versionFolder=lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_'));
cfg.resultsRoot=fullfile(projectRoot,'results','S2_4_coupled',versionFolder);
if exist(cfg.resultsRoot,'dir')~=7,mkdir(cfg.resultsRoot);end
scenario=scenario_S2_4(scenarioName);
label=lower(regexprep(scenario.name,'[^A-Za-z0-9]+','_'));
resultsDir=fullfile(cfg.resultsRoot,label,sprintf('seed_%03d',seed));
if exist(resultsDir,'dir')~=7,mkdir(resultsDir);end

fprintf('\n============================================================\n');
fprintf(' S2.4-E %s COUPLED ACTIVE EXPLORATION\n',cfg.version);
fprintf(' seed=%d | scenario=%s | plots=%d | animation=%d\n', ...
    seed,scenario.name,makePlots,makeAnimation);
fprintf(' Results: %s\n',resultsDir);
fprintf('============================================================\n');
[log,summary,maps]=mission_lifecycle_manager_S2_4(cfg,scenario);
summary.seed=seed;summary.scenario=scenario.name;summary.outputDir=resultsDir;

fprintf('Goal/unreachable/failsafe       : %d / %d / %d\n', ...
    summary.goalReached,summary.goalUnreachable,summary.failsafeTriggered);
fprintf('Exploration request/selected    : %d / %d\n', ...
    summary.explorationRequestCount,summary.explorationSelectedCount);
fprintf('Viewpoints executed             : %d\n',summary.explorationExecutedCount);
if isfield(summary,'explorationDecisionCount')&&summary.explorationDecisionCount>0
    fprintf('Competing decisions/frontiers   : %d / %d\n', ...
        summary.competingDecisionCount,summary.maxCompetingFrontierCount);
    fprintf('Target/irrelevant selections    : %d / %d\n', ...
        summary.targetRelevantSelectionCount,summary.irrelevantSelectionCount);
    fprintf('First selected tier/relevance   : %.0f / %.3f\n', ...
        summary.firstSelectedTier,summary.firstSelectedTargetRelevance);
    fprintf('Best decoy information gain     : %.3f\n', ...
        summary.firstBestIrrelevantInformationGain);
end
fprintf('Unsafe viewpoint execution steps: %d\n',summary.unsafeViewpointExecutionCount);
fprintf('Unknown commitments             : %d\n',summary.unknownCommitmentCount);
fprintf('Collision / geofence            : %d / %d\n', ...
    summary.collisionCount,summary.geofenceViolationCount);
fprintf('Exploration / mapping / mission : %d / %d / %d\n', ...
    summary.explorationPass,summary.mappingCompositePass,summary.missionOutcomePass);
fprintf('Final state                     : %s\n',summary.finalState);
fprintf('RESULT                          : %s\n\n',ternary(summary.pass,'PASS','FAIL'));

plotFiles={};animationFile='';
if makePlots
    plotFiles=plot_S2_3_dashboard(cfg,scenario,log,summary,maps,resultsDir);
end
if makeAnimation
    animationFile=animate_S2_3_flight(cfg,scenario,log,summary,maps,resultsDir);
end
matName=sprintf('S2_4_E_%s_trial_data.mat',versionFolder);
save(fullfile(resultsDir,matName),'cfg','scenario','log','summary','maps','-v7.3');
writeSummary(summary,resultsDir,cfg);
results=struct('summary',summary,'log',log,'maps',maps, ...
    'plotFiles',{plotFiles},'animationFile',animationFile,'outputDir',resultsDir);
end

function writeSummary(s,d,cfg)
versionFolder=lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_'));
f=fopen(fullfile(d,sprintf('summary_S2_4_E_%s.txt',versionFolder)),'w');
if f<0,return,end
c=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'S2.4-E coupled active exploration %s\n',cfg.version);
fprintf(f,'Scenario %s | seed %d | PASS %d\n',s.scenario,s.seed,s.pass);
fprintf(f,'Requests %d | selected %d | executed %d | rejected decisions %d\n', ...
    s.explorationRequestCount,s.explorationSelectedCount, ...
    s.explorationExecutedCount,s.explorationRejectedDecisionCount);
fprintf(f,'Unsafe viewpoint execution steps %d | unknown commitments %d\n', ...
    s.unsafeViewpointExecutionCount,s.unknownCommitmentCount);
if isfield(s,'explorationDecisionCount')
    fprintf(f,'Decision records %d | competing decisions %d | max frontiers %d\n', ...
        s.explorationDecisionCount,s.competingDecisionCount, ...
        s.maxCompetingFrontierCount);
    fprintf(f,'Target-relevant selections %d | irrelevant selections %d\n', ...
        s.targetRelevantSelectionCount,s.irrelevantSelectionCount);
    fprintf(f,'First tier %.0f | target relevance %.6f | best decoy information %.6f\n', ...
        s.firstSelectedTier,s.firstSelectedTargetRelevance, ...
        s.firstBestIrrelevantInformationGain);
end
fprintf(f,'Goal %d | unreachable %d | collision %d | geofence %d\n', ...
    s.goalReached,s.goalUnreachable,s.collisionCount,s.geofenceViolationCount);
fprintf(f,'Exploration pass %d | mapping pass %d | mission pass %d\n', ...
    s.explorationPass,s.mappingCompositePass,s.missionOutcomePass);
fprintf(f,'Final state %s\n',s.finalState);
end
function restoreCallerPath(callerPath)
path(callerPath);
end
function o=ternary(c,a,b),if c,o=a;else,o=b;end,end

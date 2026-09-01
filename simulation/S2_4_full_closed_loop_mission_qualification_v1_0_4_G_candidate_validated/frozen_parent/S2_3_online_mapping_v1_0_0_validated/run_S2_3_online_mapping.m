function results = run_S2_3_online_mapping(seed,scenarioName,makePlots,makeAnimation)
% RUN_S2_3_ONLINE_MAPPING Stage S2.3 cumulative candidate entry point.
if nargin<1||isempty(seed),seed=0;end
if nargin<2||isempty(scenarioName),scenarioName='unknown_room_nominal';end
if nargin<3||isempty(makePlots),makePlots=true;end
if nargin<4||isempty(makeAnimation),makeAnimation=false;end
rng(seed,'twister');
scriptDir=fileparts(mfilename('fullpath'));addpath(scriptDir,'-begin');
cfg=init_S2_3_config();cfg.seed=seed;
versionFolder=lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_'));
cfg.resultsRoot=fullfile(fileparts(scriptDir),'results','S2_3_online_mapping',versionFolder);
if ~exist(cfg.resultsRoot,'dir'),mkdir(cfg.resultsRoot);end
scenario=scenario_S2_3(scenarioName);
label=lower(regexprep(scenario.name,'[^A-Za-z0-9]+','_'));
resultsDir=fullfile(cfg.resultsRoot,label,sprintf('seed_%03d',seed));
if ~exist(resultsDir,'dir'),mkdir(resultsDir);end
fprintf('\n============================================================\n');
fprintf(' STAGE S2.3 %s PERCEPTION-DRIVEN ONLINE MAPPING\n',cfg.version);
fprintf(' seed=%d | scenario=%s | plots=%d | animation=%d\n',seed,scenario.name,makePlots,makeAnimation);
fprintf(' Results: %s\n',resultsDir);
fprintf('============================================================\n');
[log,summary,maps]=mission_lifecycle_manager_S2_3(cfg,scenario);
summary.seed=seed;summary.scenario=scenario.name;summary.outputDir=resultsDir;

fprintf('Goal/unreachable/failsafe : %d / %d / %d\n',summary.goalReached,summary.goalUnreachable,summary.failsafeTriggered);
fprintf('Arm/takeoff/RTL/land      : %d / %d / %d / %d\n',summary.armedEver,summary.takeoffCompleted,summary.rtlExecuted,summary.landed);
fprintf('Map version/packets       : %d / %d accepted / %d rejected / %d idle\n',summary.mapVersionFinal,summary.mapAcceptedPackets,summary.mapRejectedPackets,summary.mapNoDataPackets);
fprintf('Replay records captured   : %d\n',summary.perceptionReplayCount);
fprintf('Extensions done/planned    : %d / %d | scans/route repairs/all replans %d / %d / %d\n', ...
    summary.mapExtensionCount,summary.mapExtensionPlanCount,summary.scanHoldCount, ...
    summary.mapSafetyReplanCount,summary.replanCount);
fprintf('Map false-free/recall     : %.5f / %.3f\n',summary.mapFalseFreeRate,summary.mapOccupiedRecall);
fprintf('Map observed fraction     : %.3f | promotions %d\n',summary.mapObservedFraction,summary.mapPromotionCount);
fprintf('Unknown commitments       : %d | truth isolation %d\n',summary.unknownCommitmentCount,summary.truthIsolationPass);
fprintf('Collision / geofence      : %d / %d\n',summary.collisionCount,summary.geofenceViolationCount);
fprintf('Tracking / estimator max  : %.3f / %.3f m\n',summary.maxTrackingError_m,summary.maxEstimatorPositionError_m);
fprintf('Reference XY v/a/j        : %.3f / %.3f / %.3f\n',summary.maxReferenceSpeed_mps,summary.maxReferenceAccel_mps2,summary.maxReferenceJerk_mps3);
fprintf('Executed XY v/a/j         : %.3f / %.3f / %.3f\n',summary.maxExecutedSpeed_mps,summary.maxExecutedAccel_mps2,summary.maxExecutedJerk_mps3);
fprintf('Mapping/event/mission     : %d / %d / %d\n',summary.mapPass,summary.eventPass,summary.missionOutcomePass);
fprintf('Core T/C/E/S/MC           : %d / %d / %d / %d / %d\n', ...
    summary.trajectoryGate,summary.controllerGate,summary.estimatorGate, ...
    summary.staticGate,summary.mappingCompositePass);
fprintf('Trajectories/grid fallback: %d / %d\n',summary.trajectoryGenerationCount,summary.gridFallbackCount);
fprintf('Map gates M/E/R/S/U/T     : %d / %d / %d / %d / %d / %d\n', ...
    summary.mapPass,summary.mapExtensionPass,summary.mapSafetyReplanPass, ...
    summary.scanHoldPass,summary.goalUnreachablePass,summary.truthIsolationPass);
fprintf('Final state               : %s\n',summary.finalState);
fprintf('RESULT                    : %s\n\n',ternary(summary.pass,'PASS','FAIL'));

plotFiles={};if makePlots,plotFiles=plot_S2_3_dashboard(cfg,scenario,log,summary,maps,resultsDir);end
animationFile='';if makeAnimation,animationFile=animate_S2_3_flight(cfg,scenario,log,summary,maps,resultsDir);end
matName=sprintf('S2_3_%s_trial_data.mat',versionFolder);
save(fullfile(resultsDir,matName),'cfg','scenario','log','summary','maps','-v7.3');
write_summary(summary,resultsDir,cfg);
results=struct('summary',summary,'log',log,'maps',maps,'plotFiles',{plotFiles}, ...
    'animationFile',animationFile,'outputDir',resultsDir);
end

function write_summary(s,d,cfg)
versionFolder=lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_'));
f=fopen(fullfile(d,sprintf('summary_S2_3_%s.txt',versionFolder)),'w');if f<0,return;end
c=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'Stage S2.3 %s perception-driven online mapping\n',cfg.version);
fprintf(f,'Scenario %s | seed %d | PASS %d\n',s.scenario,s.seed,s.pass);
fprintf(f,'Goal %d | unreachable %d | failsafe %d | RTL %d | landed %d\n',s.goalReached,s.goalUnreachable,s.failsafeTriggered,s.rtlExecuted,s.landed);
fprintf(f,'Map version %d | accepted %d | rejected %d | idle %d | replay records %d | extensions completed %d | planned %d | scans %d | route repairs %d | all replans %d\n', ...
    s.mapVersionFinal,s.mapAcceptedPackets,s.mapRejectedPackets,s.mapNoDataPackets,s.perceptionReplayCount, ...
    s.mapExtensionCount,s.mapExtensionPlanCount,s.scanHoldCount,s.mapSafetyReplanCount,s.replanCount);
fprintf(f,'False free %.10f | occupied recall %.10f | observed %.10f\n',s.mapFalseFreeRate,s.mapOccupiedRecall,s.mapObservedFraction);
fprintf(f,'Unknown commitments %d | truth isolation %d | map pass %d\n',s.unknownCommitmentCount,s.truthIsolationPass,s.mapPass);
fprintf(f,'Map gates extension %d | safety replan %d | scan bound %d | unreachable %d\n', ...
    s.mapExtensionPass,s.mapSafetyReplanPass,s.scanHoldPass,s.goalUnreachablePass);
fprintf(f,'Collision %d | geofence %d | final state %s\n',s.collisionCount,s.geofenceViolationCount,s.finalState);
end
function o=ternary(c,a,b),if c,o=a;else,o=b;end,end

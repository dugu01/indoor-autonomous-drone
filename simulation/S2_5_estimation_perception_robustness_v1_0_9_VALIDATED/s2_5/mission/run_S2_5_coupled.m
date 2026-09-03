function results = run_S2_5_coupled(seed,scenarioName,makePlots,makeAnimation,saveArtifacts,verbose)
% RUN_S2_5_COUPLED Estimation/perception robustness overlay on frozen S2.4-G.
if nargin<1||isempty(seed),seed=0;end
if nargin<2||isempty(scenarioName),scenarioName='baseline';end
if nargin<3||isempty(makePlots),makePlots=false;end
if nargin<4||isempty(makeAnimation),makeAnimation=false;end
if nargin<5||isempty(saveArtifacts),saveArtifacts=true;end
if nargin<6||isempty(verbose),verbose=true;end
rng(seed,'twister');

missionDir=fileparts(mfilename('fullpath'));
s25Root=fileparts(missionDir);projectRoot=fileparts(s25Root);
parentRoot=fullfile(projectRoot,'frozen_parent','S2_3_online_mapping_v1_0_0_validated');
shadowRoot=fullfile(projectRoot,'s2_4_shadow');
coupledRoot=fullfile(projectRoot,'coupled');
scenarioS24=fullfile(coupledRoot,'scenarios');executionS24=fullfile(coupledRoot,'execution');missionS24=fullfile(coupledRoot,'mission');
scenarioS25=fullfile(s25Root,'scenarios');sensorS25=fullfile(s25Root,'sensors');perceptionS25=fullfile(s25Root,'perception');
required={parentRoot,shadowRoot,scenarioS24,executionS24,missionS24,scenarioS25,sensorS25,perceptionS25};
for i=1:numel(required),if exist(required{i},'dir')~=7,error('S2_5:PathMissing','Missing path: %s',required{i});end,end
callerPath=path;guard=onCleanup(@()path(callerPath)); %#ok<NASGU>
addpath(parentRoot,'-begin');addpath(shadowRoot,'-begin');addpath(executionS24,'-begin');addpath(scenarioS24,'-begin');addpath(missionS24,'-begin');
addpath(sensorS25,'-begin');addpath(perceptionS25,'-begin');addpath(scenarioS25,'-begin');addpath(missionDir,'-begin');

cfg=init_S2_5_config();cfg.seed=seed;
scenario=scenario_S2_5(scenarioName);
versionFolder=lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_'));
cfg.resultsRoot=fullfile(projectRoot,'results','S2_5_estimation_perception',versionFolder);
label=lower(regexprep(scenario.name,'[^A-Za-z0-9]+','_'));
resultsDir=fullfile(cfg.resultsRoot,label,sprintf('seed_%03d',seed));
if exist(resultsDir,'dir')~=7,mkdir(resultsDir);end

if verbose
    fprintf('\n============================================================\n');
    fprintf(' S2.5 %s ESTIMATION + PERCEPTION ROBUSTNESS\n',cfg.version);
    fprintf(' seed=%d | scenario=%s | plots=%d | animation=%d\n',seed,scenario.name,makePlots,makeAnimation);
    fprintf(' Results: %s\n',resultsDir);
    fprintf('============================================================\n');
end
[log,summary,maps]=mission_lifecycle_manager_S2_5(cfg,scenario);
summary.seed=seed;summary.scenario=scenario.name;summary.outputDir=resultsDir;
summary.s25CasePass=evaluate_s25_case(summary,scenario);

if verbose
    fprintf('Mission/goal/failsafe/emergency : %d / %d / %d / %d\n',summary.missionComplete,summary.goalReached,summary.failsafeTriggered,summary.emergencyLanding);
    fprintf('Estimator pos/att/gate          : %.4f m / %.4f deg / %d\n',summary.maxEstimatorPositionError_m,summary.maxEstimatorAttitudeError_deg,summary.estimatorGate);
    fprintf('Lane switches / active final    : %d / %d\n',summary.laneSwitches,summary.activeLaneFinal);
    fprintf('Map accepted/rejected/hold      : %d / %d / %d\n',summary.mapAcceptedPackets,summary.mapRejectedPackets,summary.perceptionHoldCount);
    fprintf('S2.5 nav/perception fault apps  : %d / %d\n',summary.s25SensorFaultApplicationCount,summary.s25PerceptionFaultApplicationCount);
    fprintf('Unknown/collision/geofence      : %d / %d / %d\n',summary.unknownCommitmentCount,summary.collisionCount,summary.geofenceViolationCount);
    fprintf('Truth isolation / S2.5 case     : %d / %s\n',summary.truthIsolationPass,ternary(summary.s25CasePass,'PASS','FAIL'));
    fprintf('Final state                     : %s\n\n',summary.finalState);
end

plotFiles={};animationFile='';
if makePlots,plotFiles=plot_S2_3_dashboard(cfg,scenario,log,summary,maps,resultsDir);end
if makeAnimation,animationFile=animate_S2_3_flight(cfg,scenario,log,summary,maps,resultsDir);end
if saveArtifacts
    save(fullfile(resultsDir,sprintf('S2_5_%s_trial_data.mat',versionFolder)),'cfg','scenario','log','summary','maps','-v7.3');
end
results=struct('summary',summary,'log',log,'maps',maps,'cfg',cfg,'scenario',scenario, ...
    'plotFiles',{plotFiles},'animationFile',animationFile,'outputDir',resultsDir);
end

function pass=evaluate_s25_case(s,scenario)
missionExpected=logical(field_or_local(scenario,'expectedS25MissionComplete',true));
failsafeExpected=logical(field_or_local(scenario,'expectedS25Failsafe',false));
emergencyExpected=logical(field_or_local(scenario,'expectedS25EmergencyLanding',false));
laneExpected=logical(field_or_local(scenario,'expectedS25LaneSwitch',false));
holdExpected=logical(field_or_local(scenario,'expectedS25PerceptionHold',false));
minReject=field_or_local(scenario,'expectedS25MinMapRejectedPackets',0);
minNav=field_or_local(scenario,'expectedS25MinNavFaultApplications',0);
minPer=field_or_local(scenario,'expectedS25MinPerceptionFaultApplications',0);
missionOK=(logical(s.missionComplete)==missionExpected);
failsafeOK=(logical(s.failsafeTriggered)==failsafeExpected);
emergencyOK=(logical(s.emergencyLanding)==emergencyExpected);
laneOK=~laneExpected||s.laneSwitches>=1;
holdOK=~holdExpected||field_or_local(s,'s25PerceptionSafeResponseCount',s.perceptionHoldCount)>=1;
faultOK=s.s25SensorFaultApplicationCount>=minNav&&s.s25PerceptionFaultApplicationCount>=minPer;
mapRejectOK=s.mapRejectedPackets>=minReject;
% For controlled failsafe cases the inherited scenario-specific estimator
% gate uses its observable/failsafe metric. Otherwise retain the nominal gate.
coreOK=s.missionOutcomePass&&s.trajectoryGate&&s.controllerGate&&s.estimatorGate&&s.continuityPass&&s.uncertaintyPass&&s.staticGate&&field_or_local(s,'s25MappingCompositePass',s.mappingCompositePass)&&s.executionSafetyPass;
safetyOK=s.collisionCount==0&&s.geofenceViolationCount==0&&s.unknownCommitmentCount==0&& ...
    s.unsafeViewpointExecutionCount==0&&s.staleCommandContinuationCount==0&&s.truthIsolationPass;
pass=missionOK&&failsafeOK&&emergencyOK&&laneOK&&holdOK&&faultOK&&mapRejectOK&&coreOK&&safetyOK&&~s.stateTimeoutTriggered;
end
function v=field_or_local(s,n,d),if isstruct(s)&&isfield(s,n),v=s.(n);else,v=d;end,end
function o=ternary(c,a,b),if c,o=a;else,o=b;end,end

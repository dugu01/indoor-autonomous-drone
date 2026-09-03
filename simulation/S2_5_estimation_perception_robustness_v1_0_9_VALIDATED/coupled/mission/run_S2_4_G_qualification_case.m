function results = run_S2_4_G_qualification_case(seed,faultName,timingName,scenarioName)
% RUN_S2_4_G_QUALIFICATION_CASE Targeted F mechanism robustness exercise.
% This is a qualification wrapper only. It does not modify the validated F
% supervisor, planner, trajectory generator, controller, estimator or plant.
if nargin<1||isempty(seed),seed=0;end
if nargin<2||isempty(faultName),faultName='F2';end
if nargin<3||isempty(timingName),timingName='mid';end
if nargin<4||isempty(scenarioName),scenarioName='active_goal_requires_scan';end
rng(seed,'twister');
missionDir=fileparts(mfilename('fullpath'));coupledRoot=fileparts(missionDir);projectRoot=fileparts(coupledRoot);
parentRoot=fullfile(projectRoot,'frozen_parent','S2_3_online_mapping_v1_0_0_validated');
shadowRoot=fullfile(projectRoot,'s2_4_shadow');scenarioRoot=fullfile(coupledRoot,'scenarios');executionRoot=fullfile(coupledRoot,'execution');
callerPath=path;guard=onCleanup(@()path(callerPath)); %#ok<NASGU>
addpath(parentRoot,'-begin');addpath(shadowRoot,'-begin');addpath(executionRoot,'-begin');addpath(scenarioRoot,'-begin');addpath(missionDir,'-begin');

cfg=init_S2_4_F_config();
cfg.stage='S2.4-G';
cfg.version='v0.5.4-full-closed-loop-yaw-slew-candidate';
cfg.seed=seed;
[timingProgress,timingDelay]=timingProfile(timingName);
cfg.executionSafety.validationFault=criticalFaultConfig(faultName,timingProgress,timingDelay);
scenario=scenario_S2_4(scenarioName);
[log,summary,maps]=mission_lifecycle_manager_S2_4(cfg,scenario);
summary.seed=seed;summary.scenario=scenario.name;summary.faultName=upper(char(faultName));
summary.qualificationTiming=lower(char(timingName));summary.qualificationTriggerProgress=timingProgress;
results=struct('summary',summary,'log',log,'maps',maps,'cfg',cfg,'scenario',scenario);
end

function f=criticalFaultConfig(name,progress,delay_s)
name=upper(char(name));
allowed={'F2','F3','F6','F9','F10'};
assert(any(strcmp(name,allowed)),'S2_4:GUnsupportedFault','Unsupported G critical fault: %s',name);
f=struct('name',name,'triggerDelay_s',delay_s,'triggerProgress',progress, ...
    'versionOffset',1000000,'repeatPerAuthority',false);
end

function [progress,delay_s]=timingProfile(name)
name=lower(char(name));delay_s=0.05;
switch name
    case 'early',progress=0.20;
    case 'mid',  progress=0.50;
    case 'late', progress=0.75;
    otherwise,error('S2_4:GUnknownTiming','Unknown G timing profile: %s',name);
end
end

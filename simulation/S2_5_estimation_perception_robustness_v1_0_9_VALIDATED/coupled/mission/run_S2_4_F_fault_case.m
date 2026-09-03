function results = run_S2_4_F_fault_case(seed,faultName,scenarioName)
% RUN_S2_4_F_FAULT_CASE Coupled MATLAB execution-time fault exercise.
if nargin<1||isempty(seed),seed=0;end
if nargin<2||isempty(faultName),faultName='F1';end
if nargin<3||isempty(scenarioName),scenarioName='active_goal_requires_scan';end
rng(seed,'twister');
missionDir=fileparts(mfilename('fullpath'));coupledRoot=fileparts(missionDir);projectRoot=fileparts(coupledRoot);
parentRoot=fullfile(projectRoot,'frozen_parent','S2_3_online_mapping_v1_0_0_validated');
shadowRoot=fullfile(projectRoot,'s2_4_shadow');scenarioRoot=fullfile(coupledRoot,'scenarios');executionRoot=fullfile(coupledRoot,'execution');
callerPath=path;guard=onCleanup(@()path(callerPath)); %#ok<NASGU>
addpath(parentRoot,'-begin');addpath(shadowRoot,'-begin');addpath(executionRoot,'-begin');addpath(scenarioRoot,'-begin');addpath(missionDir,'-begin');
cfg=init_S2_4_F_config();cfg.seed=seed;
cfg.executionSafety.validationFault=faultConfig(faultName);
scenario=scenario_S2_4(scenarioName);
[log,summary,maps]=mission_lifecycle_manager_S2_4(cfg,scenario);
summary.seed=seed;summary.scenario=scenario.name;summary.faultName=upper(char(faultName));
results=struct('summary',summary,'log',log,'maps',maps,'cfg',cfg,'scenario',scenario);
end
function f=faultConfig(name)
name=upper(char(name));
f=struct('name',name,'triggerDelay_s',0.10,'triggerProgress',0.20, ...
    'versionOffset',1000000,'repeatPerAuthority',false);
switch name
    case 'F1'
        f.name='F1';
    case {'F2','F3','F4','F5','F6','F7','F8','F9','F13'}
        % One post-acceptance event is sufficient. F13 is deliberately a
        % transient one-cycle obstruction; the revoked generation may never
        % resume and any later motion needs a newly accepted/revalidated authority.
    case 'F10'
        f.triggerProgress=0.45;
    case 'F11'
        % Wait until a material route prefix has actually been traversed.
        f.triggerProgress=0.45;
    case 'F14'
        % Re-inject once for each newly accepted authority generation until
        % the explicit invalidation bound terminates the retry sequence.
        f.triggerDelay_s=0.05;f.triggerProgress=0.10;f.repeatPerAuthority=true;
    case 'F15'
        f.name='F15';
end
end

function info = setup_S2_4_E_path(projectRoot)
% SETUP_S2_4_E_PATH Configure the coupled S2.4-E MATLAB session once.
%
% Run from the project root:
%   info = setup_S2_4_E_path();
%
% This function is idempotent. Re-running it does not remove any existing
% MATLAB paths. The coupled folders are promoted ahead of the frozen parent,
% while the frozen parent itself is never modified.

if nargin < 1 || isempty(projectRoot)
    projectRoot = fileparts(mfilename('fullpath'));
end
projectRoot = char(projectRoot);

coupledRoot  = fullfile(projectRoot,'coupled');
missionRoot  = fullfile(coupledRoot,'mission');
executionRoot= fullfile(coupledRoot,'execution');
scenarioRoot = fullfile(coupledRoot,'scenarios');
validationRoot=fullfile(coupledRoot,'validation');
shadowRoot   = fullfile(projectRoot,'s2_4_shadow');
parentRoot   = fullfile(projectRoot,'frozen_parent', ...
    'S2_3_online_mapping_v1_0_0_validated');

required = {missionRoot,executionRoot,scenarioRoot,validationRoot, ...
    shadowRoot,parentRoot};
for k = 1:numel(required)
    assert(isfolder(required{k}), ...
        'S2_4:PathMissing','Missing required folder: %s',required{k});
end

% Add parent first, then promote coupled layers. The last '-begin' call has
% highest precedence, so mission/validation entry points resolve correctly.
addpath(parentRoot,'-end');
addpath(shadowRoot,'-begin');
addpath(executionRoot,'-begin');
addpath(scenarioRoot,'-begin');
addpath(missionRoot,'-begin');
addpath(validationRoot,'-begin');

rehash;

info = struct();
info.projectRoot = projectRoot;
info.missionFunction = which('run_S2_4_coupled');
info.requestFunction = which('exploration_request_S2_4');
info.validationFunction = which('validate_S2_4_E_all');
info.pass = ~isempty(info.missionFunction) && ...
            ~isempty(info.requestFunction) && ...
            ~isempty(info.validationFunction);

fprintf('\nS2.4-E MATLAB PATH SETUP: %s\n',localTernary(info.pass,'PASS','FAIL'));
fprintf(' run_S2_4_coupled             : %s\n',info.missionFunction);
fprintf(' exploration_request_S2_4    : %s\n',info.requestFunction);
fprintf(' validate_S2_4_E_all          : %s\n\n',info.validationFunction);

assert(info.pass,'S2_4:PathSetupFailed', ...
    'One or more S2.4-E entry points could not be resolved.');
end

function out = localTernary(condition,a,b)
if condition
    out = a;
else
    out = b;
end
end

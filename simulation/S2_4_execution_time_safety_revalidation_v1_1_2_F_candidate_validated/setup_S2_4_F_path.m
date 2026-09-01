function info = setup_S2_4_F_path(projectRoot)
% SETUP_S2_4_F_PATH Configure S2.4-F and return its validation entry point.
if nargin<1||isempty(projectRoot),projectRoot=fileparts(mfilename('fullpath'));end
setup_S2_4_E_path(projectRoot);
validationRoot=fullfile(projectRoot,'coupled','validation');missionRoot=fullfile(projectRoot,'coupled','mission');executionRoot=fullfile(projectRoot,'coupled','execution');
addpath(executionRoot,'-begin');addpath(missionRoot,'-begin');addpath(validationRoot,'-begin');rehash;
info=struct('pass',~isempty(which('validate_S2_4_F_all')),'validationFunction',which('validate_S2_4_F_all'));
assert(info.pass,'S2_4:FPathSetupFailed','validate_S2_4_F_all not resolved.');
end

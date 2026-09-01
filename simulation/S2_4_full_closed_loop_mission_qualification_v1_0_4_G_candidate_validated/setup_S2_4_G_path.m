function info = setup_S2_4_G_path(projectRoot)
% SETUP_S2_4_G_PATH Configure the S2.4-G qualification layer.
if nargin<1||isempty(projectRoot),projectRoot=fileparts(mfilename('fullpath'));end
setup_S2_4_F_path(projectRoot);
validationRoot=fullfile(projectRoot,'coupled','validation');
missionRoot=fullfile(projectRoot,'coupled','mission');
addpath(missionRoot,'-begin');addpath(validationRoot,'-begin');rehash;
info=struct('pass',~isempty(which('validate_S2_4_G_all'))&& ...
    ~isempty(which('run_S2_4_G_qualification_case')), ...
    'validationFunction',which('validate_S2_4_G_all'), ...
    'caseFunction',which('run_S2_4_G_qualification_case'));
assert(info.pass,'S2_4:GPathSetupFailed','S2.4-G qualification functions not resolved.');
end

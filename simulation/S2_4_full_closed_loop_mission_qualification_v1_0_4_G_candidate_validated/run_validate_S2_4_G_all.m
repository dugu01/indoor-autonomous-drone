function gate = run_validate_S2_4_G_all()
% RUN_VALIDATE_S2_4_G_ALL One-command full S2.4-G qualification.
% 1) re-runs the complete A-D MATLAB shadow gate,
% 2) re-runs E + validated F coupled qualification,
% 3) executes the targeted 75-run critical fault timing/seed matrix,
% 4) rechecks frozen-parent integrity after all coupled runs.
projectRoot=fileparts(mfilename('fullpath'));
oldPwd=pwd;guard=onCleanup(@()cd(oldPwd)); %#ok<NASGU>
setenv('PYTHONDONTWRITEBYTECODE','1');

% Run the historical A-D script in the base workspace because that script
% intentionally begins with clear/clc/close. Any failed A-D gate throws.
fprintf('\n============================================================\n');
fprintf(' S2.4-G PHASE 1 — COMPLETE A-D SHADOW QUALIFICATION\n');
fprintf('============================================================\n');
cd(projectRoot);
rootEsc=strrep(projectRoot,'''','''''');
evalin('base',sprintf('cd(''%s''); run_validate_S2_4_AD_all;',rootEsc));

% A-D deliberately manipulates MATLAB path/cwd. Re-establish this package.
cd(projectRoot);restoredefaultpath;rehash toolboxcache;
addpath(projectRoot,'-begin');setup_S2_4_G_path(projectRoot);
gate=validate_S2_4_G_all(projectRoot);
assert(gate.pass,'S2_4:GValidationFailed','S2.4-G full closed-loop qualification failed.');
end

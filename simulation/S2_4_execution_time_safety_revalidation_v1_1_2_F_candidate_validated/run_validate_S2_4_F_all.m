function gate = run_validate_S2_4_F_all()
% RUN_VALIDATE_S2_4_F_ALL One-command MATLAB entry point for S2.4-F.
projectRoot=fileparts(mfilename('fullpath'));
setup_S2_4_F_path(projectRoot);
gate=validate_S2_4_F_all();
assert(gate.pass,'S2_4:FValidationFailed','S2.4-F combined MATLAB gate failed.');
end

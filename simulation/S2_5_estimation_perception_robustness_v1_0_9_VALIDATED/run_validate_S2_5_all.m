function gate = run_validate_S2_5_all()
% RUN_VALIDATE_S2_5_ALL Single-command S2.5 qualification entry point.
root=fileparts(mfilename('fullpath'));old=pwd;guard=onCleanup(@()cd(old)); %#ok<NASGU>
cd(root);setup_S2_5_path();
gate=validate_S2_5_all();
assert(gate.pass,'S2_5:ValidationFailed','S2.5 estimation/perception robustness gate failed.');
end

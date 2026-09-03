function report = run_validate_S2_5_v1_0_6_preflight()
% RUN_VALIDATE_S2_5_V1_0_6_PREFLIGHT Single-command five-case fail-fast gate.
root=fileparts(mfilename('fullpath'));old=pwd;guard=onCleanup(@()cd(old)); %#ok<NASGU>
cd(root);setup_S2_5_path();
report=run_S2_5_v1_0_6_historical_preflight();
assert(report.pass,'S2_5:V106HistoricalPreflightFailed','S2.5 v1.0.6 five-case preflight failed.');
end

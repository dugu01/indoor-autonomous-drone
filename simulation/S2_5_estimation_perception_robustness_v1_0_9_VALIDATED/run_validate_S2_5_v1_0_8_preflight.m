function report = run_validate_S2_5_v1_0_8_preflight()
root=fileparts(mfilename('fullpath'));old=pwd;guard=onCleanup(@()cd(old)); %#ok<NASGU>
cd(root);setup_S2_5_path();report=run_S2_5_v1_0_8_remaining_four_preflight();
end

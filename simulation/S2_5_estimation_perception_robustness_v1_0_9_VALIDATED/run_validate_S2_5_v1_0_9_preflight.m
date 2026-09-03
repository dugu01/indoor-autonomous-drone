function report = run_validate_S2_5_v1_0_9_preflight()
root=fileparts(mfilename('fullpath'));cd(root);setup_S2_5_path();report=run_S2_5_v1_0_9_remaining_four_preflight();
end

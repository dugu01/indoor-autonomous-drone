# MATLAB execution protocol

MATLAB was not available in the package-generation environment. The MATLAB source is supplied for the user to run.

From the S2.4 development folder:

```matlab
addpath(fullfile(pwd,'s2_4_shadow'));
trial = '/absolute/path/to/final/S2_3_v1_0_0_candidate_trial_data.mat';
out = fullfile(pwd,'results','nominal_shadow');
gate = validate_S2_4_AD(trial,out);
```

Required order:

1. Run `python tools/audit_parent_immutability.py`.
2. Run `python tools/audit_truth_isolation_s2_4.py`.
3. Run `python python_tests/s2_4_ad_contract_backtest.py`.
4. Run the Python recorded replay with the nominal MAT trace.
5. Run `validate_S2_4_AD` in MATLAB for nominal and each compatible final difficult S2.3 raw trace. Do not substitute older incompatible schemas.
6. Review every rejected candidate and selected utility breakdown.
7. Do not connect `shadowRecommendation` to the lifecycle manager until the complete A-D MATLAB gate passes.

The MATLAB runner first invokes `replay_perception_log_S2_3`. A failed inherited exact mapper replay aborts S2.4 immediately.

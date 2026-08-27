# Final clean-build verification

Do not overlay this folder onto the development workspace. Extract it as a new directory.

## 1. Static/offline gate
```bash
python3 coupled/validation/run_all_checks_S2_4_E.py
```
Required: aggregate static/offline PASS.

## 2. A–D MATLAB verification
```matlab
run_validate_S2_4_AD_all
```

## 3. Fresh MATLAB session
```matlab
clear; clc; close all force;
restoredefaultpath;
rehash toolboxcache;
```
`cd` to the clean release root, then run:
```matlab
report = run_validate_S2_4_release_candidate();
disp(report);
assert(report.pass);
```

Expected:
- request contracts PASS;
- controlled adversarial policy PASS;
- literal geometry PASS;
- Milestone 1 PASS;
- layered seed-0 benchmark PASS;
- physical seeds 0:9 = 10/10 PASS;
- clean-decoy activation = 4/10 on `[0 3 4 7]`;
- reference parity PASS.

If this passes, make no executable edits before final freeze.

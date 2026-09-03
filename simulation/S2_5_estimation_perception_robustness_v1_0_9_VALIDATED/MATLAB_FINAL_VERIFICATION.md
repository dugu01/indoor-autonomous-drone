# MATLAB Final Verification — S2.4-G v1.0.4

Run from the freshly extracted package root:

```matlab
gate = run_validate_S2_4_G_all();
```

The command first reruns A-D and E/F, then 5 no-fault seeds. Before the rest of the critical campaign it executes the four historical residuals:

- F2 MID seed=3
- F3 MID seed=3
- F9 MID seed=3
- F10 MID seed=3

Each must pass the unchanged estimator gate (`maxEstimatorAttitudeError_deg <= 2.0`) and all other G gates. Their results are cached and reused in the full matrix, so there are still exactly 75 unique critical fault runs.

Expected preflight format:

```text
F2 MID seed=3 : PASS | E=1 att=<2.0/2.0 deg yawRateMax=... mission=1 core=1 safety=1 truth=1
```

Final acceptance remains:

- A-D PASS
- reviewed runtime delta PASS
- E+F PASS
- no-fault baselines 5/5 PASS
- critical matrix 75/75 PASS
- mission completion PASS
- closed-loop integrity PASS
- hard safety PASS
- actual truth access isolation PASS
- frozen-parent integrity PASS

No v1.0.4 MATLAB coupled PASS is claimed until this command succeeds in MATLAB.

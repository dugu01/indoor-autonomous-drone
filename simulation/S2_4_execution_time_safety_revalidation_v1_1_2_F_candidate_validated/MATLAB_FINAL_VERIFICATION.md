# S2.4-F v1.1.2 MATLAB verification

Use a newly extracted folder. Do not overlay it on an older F candidate.

In a fresh MATLAB session:

```matlab
clear; clc; close all force;
restoredefaultpath;
rehash toolboxcache;
```

`cd` to the extracted package root and run the single qualification command:

```matlab
gate = run_validate_S2_4_F_all();
```

Required before F can be frozen:
- inherited S2.4-E combined MATLAB gate remains PASS;
- deterministic F contracts PASS, including the runtime/planning stop-gate regression;
- F1 no-fault E reference parity PASS;
- F2-F11/F13/F14 coupled qualification PASS;
- stale command continuation = 0;
- collision = 0;
- geofence violations = 0;
- unknown commitments = 0;
- unsafe viewpoint execution = 0;
- truth isolation PASS;
- F14 terminates at the configured authority-invalidation bound rather than state timeout;
- F15 is printed as NOT APPLICABLE, not PASS as a predictive interface test.

If a case fails, the wrapper prints the last safety reason plus:
- stopping distance `d_stop`;
- remaining route length;
- whether the route itself provides the stopping reserve;
- terminal-overrun shortfall;
- whether terminal-overrun support is known-free.

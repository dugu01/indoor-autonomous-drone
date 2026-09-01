# S2.4 A-D validation gates

## Completed in this package-generation environment

- uploaded S2.3 parent byte identity: PASS, 208/208 files;
- final S2.3 closure manifest: PASS, 203/203 paths;
- inherited S2.3 static audit: PASS;
- inherited S2.3 truth-isolation audit: PASS;
- inherited MATLAB source sanity: PASS, 85 files;
- inherited mechanism tests: PASS, 23/23;
- inherited Python scenario contracts: PASS, 12/12;
- S2.4 truth and command isolation audit: PASS;
- S2.4 MATLAB textual/source sanity: PASS, 9 files;
- S2.4 combined scenario contracts: PASS, 15/15;
- recorded nominal S2.3 shadow replay: PASS;
- uncertainty replay repeat digest: exact match;
- incremental frontier extraction equals full extraction: PASS for all 81 snapshots;
- accepted viewpoints in the final nominal replay: 0 under exact physical-and-unknown inflation;
- unsafe accepted viewpoints: 0;
- accepted-viewpoint and target-ranking mechanisms: PASS in the combined 15-scenario matrix;
- unbounded frontier tracks: 0; 19 tracks total in the nominal replay;
- S2.4 flight commands issued: 0.

## Requires the user's MATLAB installation

- execution of the supplied MATLAB A-D shadow package;
- inherited exact mapper replay called from the MATLAB S2.4 runner;
- MATLAB nominal trace gate;
- MATLAB difficult-scenario trace replay gate when compatible final raw MAT traces are supplied;
- exact MATLAB candidate-score repeat comparison;
- final review of every rejection reason and target-ranking decision.

## Not enabled by this A-D package

- connecting a selected viewpoint to the S2.3 mission manager;
- coupled active-exploration flight;
- final deterministic 15/15 coupled MATLAB release matrix;
- final critical 60/60 coupled multi-seed matrix.

Those gates belong to the next integration package after A-D shadow validation succeeds in MATLAB.

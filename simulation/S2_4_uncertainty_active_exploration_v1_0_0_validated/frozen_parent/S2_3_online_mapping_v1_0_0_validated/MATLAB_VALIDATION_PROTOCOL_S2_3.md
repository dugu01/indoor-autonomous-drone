# MATLAB validation protocol — Stage S2.3 cumulative candidate

1. Keep the frozen S2.2 folder unchanged.
2. Replace only the S2.3 development folder with this cumulative package.
3. Activate only the S2.3 development folder on the MATLAB path.
4. Run `which ... -all` for the entry point, lifecycle manager, mapper,
   route-change checker and strict trajectory adapter.
5. Run `UNKNOWN_ROOM_NOMINAL`, seed 0 without plots or animation.
6. Preserve and inspect the complete console output and v7.3 trial-data MAT file.
7. Check, individually:
   - completed versus planned map extensions;
   - scan holds, route repairs and total replans;
   - goal completion and landing completion;
   - false-free rate and occupied recall;
   - unique static promotions;
   - reference and executed speed/acceleration/jerk;
   - estimator attitude error;
   - collision, geofence, unknown commitment and truth isolation.
8. Only after nominal acceptance, rerun with the tabbed dashboard.
9. Run deterministic scenarios individually.
10. Convert every discovered failure into a permanent focused MATLAB regression.
11. Run `validate_S2_3(false)`.
12. Run `validate_S2_3_legacy_regression()`.
13. Run `validate_S2_3_multiseed(0:9)` only after deterministic acceptance.

No MATLAB validation claim is permitted until the actual outputs have been
supplied and reviewed. Python and static checks are mechanism evidence only.

## Required rerun after third-run cross-check

Run only `UNKNOWN_ROOM_NOMINAL`, seed 0. Confirm before any matrix run:

1. `goalReached = 1` and `goalUnreachable = 0`;
2. final state is `COMPLETE`;
3. map false-free rate is <= 0.005;
4. map occupied recall is >= 0.95;
5. collision, geofence and unknown-commitment counts remain zero;
6. reference and executed kinematic gates remain PASS;
7. truth isolation remains PASS.

If any item fails, preserve the complete console output and trial MAT file and
do not continue to the deterministic matrix.

## Required coupled rerun after exact boundary-policy replay

The next MATLAB run must use the integrated known-boundary candidate. Run only
`UNKNOWN_ROOM_NOMINAL`, seed 0. Required gates:

- exact mapper replay after the run: PASS;
- goal reached / unreachable: 1 / 0;
- final lifecycle state: COMPLETE;
- false-free rate <= 0.005;
- occupied recall >= 0.95;
- zero collision, geofence, and unknown-space commitments;
- mapping, mission lifecycle, controller, estimator, trajectory and truth
  isolation gates all PASS.

Do not start deterministic or multi-seed matrices until this coupled nominal
run and its exact packet replay both pass.

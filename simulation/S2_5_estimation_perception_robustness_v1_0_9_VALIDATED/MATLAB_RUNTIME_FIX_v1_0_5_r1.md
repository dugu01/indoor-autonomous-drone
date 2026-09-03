# S2.5 v1.0.5 r1 MATLAB runtime fix

This revision does not change the CSE/SIE architecture or any safety/estimator/map thresholds.

Runtime correction:
- `s2_5/mission/plan_recovery_viewpoint_S2_5.m` line 371 used the invalid MATLAB token `!` in `&&!continuous_segment_safe_local(...)`.
- Corrected to MATLAB logical negation: `&&~continuous_segment_safe_local(...)`.

Validation-process correction:
- MATLAB source audit now rejects stray `!` operators in S2.5 function code.
- Historical preflight now prints the existing worker `errorIdentifier` and `errorMessage` before failing.
- The qualification worker itself remains byte-identical to v1.0.4-r1.

No files under the frozen validated S2.4-G parent or frozen S2.3 parent were modified.

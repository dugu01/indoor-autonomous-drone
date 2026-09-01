# Pre-cleanup validation reference

This is the expected parity target for the clean-build MATLAB verification.

## Controlled adversarial policy
- target information gain / raw utility: `5 / 0.32`
- decoy information gain / raw utility: `12 / 0.47`
- both safety-feasible
- target Tier 1, decoy Tier 3
- target selected
- contract: PASS

## Physical coupled benchmark
- upper/lower fork safe widths: `0.646 / 0.896 m`
- target/decoy branch safe widths: `0.996 / 2.196 m`
- target/decoy route lengths: `5.414 / 5.256 m`
- geometry contract: PASS

## MATLAB seeds 0:9
- mission PASS: `10/10`
- clean-decoy activation: `4/10`, seeds `[0 3 4 7]`
- target selections: `10/10`
- irrelevant selections: `0`
- collisions: `0`
- geofence violations: `0`
- unknown commitments: `0`
- unsafe viewpoint executions: `0`
- goal reached: `10/10`
- RTL + landing: `10/10`
- truth isolation: PASS
- layered multiseed V6 gate: PASS

The physical decoy was not more informative than the selected target in the physical runs.
The stronger `IG_D > IG_T` and `U_D > U_T` property is proven by the controlled policy contract.

# Cumulative Static Audit — Stage S2.2 v0.5

Date: 2026-07-11

This is a package/source audit. It does not constitute MATLAB runtime validation.

```text
Stage S2.2 v0.5 cumulative static audit
==============================================================
[PASS] required package files — 31 MATLAB files, 14 reports
[PASS] four-argument public interface
[PASS] versioned result path
[PASS] validated v0.4 configuration constants retained — 129 legacy fields unchanged
[PASS] 12-scenario matrix — 7 frozen regressions + 5 lifecycle
[PASS] unsafe-home rejection is geometric
[PASS] exact validated v0.4 core preserved
[PASS] dispatcher preserves legacy core
[PASS] exact validated v0.4 ESKF preserved
[PASS] estimator routing is isolated
[PASS] all lifecycle states present — 19 states
[PASS] aid timestamps start invalid
[PASS] no optimistic aid freshness at construction
[PASS] freshness updated only after accepted filter update
[PASS] preflight uses accepted-age/eligibility/covariance gates
[PASS] preflight does not depend on instant packet flags
[PASS] airborne goal is not treated as landing footprint
[PASS] mission decisions do not read truth state
[PASS] landing uses contact-confirmed detector
[PASS] XY-loss trigger is one-shot
[PASS] XY-loss descent disables horizontal control
[PASS] normal lifecycle requires completion plus truth validation
[PASS] vertical reference and executed gates present
[PASS] RTL obstacle count tied to actual repair
[PASS] docked tabbed plot convention
[PASS] versioned v0.5 output conventions
[PASS] animation replays obstacle insertion time
[PASS] vertical metrics exposed in console/dashboard
[PASS] legacy airborne extensions are conditional
[PASS] landing confirmation cuts motor command immediately
[PASS] Monte Carlo covers all five lifecycle scenarios
[PASS] MATLAB file/function-name consistency — 50 .m files
[PASS] internal _S2_2 calls resolve — all calls resolve
[PASS] configuration fields complete — 171 fields
[PASS] scenario fields complete — 46 fields
[PASS] no absolute developer paths — portable paths
[PASS] source integrity — no NUL/conflict markers
[PASS] Python lifecycle matrix — 50/50 expected
[PASS] focused lifecycle mechanism audit — 9/9 PASS
[PASS] preserved v0.4 mechanism regressions
[PASS] MATLAB source sanity scan
[PASS] audit evidence report complete — 9/9 PASS
--------------------------------------------------------------
Result: 42/42 checks passed
CUMULATIVE STATIC AUDIT: PASS
```

## Preservation evidence

```text
v0.4 mission core normalized SHA-256:
9ac6336160b3ee68f032b6610fa6684a83854109ecddd3a854bd221a1353a483

v0.4 ESKF file SHA-256:
b4cb3f8a18b3936520cf91fd0215c3f0e28c230294b6a9c367319aaf2b742d25
```

The audit also compares current configuration assignments against `V0_4_CONFIG_BASELINE_S2_2.json`; 129 legacy assignments are unchanged.

## Executed supporting checks

- `python3 -m py_compile *.py`: PASS;
- lifecycle/grid matrix: 50/50 PASS;
- focused lifecycle mechanism audit: 9/9 PASS;
- v0.4 tracker/shaper regression: PASS;
- v0.4 rejoin/brake regression: PASS;
- lightweight MATLAB source sanity: 50 files checked, PASS.

## Boundary

The audit does not execute MATLAB. MATLAB runtime validation remains required for the coupled ESKF, controller, 6-DOF dynamics, complete lifecycle, plotting and animation.

# S2.5 v1.0.6 — Remaining Root Causes and Minimum Fix

## Evidence source

The v1.0.5-r1 MATLAB historical preflight achieved 3/5 PASS with hard safety PASS in all five cases. A two-case diagnostic then captured NAV_IMU_FAULT_VIO_OUTAGE seed 1 and both recovery invocations of PERCEPTION_RANGE_SPIKE seed 2.

## NAV_IMU_FAULT_VIO_OUTAGE seed 1

The mission reached the goal with hard safety PASS and successful CSE/SIE recovery, but `scanHoldCount=10` exceeded the inherited scenario `expectedMaxScanHolds=8`. This is an accounting mismatch introduced by the new recovery episode, not a collision/estimator/map-safety failure.

v1.0.6 preserves the inherited `scanHoldPass` exactly and reports it unchanged. It adds a separate S2.5 recovery-aware accounting result whose allowance is derived from the configured recovery semantics:

`(mapMaxNoProgressScans + 1) * informativeRecoveryRelocationCount`

This corresponds to the maximum configured no-progress scans plus the mandatory sensing hold at a newly reached recovery vantage. No configured scan threshold is modified.

## PERCEPTION_RANGE_SPIKE seed 2

The first v1.0.5 recovery succeeded. At the second recovery state the inflated map had no route and every recovery endpoint failed metric routing. Offline diagnosis found one critical persistent false-static voxel at approximately `(3.0, 4.7) m`; removing only that voxel restored the known-free route. The test-scenario truth geometry is used only in this offline diagnosis, never at runtime.

The frozen S2.3 mapper promotes static occupancy after two hit increments and increments its hit counter once per accepted ray. Thus two rays in one corrupted packet can supply both increments to the same voxel.

v1.0.6 adds a fault-agnostic pre-map integrity layer without modifying the frozen mapper:

1. at most one occupied endpoint update per voxel per perception packet across LiDAR and depth;
2. reject a claimed farther hit if an already-persistent static voxel lies earlier on the same ray;
3. rejected hit rays are removed, not converted into free rays.

This mirrors the scan-level behavior used by OctoMap, whose `insertPointCloud()` explicitly updates each voxel only once per point cloud and gives occupied updates precedence over free updates.

## Frozen invariants

No changes to S2.4-G/S2.3 parent bytes, estimator gates, packet age/pose gates, inflation radius, unknown-as-occupied behavior, route freshness, dynamic rejection, trajectory limits, stopping reserve, authority limits, no-progress scan configuration, map-extension bound, collision/geofence checks, or truth isolation.

## Validation status before MATLAB

Python/source gates required before release:

- exact first-recovery historical snapshot replay: 5/5;
- negative controls: 14/14;
- local perturbations: 10/10;
- recovery Monte Carlo: 40/40 safe, 0 unsafe;
- v1.0.6 root-cause backtest: PASS;
- per-packet integrity microtests: PASS;
- parallel 71-mission harness semantics: PASS;
- inherited S2.4-G local aggregate: PASS;
- frozen S2.4-G parent: 353/353 unchanged.

MATLAB runtime qualification remains required.

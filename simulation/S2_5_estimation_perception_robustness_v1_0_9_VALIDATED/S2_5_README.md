# VALIDATED FREEZE — S2.5 v1.0.9

**Status:** VALIDATED. Full MATLAB qualification: **71/71 unique coupled missions PASS** (5/5 baseline, 60/60 recoverable, 6/6 fail-safe); inherited S2.4-F PASS; frozen S2.4-G parent 353/353 byte-identical.

This tree is the immutable parent for S2.6. See `S2_5_VALIDATION_REPORT_v1_0_9.md` and `FROZEN_PARENT_CONTRACT_S2_5.md`.

---

# S2.5 Estimation + Perception Robustness — v1.0.9 Parallel Candidate

## v1.0.9 focused correction

The user-executed v1.0.8 internal NAV trace proved the remaining high-noise failure is a lifecycle dead state: `LIFECYCLE_REPLAN_BRAKE` was restored after `NAV_DEGRADED_HOLD` had already canceled its pending replan. v1.0.9 resumes through `PLAN_OUTBOUND`/`PLAN_RTL` and requires a fresh plan instead.

For the brief perception cases, all stale samples were already inside zero-translation `LIFECYCLE_REPLAN_BRAKE`; v1.0.9 records this as an S2.5 safe degraded-perception response without forcing a redundant state transition. The original `perceptionHoldCount` remains exposed, and prolonged perception loss still requires an actual perception hold. No safety threshold is relaxed. See `S2_5_V1_0_9_DEAD_STATE_AND_SAFE_RESPONSE_FIX.md`.

Required first MATLAB gate:

```matlab
setenv('S2_5_WORKERS','4');
report = run_validate_S2_5_v1_0_9_preflight();
```

---


This candidate is a direct overlay on the validated S2.4-G parent. The frozen S2.4-G and S2.3 trees remain byte-identical.

## Why v1.0.6

The v1.0.5 closed-loop historical preflight improved from 0/5 to 3/5 while keeping hard safety PASS in all five cases. A two-case diagnostic then isolated the two remaining failures:

1. **NAV_IMU_FAULT_VIO_OUTAGE seed 1** reached the goal safely, but the inherited `expectedMaxScanHolds=8` accounting gate was exceeded (`scanHoldCount=10`) because the new recovery architecture deliberately creates a fresh sensing context. v1.0.6 leaves the inherited scan-hold gate untouched and exposes a separate S2.5 recovery-aware accounting gate derived from the configured no-progress budget and the number of successful recovery relocations.
2. **PERCEPTION_RANGE_SPIKE seed 2** failed after a successful first recovery because a single persistent false-static voxel at approximately `(3.0, 4.7) m` closed the only known-free passage after inflation. The frozen S2.3 mapper increments `hitCount` per ray and promotes static occupancy after two observations, so multiple rays from one faulty packet can supply both increments. v1.0.6 adds a fault-agnostic pre-map packet integrity overlay: at most one occupied endpoint update per voxel per packet, plus rejection of claimed hits behind already-persistent static occupancy.

The map fix is consistent with OctoMap-style scan integration, where a voxel is updated only once per point cloud and occupied nodes take precedence over free updates. The frozen S2.3 mapper itself is not modified.

## Runtime changes relative to v1.0.5-r1

- Added `s2_5/perception/sanitize_perception_packet_S2_5.m`.
- `mission_lifecycle_manager_S2_5.m` supplies the sanitized packet to the frozen mapper and uncertainty replay, while retaining the original fault packet for S2.5 fault accounting.
- Added integrity counters to the summary.
- Added a separate S2.5 recovery-aware scan accounting result; the inherited `scanHoldPass` and inherited `mappingCompositePass` remain calculated and exposed unchanged.
- `run_S2_5_coupled.m` uses the S2.5 composite only for S2.5 case acceptance.
- CSE/SIE recovery code from v1.0.5-r1 is unchanged.

## Safety invariants unchanged

No change to estimator gates, map packet-age/pose gates, 0.602 m inflation, unknown-as-occupied semantics, route freshness, dynamic occupancy rejection, trajectory limits, stopping reserve, authority invalidation limit, configured no-progress scans, map extension bound, collision/geofence checks, or truth isolation.

## Required runtime gate

Run only the five historical cases first:

```matlab
setenv('S2_5_WORKERS','4');
report = run_validate_S2_5_v1_0_6_preflight();
```

Do not run the full 71-mission matrix unless this returns PASS 5/5.

MATLAB runtime is not available in the build environment, so no MATLAB v1.0.6 mission pass is claimed in this package.

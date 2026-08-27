# Known-boundary policy replay evidence

## Exact baseline replay

- Raw records: 1342
- Accepted / rejected: 1342 / 0
- Exact mapper arrays: PASS
- Production metrics exact match: PASS
- Original and replay false-free rate: 0.005818006327
- Original and replay occupied recall: 0.967482785004
- Idle-control counter mismatch is informational because the replay stream
  contains only accepted perception events.

## Counterfactual policy replay

Policy change: initialize only the known room/geofence boundary from `cfg.room`
as persistent prohibited space before inserting the identical packet stream.
No unknown obstacle truth was supplied to autonomy.

- Baseline exact arrays / metrics: PASS / PASS
- Candidate accepted / rejected: 1342 / 0
- Known boundary voxels registered: 3120
- Candidate false-free rate: 0.00174985
- Candidate occupied recall: 0.990054
- False-free voxels removed: 152
- Production requirements: PASS

## Coupled implementation decision

Integrate exactly this boundary initialization into
`init_probabilistic_map_S2_3.m`. Keep all S2.2 and existing S2.3 flight,
planning, estimation, control, trajectory, lifecycle and safety thresholds
unchanged. Coupled MATLAB validation remains required.

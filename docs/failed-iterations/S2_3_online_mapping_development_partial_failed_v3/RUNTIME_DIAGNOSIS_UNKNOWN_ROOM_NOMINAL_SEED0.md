# S2.3 nominal runtime diagnosis — seed 0

## Supplied MATLAB outcome

The first candidate entered `PREFLIGHT_REJECT` at 0.60 s before arming.

## Recorded-state root cause

The saved MAT trace shows at the rejection sample:

- lifecycle state changed from `PREFLIGHT` to `PREFLIGHT_REJECT` at 0.60 s;
- estimator aid, lane, covariance and update-count gates were all valid;
- launch footprint was observed free;
- `log.perceptionFresh` was true;
- the map had accepted 10 perception observations;
- the most recent LiDAR/depth observations were only 0.12/0.10 s old;
- `preflightCheck.perceptionOK` was nevertheless false.

The old preflight code tested whether the single asynchronous
`perceptionPacket` evaluated on the exact preflight control step contained a
new LiDAR scan or depth frame. At 0.60 s it did not, even though recent accepted
mapping observations were fresh. This repeated the previously documented
exact-time-step versus freshness failure mechanism.

## Cumulative correction

1. Preflight perception qualification now uses the age of the latest accepted
   LiDAR/depth map observation.
2. A minimum accepted-map-packet gate is applied.
3. Scheduled control steps with no perception event are counted as idle, not
   rejected mapping packets.
4. The map validator no longer labels the node-centred z=0 near-ground air
   layer as floor occupancy; floor contact remains independently validated by
   the plant and landing logic.

No S2.2 estimator, controller, planner, trajectory, landing or safety threshold
was changed.

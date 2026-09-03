# S2.5 v1.0.8 integration correction

No estimator, map-safety, trajectory, stopping, recovery, or timeout threshold is changed.

1. The end-of-step call to `simulate_perception_packet_S2_5` now receives `lifecycleState` in its context. v1.0.7 constructed state-triggered P3/P4/C1 faults correctly but dropped this field immediately before the actual packet-generation call, so those faults could never arm at runtime.
2. The v1.0.8 preflight and full validator restart any existing process pool before qualification. This prevents MATLAB workers created under an older extracted candidate from executing stale same-named functions after the client switches package paths.
3. The v1.0.7 replan-brake `stateEntryTime=t` correction is retained unchanged. With a fresh worker pool it is now actually exercised.

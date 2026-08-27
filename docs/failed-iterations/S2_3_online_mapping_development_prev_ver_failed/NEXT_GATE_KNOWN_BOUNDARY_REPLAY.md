# S2.3 next validation gate: known-boundary policy replay

The exact mapper replay reproduced every map array and all map metrics. Its
only reported counter mismatch was `noDataPackets`: the coupled loop counted
2,684 control cycles without a perception event, while the accepted-packet
replay intentionally contains only 1,342 accepted sensor records. The revised
exact replay treats this as informational bookkeeping, not mapper behaviour.

Before changing coupled flight code, run `replay_known_boundary_policy_S2_3`
on the captured packet stream. The candidate policy registers only the already
known room/geofence X/Y boundary voxels as persistent prohibited space. It does
not use unknown obstacle truth. The actual mapper and production validator are
then executed on the exact recorded packets and estimated poses.

Required gate:

- baseline arrays reproduce exactly;
- baseline metrics reproduce exactly;
- no replay packet is rejected;
- candidate false-free rate <= configured limit;
- candidate occupied recall >= configured limit;
- policy source uses only `cfg.room` and map coordinate arrays.

This package changes replay/backtest tooling only. Coupled mapping, planning,
control, estimation and lifecycle behaviour remain unchanged.

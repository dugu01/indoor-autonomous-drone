# S2.4 15-scenario contract matrix

All geometry contracts use the inherited 0.10 m XY grid and the final 0.602 m effective inflation.

| # | Contract | Required result |
|---:|---|---|
| 1 | Target visible through one frontier | Target-corridor Tier-1 viewpoint selected |
| 2 | Two unknown corridors | Target-relevant branch is selected over the non-target branch |
| 3 | Occluded doorway | Direct occluded candidates rejected; offset view selected |
| 4 | Misleading large frontier | High-gain unrelated Tier-3 candidates rejected |
| 5 | Stale free corridor | Target-first route check requests hold/rescan |
| 6 | Repeated failed frontier | Failed track enters cooldown; alternative selected |
| 7 | Frontier exhaustion | Bounded useful exploration terminates unreachable |
| 8 | Moving route crossing | All crossing candidates rejected; wait/replan |
| 9 | Moving object stops | Covariance/clearance increases conservatively |
| 10 | Temporarily occluded track | Track occupancy retained until timeout |
| 11 | Depth degradation | Depth contribution reduced; safe LiDAR continuation |
| 12 | LiDAR degradation | Information gain reweighted toward depth |
| 13 | Complete blackout | Stable frontier state and safe hold; no fabricated gain |
| 14 | Narrow unsafe access | Viewpoint rejected for occupied/unreachable support |
| 15 | Multiple active scans | Persistent track lineage remains bounded; goal route opens |

The executable catalogue is `python_tests/s2_4_ad_contract_backtest.py`. Every scenario is run in one process, and any failure makes the aggregate gate fail.

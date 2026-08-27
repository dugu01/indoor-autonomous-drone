# Known Limitations — S2.4-E Candidate

1. MATLAB execution was not available in the packaging environment. The coupled mission is source-audited but not claimed as numerically passed until run locally.
2. The first integration reuses the inherited `TRACK_OUTBOUND` and `SCAN_HOLD` states rather than introducing a final expanded state machine. This keeps the first change small and reversible.
3. Predictive dynamic-obstacle covariance is not yet connected to the live coupled route. The current coupled gate uses the S2.3 dynamic occupancy layer; predictive crossing remains covered by the designed offline contracts.
4. Exploration is enabled only for outbound target acquisition. RTL and landing remain unchanged S2.3 behaviour.
5. Viewpoint weights are still the initial hand-tuned shadow values. Sensitivity and multi-seed tuning are later release tasks.
6. Frontier failure/cooldown fields exist in the persistent data structure, but full execution-driven failure updates are not yet wired into the coupled lifecycle.
7. The first scenario is one move-scan-replan milestone. It is not the full 15-scenario coupled release.

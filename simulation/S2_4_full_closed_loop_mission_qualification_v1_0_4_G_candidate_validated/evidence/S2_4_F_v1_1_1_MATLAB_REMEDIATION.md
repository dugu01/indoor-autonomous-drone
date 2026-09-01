# Evidence: v1.1.1 coupled MATLAB failure and v1.1.2 remediation

## User MATLAB result
The v1.1.1 package preserved the complete S2.4-E MATLAB gate. The first F no-fault run then failed:
- no-fault E reference parity: FAIL;
- F1 revalidations/pass/lease/invalidation = 3 / 0 / 0 / 3;
- last safety reason = `KNOWN_FREE_STOP_INVALID`;
- F2-F14 fault cases mostly showed `faultInjections=0` because the exploration authority was revoked before the post-acceptance trigger could be reached.

## Root cause from exact source audit
`revalidate_active_exploration_request_S2_4_F.m` called frozen
`validate_known_free_stop_S2_3` on every `TRACK_OUTBOUND` execution sample.

The frozen S2.3 helper is used in the inherited planning path. It combines:
1. the conservative stopping-distance formula;
2. a minimum route-length requirement; and
3. terminal endpoint footprint clearance through `landing_zone_clear_S2_2`.

The inherited live S2.3 route-change supervisor does **not** call this complete planning-time terminal gate every cycle. It watches newly blocked future-route cells and invokes the existing repair/brake/replan machinery.

Therefore v1.1.1 added a stronger but semantically misplaced terminal check during intermediate execution. That could revoke an unchanged exploration authority even when the remaining route, viewpoint and local hold support were still executable.

## v1.1.2 correction
- Frozen S2.3 files remain byte-identical.
- Planning still uses `validate_known_free_stop_S2_3` unchanged.
- Runtime F no longer re-runs that planning-time terminal gate.
- Runtime stopping reserve uses the same conservative stop model:
  `d_stop = v^2/(2*a_decel) + v*t_delay + d_margin`.
- If remaining accepted-route arc length is at least `d_stop`, the stop reserve is valid on the already-inflated execution grid.
- If remaining route is shorter, the shortfall must be contained in a known-free terminal overrun disk around the already validated viewpoint with valid hold support.
- No extra vehicle-radius inflation is added to the already-inflated execution grid.

## Backtest strengthened
The previous Python semantic model omitted stopping support; that was inadequate and is corrected.
The new source-faithful backtest adds:
- `R1`: an off-route cell within the old terminal footprint causes the frozen planning-time gate to fail, while the unchanged runtime route has ample stopping reserve and must remain valid;
- `STOP`: a high-speed short remaining route with blocked terminal overrun support must still be rejected.

Both regressions PASS locally together with F1-F14, scheduler semantics, A-E regression, truth isolation and frozen-parent integrity.

## Runtime status
MATLAB/Octave are not available in the packaging environment. v1.1.2 coupled F qualification therefore remains pending and is not claimed as PASS.

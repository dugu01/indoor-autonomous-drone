# Evidence: v1.1.0 coupled MATLAB failures and v1.1.1 remediation

Source of failure evidence: user's MATLAB execution of `run_validate_S2_4_F_all()` on the v1.1.0
candidate. The inherited S2.4-E gate and deterministic F helper contracts passed. Runtime results:

- F1 no-fault E parity: FAIL
- F2: PASS
- F3: PASS
- F4: FAIL (`injected=1`, `detected=0`)
- F5: PASS
- F6: FAIL (`injected=1`, `detected=0`)
- F7: FAIL (`injected=1`, `detected=0`)
- F8: FAIL (`injected=1`, `detected=0`)
- F9: PASS
- F10: FAIL (`detected=1`, `unknown=1`)
- F11: FAIL (`injected=0`)
- F13: FAIL
- F14: FAIL (`injected=1`, `detected=0`)

## Root-cause classification
1. **Fault scheduling defect:** v1.1.0 anchored injection timing to the first accepted request, not
   to the currently executing authority. This explains injected-but-not-detected cases and made
   F11 able to trigger before a traversed prefix existed.
2. **F10 fixture defect:** the validation hook marked `startXY` itself occupied, capable of
   manufacturing an unsafe-reference/unknown-commitment count instead of isolating retreat.
3. **F1 lease/terminal integration defect:** extending E's 1.0 s TTL was not acceptable baseline
   parity. Also, the inherited S2.3 path-length stop test is a planning-time terminal reserve check
   and can become false after the vehicle has nearly consumed the route during arrival confirmation.
4. **F13/F14 generation semantics:** temporary/repeated invalidations need explicit authority-
   generation semantics rather than a single first-acceptance timer.

## v1.1.1 changes
- authority/progress-relative injection only in active `TRACK_OUTBOUND`;
- one injection per authority generation, with F14 repeating across generations;
- F11 waits for >=45% progress and perturbs an offset traversed-region cell;
- F10 blocks forward route + retreat-goal neighbors but not start/current cell;
- exact E TTL preserved; successful current checks renew a rolling execution lease;
- already expired authority rejected before any renewal;
- terminal known-free stop disk fallback only inside inherited arrival envelope;
- F14 qualification requires bounded repeated invalidations and `GOAL_UNREACHABLE`, not timeout.

No gate was weakened to convert a failure into PASS. Coupled MATLAB v1.1.1 remains pending.

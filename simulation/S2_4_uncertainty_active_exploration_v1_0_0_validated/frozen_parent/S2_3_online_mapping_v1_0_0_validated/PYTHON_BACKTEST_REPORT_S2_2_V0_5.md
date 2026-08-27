# Python and Static Evidence — Stage S2.2 v0.5

## 1. Lifecycle/grid scenario matrix

Executed runs: **50**  
Result: **50/50 PASS**

| Scenario | Pass | Runs | Worst geometric centre clearance [m] |
|---|---:|---:|---:|
| `full_mission_nominal` | 10 | 10 | 0.256 |
| `rtl_obstacle_replan` | 10 | 10 | 0.256 |
| `alternate_landing_zone` | 10 | 10 | 0.256 |
| `preflight_reject_unsafe_home` | 10 | 10 | N/A — rejected before flight |
| `xy_loss_emergency_land` | 10 | 10 | 0.402 |

This is a high-level lifecycle and grid-planning regression. It does not reproduce the MATLAB four-lane ESKF or 6-DOF plant.

## 2. Focused mechanism audit

Executed tests: **9**  
Result: **9/9 PASS**

The focused tests cover:

1. accepted-aid freshness starts invalid;
2. asynchronous packet arrival is not treated as aid health;
3. XY-loss emergency transition is one-shot;
4. XY-loss emergency descent is vertical-only;
5. landing requires persistent contact confirmation;
6. estimated-state arrival requires dwell;
7. vertical seventh-order profile respects derivative limits;
8. mission decision section has no direct truth-state reads;
9. animation uses obstacle insertion timestamps.

Evidence files:

```text
AUDIT_BACKTEST_RESULTS_S2_2_V0_5.json
AUDIT_BACKTEST_REPORT_S2_2_V0_5.txt
```

## 3. Preserved v0.4 mechanism regressions

Both focused v0.4 regressions were rerun:

- tracker promotion and velocity-command shaping: PASS;
- rejoin speed and brake-before-replan eligibility: PASS.

These are mechanism regressions, not complete MATLAB scenario execution.

## 4. Cumulative source audit

`audit_S2_2_v0_5.py` reruns the Python tests and checks package/source invariants. Current result:

```text
42/42 PASS
CUMULATIVE STATIC AUDIT: PASS
```

Important checks include exact preservation of the validated v0.4 mission core and ESKF, legacy configuration-value retention, lifecycle/legacy estimator routing, field/function resolution and source hygiene.

## MATLAB boundary

Only MATLAB execution can validate the complete coupled lifecycle, ESKF, controller, 6-DOF dynamics, plotting and animation. No MATLAB v0.5 pass is claimed in this report.

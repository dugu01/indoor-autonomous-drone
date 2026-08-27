# Consolidated Audit Notes — Stage S2.2 v0.5

This file records why the earlier v0.5 candidate and standalone preflight patches were superseded.

## Defects found during full-package review

1. The airborne mission goal was incorrectly tested with a landing-footprint rule.
2. Preflight used exact-step asynchronous sensor packet flags instead of accepted aid freshness.
3. Aid timestamps were initially marked fresh before any update had been accepted.
4. Preflight did not explicitly require active-lane eligibility, covariance bounds and a minimum accepted-update count.
5. Lifecycle mission decisions contained direct truth-state completion checks.
6. Landing did not use a persistent contact-confirmed onboard-style detector.
7. Recurring RTL requests could repeatedly reset the emergency-hold timer.
8. XY-loss descent could retain horizontal control toward stale coordinates.
9. Vertical executed kinematic limits were not exposed as independent validation gates.
10. Animation could show the final obstacle map before obstacles had actually appeared.
11. The initial freshness correction modified the ESKF shared by validated v0.4 regressions.
12. The Monte Carlo validator omitted the preflight-rejection lifecycle case.
13. Landing transition retained one extra integration-step motor command before disarm.

## Consolidated corrections

- isolated the lifecycle ESKF from the exact validated v0.4 ESKF;
- made accepted-update timestamps authoritative;
- strengthened preflight gates and diagnostics;
- made mission decisions estimator/simulated-signal based;
- added confirmation dwell for takeoff, arrivals and landing contact;
- made XY-loss emergency descent vertical-only and one-shot;
- added vertical reference/executed kinematic gates;
- replayed obstacle history by timestamp;
- added all five lifecycle cases to Monte Carlo validation;
- cut motor command immediately on transition to disarm;
- added explicit provenance hashes and legacy-config comparison.

## Executed audit evidence

```text
lifecycle/grid matrix       50/50 PASS
focused mechanism audit      9/9 PASS
v0.4 patch mechanisms        PASS
MATLAB source sanity         PASS
cumulative static audit     42/42 PASS
```

## Honest boundary

The consolidated audit can detect source, interface, routing, configuration and high-level logic defects. It cannot execute MATLAB in this environment. The user’s MATLAB run is authoritative for coupled ESKF, 6-DOF controller, lifecycle transitions, plots and animations.

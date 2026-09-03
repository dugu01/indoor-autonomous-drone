# S2.5 v1.0.6 Parallel Qualification

- Default process workers: 4; bounded maximum: 8.
- Historical fail-fast: 5 exact historical recoverable cases.
- If and only if historical 5/5 passes: inherited S2.4-F regression, 5 no-fault baselines, 60 recoverable cases, 6 fail-safe cases.
- Unique coupled missions: 71.
- Historical results are cached and reused in the 60-case recoverable matrix.
- Workers return compact summaries; normal standalone runner can still save full artifacts.

Start with:

```matlab
setenv('S2_5_WORKERS','4');
report = run_validate_S2_5_v1_0_6_preflight();
```

Then, only after PASS 5/5:

```matlab
gate = run_validate_S2_5_all();
```

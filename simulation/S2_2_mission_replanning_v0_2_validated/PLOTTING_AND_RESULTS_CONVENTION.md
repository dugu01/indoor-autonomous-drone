# Plotting and Results Convention

Locked for Stage S2.2 and all subsequent project stages.

1. Related plots for one scenario are grouped in one `tiledlayout` dashboard.
2. Scenario dashboards use `WindowStyle='docked'`, so MATLAB displays them as tabs in one Figures window.
3. Rerunning the same scenario replaces its previous tab rather than creating duplicates.
4. Each dashboard is saved as both PNG and MATLAB FIG when supported.
5. Results use a stage/version hierarchy:

```text
simulation/results/
└── S2_2_mission_replanning/
    └── v0_2/
        ├── incremental_static_insert/seed_000/
        ├── dynamic_crossing_yield/seed_000/
        ├── dynamic_blocker_becomes_static/seed_000/
        ├── sensor_dropout_recover/seed_000/
        ├── sensor_dropout_failsafe/seed_000/
        ├── two_dynamic_crossings/seed_000/
        └── validation/
```

6. Future versions use separate folders such as `v0_3`, `v0_4`, and final `v1_0_0`; earlier results are never overwritten.

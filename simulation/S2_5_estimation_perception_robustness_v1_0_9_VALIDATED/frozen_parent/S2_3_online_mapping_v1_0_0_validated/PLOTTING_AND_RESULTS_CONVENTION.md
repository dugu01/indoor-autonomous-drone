# Plotting and Results Convention

- Each validation scenario opens as one docked MATLAB figure tab.
- Each tab uses one `tiledlayout` dashboard; related panels are not opened as separate windows.
- Re-running the same scenario closes and replaces its previous tab.
- Every dashboard is saved as PNG and editable FIG.
- Each version writes to a separate results tree.

Stage S2.2 v0.4 path:

```text
simulation/results/S2_2_mission_replanning/v0_4/<scenario>/seed_000/
```

Validation report path:

```text
simulation/results/S2_2_mission_replanning/v0_4/validation/
```

Generated `.mat`, `.fig`, `.png` and `.mp4` files remain outside the source folder.

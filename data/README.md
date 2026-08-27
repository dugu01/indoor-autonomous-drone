# Data Directory

Contains datasets, logs, recordings, annotations, and experiment outputs used in this project.

Examples:
- Sensor recordings
- Training/validation datasets
- Ground truth annotations
- Evaluation logs
- Processed experiment data

Large datasets should not be committed directly to GitHub.  
Instead, provide download links or external storage references.

## Layout

```
data/results/
  S1_dynamics_pid/               PID / dynamics step-response outputs
  S2_visual_slam/                Stage S2 6-DoF LiDAR-SLAM + ESKF trials (nominal / stress)
  S2_1_robust_multilane/         Stage S2.1 multi-lane estimator resilience trials
  S2_2_mission_replanning/       Stage S2.2 autonomous replanning scenario + multi-seed sweeps
  S2_2_mission_replanning_v0_2/  Early S2.2 scenario snapshot (retained for traceability)
  S2_3_online_mapping/           Stage S2.3 probabilistic online-mapping trial dashboards
```

Each trial folder holds PNG report tabs, a text summary, and (when requested) an
MP4 flight animation.

## Excluded artifacts

To keep the repository within GitHub limits, the following regenerable binary
artifacts are **not** tracked (see `.gitignore`):

| Pattern | What it is | How to regenerate |
| ------- | ---------- | ----------------- |
| `*.mat` | Raw per-seed trial data (~5 GB) | Re-run the stage entry point in `simulation/<stage>/` in MATLAB |
| `*.fig` | MATLAB dashboard figures | Re-run the stage `plot_*` / dashboard script; PNG exports are kept |
| `*.zip` | Packaged stage/result snapshots | Re-zip the corresponding stage directory |

The committed PNG dashboards and text summaries are sufficient to review every
validated result without the raw `.mat` files.


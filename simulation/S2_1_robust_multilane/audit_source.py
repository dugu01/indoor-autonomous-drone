#!/usr/bin/env python3
from pathlib import Path
root=Path(__file__).resolve().parent
s=(root/"run_S2_lidar_slam.m").read_text()
checks={
 "locked_four_argument_interface":"function results = run_S2_lidar_slam(seed, runStress, makePlots, makeAnimation)" in s,
 "external_dashboard":"plot_S2_dashboard(" in s and "plot_S2_results(" not in s,
 "conditional_animation":"if makeAnimation,animationFile=animate_S2_flight" in s,
 "four_observable_lanes":all(x in s for x in ["Primary IMU + all aids","Backup IMU + all aids","Primary IMU + VIO","Backup IMU + LiDAR"]),
 "normalized_nis":"nis/gate" in s,
 "conditional_gravity":"gravityAccelNormGate" in s and "gravityMaxGyro" in s,
 "output_blending":"blendedOutput_S2" in s,
 "path_validation":"validateReferencePath_S2" in s,
 "separate_global_graph":"buildGlobalPoseGraph_S2" in s,
 "s2_1_results_folder":"S2_1_robust_multilane" in s,
}
for k,v in checks.items(): print(f"{k:32s}: {'PASS' if v else 'FAIL'}")
raise SystemExit(0 if all(checks.values()) else 1)

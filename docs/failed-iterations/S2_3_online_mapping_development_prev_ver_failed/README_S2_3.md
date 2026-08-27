# Stage S2.3 v1.0.0 candidate

Perception-driven probabilistic mapping and safe receding-horizon navigation in
unknown indoor environments.

## Status

This is one cumulative MATLAB development candidate derived from frozen S2.2
v1.0.0. Two user-run nominal MATLAB trials have been investigated from their
complete saved MAT data. The present cumulative corrections pass source,
truth-isolation, immutability, recorded-trace and 15 independent mechanism
checks, but the corrected package has not yet been rerun in MATLAB.

It must not be described as MATLAB validated or renamed as the final S2.3
v1.0.0 release.

## First rerun command

```matlab
restoredefaultpath;
rehash toolboxcache;
cd('/path/to/simulation/S2_3_online_mapping_development');
addpath(pwd,'-begin');
clear functions; clear classes; close all; clc;
rehash path;

which run_S2_3_online_mapping -all
which mission_lifecycle_manager_S2_3 -all
which update_probabilistic_map_S2_3 -all
which changed_cells_affect_route_S2_3 -all
which generate_strict_trajectory_S2_3 -all

results = run_S2_3_online_mapping(0,'unknown_room_nominal',false,false);
```

Do not run the full scenario or multi-seed matrices until the nominal console
output and saved MAT file have been inspected.

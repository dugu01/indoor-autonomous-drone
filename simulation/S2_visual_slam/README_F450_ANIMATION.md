# Stage S2 F450 animation integration

## Selected CAD model

The animation uses the assembly STL parts extracted from `dji-f450-5.snapshot.1.zip`.
This model was selected because it contains separately positioned frame plates, four
arms, four motors and four propellers. The measured combined model envelope is about
0.517 x 0.517 x 0.100 m, and the motor centres reproduce the 450 mm diagonal F450
wheelbase. Separate propeller meshes allow individual rotor animation.

The other uploads were not selected:

- `f450 quadcopter frame top body part.stl` contains only the 108 mm top plate.
- `f450-drone-framestl.stl` is a single very high-poly mesh with a direct imported
  envelope near 1.69 x 1.42 m, so it is not correctly scaled for direct F450 use.
- `f450-quadcopter-drone-frame-1.snapshot.8.zip` contains STEP/SolidWorks parts but
  no ready, positioned STL assembly; it would require CAD assembly conversion.

## Motor and propeller parameters

- Motor KV: 920 rpm/V
- User voltage range: 7-12 V
- No-load speed range: 6,440-11,040 rpm
- Nominal animation voltage: 11.1 V
- Assumed loaded-speed factor: 0.85
- T/W: 2.40
- Estimated hover speed used when ESC telemetry is unavailable: about 5,600 rpm
- Propeller diameter: 0.254 m (10 inch)

The displayed RPM is a physically scaled visualization estimate, not a replacement
for ESC telemetry. If `est.motorRPM` is later logged as an N-by-4 matrix, the
animation automatically uses those four measured/estimated motor speeds.

## D435i visualization

The animation draws a body-fixed camera frustum using:

- horizontal FOV: 87 degrees
- vertical FOV: 58 degrees
- display range: 1.35 m

The visual frustum points along body +x. Replace the animation and estimator camera
extrinsics with the measured mounting transform before hardware flight.

## Collision envelope and geofence

The supplied 25.4 cm value is interpreted as the 10-inch propeller **diameter**, not
the full drone collision radius. With a 0.225 m centre-to-motor arm radius:

`collision radius = 0.225 + 0.254/2 = 0.352 m`

The centre-of-mass geofence additionally includes:

- 0.10 m navigation-error allowance
- 0.05 m control/braking allowance

Therefore the horizontal room/obstacle inset is about 0.502 m. The green box is the
safe room-centre geofence; red dotted regions are inflated obstacle keep-out zones.

The old figure-eight passed through the second obstacle. It has been replaced by a
collision-checked approximately 19.7 m figure-eight. A preflight path validator now
aborts the simulation if a reference path violates the geofence or inflated obstacle
regions.

## Folder layout

Place the folder as:

```
indoor-autonomous-drone/
└── simulation/
    └── S2_visual_slam/
        ├── run_S2_lidar_slam.m
        ├── plot_S2_results.m
        ├── animate_S2_flight.m
        └── assets/
            └── F450/
                ├── base_top.stl
                ├── base_bottom.stl
                ├── arm_1.stl ... arm_4.stl
                ├── motor_1.stl ... motor_4.stl
                └── prop_1.stl ... prop_4.stl
```

## Run

```matlab
clear functions; clear classes; close all; clc;
results = run_S2_lidar_slam(0,true,true,true);
```

The first STL run creates `assets/F450/f450_animation_mesh_cache.mat` after reducing
the high-resolution CAD meshes for real-time display. Later runs load the cache.

For faster animation without the STL model, call `animate_S2_flight` directly with
`'UseSTL',false`. Static report plots remain in one tabbed MATLAB dashboard.

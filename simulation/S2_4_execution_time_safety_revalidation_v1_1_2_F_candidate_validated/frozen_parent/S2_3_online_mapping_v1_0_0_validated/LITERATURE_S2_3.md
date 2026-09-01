# Stage S2.3 literature traceability

## Probabilistic occupancy mapping
Hornung et al., *OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees*, Autonomous Robots, 2013.

Adopted: log-odds occupancy, explicit free/occupied/unknown space, ray-based free-space updates.

Project adaptation: bounded layered array at 0.10 m XY and 0.20 m Z resolution for deterministic base-MATLAB regression. This is not a full OctoMap implementation.

## Incremental replanning
Koenig and Likhachev, *D* Lite*, AAAI 2002.

Adopted: repair graph costs after occupancy changes. The existing validated S2.2 D* Lite implementation is retained.

## Safe navigation in unknown space
Tordesillas et al., *FASTER: Fast and Safe Trajectory Planner for Navigation in Unknown Environments*, IEEE T-RO.

Adopted: retain a stopping solution terminating in known free space.

Project adaptation: S2.3 does not optimize through unknown space and does not reproduce the FASTER MIQP. It uses conservative goal-directed known-free frontier segments.

## Incremental volumetric mapping for MAV planning
Oleynikova et al., *Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning*, IROS 2017.

Adopted: separate local mapping representation from the planner-facing map. S2.3 uses occupancy rather than a full ESDF.

## Project-specific integration
- Four-lane fault-tolerant ESKF from S2.1/S2.2.
- Perception-derived layered occupancy feeding the frozen D* Lite and trajectory cores.
- Map-versioned trajectory revalidation.
- Goal-directed scan-and-advance lifecycle.
- Truth-isolation source audit and independent map validation.

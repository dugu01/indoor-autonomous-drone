# Source Baseline — Stage S2.2 v0.5

## Parent archive

The package was built from the user-supplied archive:

```text
S2_2_mission_replanning_v0_4_validated.zip
SHA-256: fb252c35becaa256385f4a4ef946e806f05bfb2da9902ef6ba800e735441de69
```

The archive was supplied after all seven v0.4 MATLAB scenarios passed.

## Exact v0.4 runtime isolation

The uploaded v0.4 mission-manager executable body is retained in:

```text
mission_manager_v0_4_core_S2_2.m
normalized executable-body SHA-256:
9ac6336160b3ee68f032b6610fa6684a83854109ecddd3a854bd221a1353a483
```

The exact uploaded v0.4 estimator is restored as:

```text
multi_lane_eskf_S2_2.m
file SHA-256:
b4cb3f8a18b3936520cf91fd0215c3f0e28c230294b6a9c367319aaf2b742d25
```

Legacy scenarios call those two preserved components. Lifecycle scenarios call the separate:

```text
mission_lifecycle_manager_S2_2.m
multi_lane_eskf_lifecycle_S2_2.m
```

This prevents lifecycle preflight changes from silently altering the validated v0.4 estimator startup behavior.

## Configuration preservation

`V0_4_CONFIG_BASELINE_S2_2.json` records the normalized right-hand sides of all v0.4 configuration assignments. The cumulative audit verifies that all 129 legacy assignments remain unchanged, excluding only:

```text
version
methodName
```

New v0.5 fields are appended for lifecycle operation.

## Backward-compatible shared extensions

- `init_quadrotor_state_S2_2.m` adds an optional start altitude; the original two-argument call remains the airborne v0.4 path.
- `quadrotor_dynamics_S2_2.m` adds ground contact only when `groundHeight_m` is reached.
- `geometric_controller_S2_2.m` changes horizontal control only when the lifecycle reference explicitly sets `horizontalControlEnabled=false`.
- `simulate_sensor_packet_S2_2.m` adds a ground-contact signal; existing sensor measurements are unchanged.

The seven-scenario v0.4 runtime regression remains mandatory because static preservation checks cannot replace MATLAB execution.

## Validation status

- v0.4 parent: MATLAB runtime validated by the user.
- v0.5 Python/static evidence: executed and passing.
- v0.5 MATLAB lifecycle: candidate pending user runtime validation.

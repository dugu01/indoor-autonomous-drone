# S2.5 architecture

## Frozen boundary

The complete S2.4-G v1.0.4 validated tree is the immutable parent. `s2_5/evidence/S2_4_G_VALIDATED_SHA256SUMS.txt` contains its 353-file inventory. `audit_S2_4_G_parent_immutability.py` must pass before and after S2.5 qualification.

No parent estimator, planner, controller, mapper, active-exploration policy, execution-safety revalidator or frozen S2.3 file is edited.

## Additive S2.5 overlay

S2.5 adds:

1. `simulate_sensor_packet_S2_5.m` — same nominal S2.2 sensor equations plus deterministic qualification faults.
2. `simulate_perception_packet_S2_5.m` — same nominal S2.3 raw-ray generation plus dropout/staleness/range-spike faults.
3. `mission_lifecycle_manager_S2_5.m` — source-identical S2.4-G manager except it calls the two S2.5 packet generators and reports fault/rejection counters.
4. `scenario_S2_5.m` — deterministic fault contracts.
5. qualification/static/backtest scripts.

The autonomy stack consumes only the modified *synthetic packet*. Environment truth remains confined to simulation generation and independent validation, exactly as in S2.4.

## Existing robustness mechanisms being qualified

The inherited four-lane ESKF already provides:

- VIO/LiDAR/range/barometer innovation gating;
- aid-age and covariance eligibility checks;
- two independent IMU groups;
- innovation-based IMU fault attribution;
- bounded lane switching and output blending;
- controlled degradation when horizontal aiding becomes unavailable.

The inherited mapping/lifecycle layer already provides:

- estimated-pose covariance rejection for map insertion;
- packet-age rejection;
- probabilistic occupied/free/unknown representation;
- fail-closed unknown-space planning;
- perception freshness hold;
- controlled emergency landing after persistent total obstacle-perception loss.

S2.5 qualifies these mechanisms under faults rather than replacing them.

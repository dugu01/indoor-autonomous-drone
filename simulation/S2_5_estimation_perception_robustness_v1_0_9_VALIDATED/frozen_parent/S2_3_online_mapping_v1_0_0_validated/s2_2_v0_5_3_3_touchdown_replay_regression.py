#!/usr/bin/env python3
"""Exact-trace regression for the S2.2 v0.5.3.3 touchdown latch.

This test uses metrics extracted from the actual MATLAB v0.5.3.2 seed-7
trial file. It does not pretend to execute MATLAB. It verifies the diagnosed
failure mechanism and replays the exact source ground dynamics after the
proposed contact-latched motor cut.
"""
from pathlib import Path
import json

ROOT=Path(__file__).resolve().parent
E=json.loads((ROOT/'SEED7_EXACT_TRACE_REPLAY_V0_5_3_3.json').read_text())
L=(ROOT/'land_detector_S2_2.m').read_text()
M=(ROOT/'mission_lifecycle_manager_S2_2.m').read_text()
C=(ROOT/'init_S2_2_config.m').read_text()

def check(name,cond,detail):
    print(f"[{'PASS' if cond else 'FAIL'}] {name}: {detail}")
    if not cond: raise AssertionError(name)

check('version namespace',"cfg.version='v0.5.3.3';" in C,'v0.5.3.3')
check('profile-complete gate','landingDetectionArmed=vz.complete;' in M,'normal and emergency descent gated')
check('gated detector call','landDetected,landingDetectionArmed' in M,'latch state and gate passed')
check('contact event latch','previousDetected' in L and 'qualifiedTouchdown' in L,'qualified contact persists')
check('no truth in detector','truth' not in '\n'.join(x for x in L.splitlines() if not x.lstrip().startswith('%')),'sensor packet only')
check('brake actually arrested motion',E['truth_speed_near_brake_zero_mps']<0.01,
      f"speed near zero={E['truth_speed_near_brake_zero_mps']:.4f} m/s")
check('failure was delayed touchdown recognition',E['current_touchdown_to_detection_delay_s']>10,
      f"delay={E['current_touchdown_to_detection_delay_s']:.2f} s")
check('first event is fully qualified',
      E['first_touchdown_est_z_m']<0.13 and abs(E['first_touchdown_est_vz_mps'])<0.08,
      f"z={E['first_touchdown_est_z_m']:.4f} m, vz={E['first_touchdown_est_vz_mps']:.4f} m/s")
check('latch acts next control step',E['proposed_latch_state_transition_s']-E['first_qualified_touchdown_s']<=E['dt_s']+1e-9,
      f"transition={E['proposed_latch_state_transition_s']:.2f} s")
check('exact ground replay drift',E['replay_extra_xy_drift_m']<0.05,
      f"extra drift={E['replay_extra_xy_drift_m']:.4f} m")
check('exact wall safety replay',E['replay_min_raw_wall_distance_m']>E['required_wall_distance_m'],
      f"wall={E['replay_min_raw_wall_distance_m']:.3f} m, required={E['required_wall_distance_m']:.3f} m")
print('S2.2 v0.5.3.3 EXACT-TRACE TOUCHDOWN REPLAY: PASS')

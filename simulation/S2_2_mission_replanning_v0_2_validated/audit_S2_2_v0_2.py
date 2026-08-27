#!/usr/bin/env python3
from pathlib import Path
root=Path(__file__).resolve().parent
checks={
 'main_interface': 'function results = run_S2_2_mission_replanning(seed,scenarioName,makePlots,makeAnimation)' in (root/'run_S2_2_mission_replanning.m').read_text(),
 'dstar_lite': "dstar_lite_S2_2('repair'" in (root/'mission_manager_S2_2.m').read_text(),
 'velocity_obstacle_filter': 'velocity_obstacle_filter_S2_2' in (root/'mission_manager_S2_2.m').read_text(),
 'alpha_beta_tracker': 'alpha_beta_track_S2_2' in (root/'mission_manager_S2_2.m').read_text(),
 'no_data_hold': 'noDataStopTimeout_s' in (root/'mission_manager_S2_2.m').read_text(),
 'dynamic_promotion': 'stoppedPersistence_s' in (root/'mission_manager_S2_2.m').read_text(),
 'six_scenarios': all(x in (root/'validate_S2_2.m').read_text() for x in ['incremental_static_insert','dynamic_crossing_yield','dynamic_blocker_becomes_static','sensor_dropout_recover','sensor_dropout_failsafe','two_dynamic_crossings']),
 'literature_file': (root/'LITERATURE_S2_2.md').exists(),
}
for k,v in checks.items(): print(f'{k:30s}: {"PASS" if v else "FAIL"}')
raise SystemExit(0 if all(checks.values()) else 1)

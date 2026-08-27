from pathlib import Path
root=Path(__file__).resolve().parent
required=['run_S2_2_mission_replanning.m','mission_manager_S2_2.m','generate_min_snap_trajectory_S2_2.m',
'sample_min_snap_trajectory_S2_2.m','sample_min_snap_state_S2_2.m','track_smooth_trajectory_S2_2.m',
'jerk_limited_step_S2_2.m','plot_S2_2_dashboard.m','validate_S2_2.m','LITERATURE_S2_2.md']
missing=[x for x in required if not (root/x).exists()]
assert not missing,missing
text=(root/'run_S2_2_mission_replanning.m').read_text()
assert 'function results = run_S2_2_mission_replanning(seed,scenarioName,makePlots,makeAnimation)' in text
assert "'v0.3'" in (root/'init_S2_2_config.m').read_text()
print('S2.2 v0.3 static package audit: PASS')

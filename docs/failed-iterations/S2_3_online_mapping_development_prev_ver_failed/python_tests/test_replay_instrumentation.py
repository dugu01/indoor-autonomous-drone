from pathlib import Path
root=Path(__file__).resolve().parents[1]
manager=(root/'mission_lifecycle_manager_S2_3.m').read_text()
runner=(root/'run_S2_3_online_mapping.m').read_text()
replay=(root/'replay_perception_log_S2_3.m').read_text()
checks={
 'capture_schema': "S2_3_RAW_RAYS_POSE_V1" in manager,
 'capture_packet': "'packet',perceptionPacket" in manager,
 'capture_pose': "'pose',replayPose" in manager,
 'capture_call_time': "'callTime',t" in manager and "'callTime',0" in manager,
 'summary_count': 'perceptionReplayCount' in manager and 'Replay records captured' in runner,
 'replay_actual_mapper': 'update_probabilistic_map_S2_3' in replay,
 'replay_production_validator': 'validate_map_against_truth_S2_3' in replay,
}
for k,v in checks.items(): print(f'{k:32s}: {"PASS" if v else "FAIL"}')
raise SystemExit(0 if all(checks.values()) else 1)

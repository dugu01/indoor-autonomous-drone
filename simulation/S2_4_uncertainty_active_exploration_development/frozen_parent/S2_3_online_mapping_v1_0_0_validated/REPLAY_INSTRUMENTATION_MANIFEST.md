# Replay instrumentation source changes

Only these existing MATLAB files were modified:
- mission_lifecycle_manager_S2_3.m: capture accepted packet, exact insertion pose/time, and replay count.
- run_S2_3_online_mapping.m: print/save replay count.

New files:
- replay_perception_log_S2_3.m
- REPLAY_FIRST_NEXT_STEP.md
- python_tests/test_replay_instrumentation.py

Production mapping/planning/control code is otherwise unchanged.

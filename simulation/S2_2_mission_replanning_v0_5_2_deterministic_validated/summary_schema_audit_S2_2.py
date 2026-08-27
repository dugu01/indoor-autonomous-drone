#!/usr/bin/env python3
from pathlib import Path
import re, sys
root=Path(__file__).resolve().parent
run=(root/'run_S2_2_mission_replanning.m').read_text()
required=[
    "isLifecycle = isfield(s,'lifecycleEnabled') && s.lifecycleEnabled;",
    'if isLifecycle',
    's.maxEstimatorPositionErrorObservable_m',
    's.estimatorPositionErrorAtFailsafeTrigger_m',
    's.maxEstimatorPositionErrorPostLoss_m',
    's.estimatorFailsafeMetric_m',
    's.navigationDegradedHoldCount',
    's.maxEmergencyHorizontalDrift_m',
]
missing=[x for x in required if x not in run]
# Ensure lifecycle-only block is between guard and its matching local end,
# before common reference metrics resume.
a=run.find("isLifecycle = isfield(s,'lifecycleEnabled') && s.lifecycleEnabled;")
b=run.find("fprintf(f,'Reference XY v/a/j",a)
block=run[a:b]
fields=required[2:]
contained=all(x in block for x in fields) and 'if isLifecycle' in block
if missing or not contained:
    print('SUMMARY SCHEMA AUDIT: FAIL')
    print('missing:',missing)
    sys.exit(1)
print('SUMMARY SCHEMA AUDIT: PASS')
print('Lifecycle-only estimator-loss fields are guarded in write_summary.')

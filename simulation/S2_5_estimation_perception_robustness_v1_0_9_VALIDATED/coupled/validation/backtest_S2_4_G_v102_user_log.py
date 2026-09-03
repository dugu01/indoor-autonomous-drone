#!/usr/bin/env python3
"""Regression against the user's actual MATLAB G v1.0.2 console log.

The four residual failures must be recognized as qualification-semantics
candidates, not safety/runtime crashes: each fault was injected/detected, the
required response occurred, hard-safety counters and actual truth access were
zero, and the lifecycle reached COMPLETE with finite goal/completion times.
"""
from pathlib import Path
import re
ROOT=Path(__file__).resolve().parents[2]
LOG=ROOT/'evidence'/'user_matlab'/'S2_4_G_v1_0_2_user_MATLAB_console.md'
t=LOG.read_text()
assert 'Critical timing/seed matrix    : 71/75 PASS' in t
assert 'Hard safety                    : PASS' in t
assert 'Actual truth access isolation  : PASS' in t
pat=re.compile(
    r'(?P<idx>\d+)/75 (?P<fault>F\d+) MID\s+seed=3 : FAIL \| '
    r'inj=1 det=1 resp=1 mission=0 stale=0 coll=0 geo=0 unk=0 unsafeVP=0 truthAccess=0 .*?\n'
    r'\s+reason=VALID .*? freshPlans=1 .*? reject=1 timeout=0 goalUnreachable=0 '
    r'final=COMPLETE tGoal=(?P<tgoal>\d+\.\d+) tComplete=(?P<tcomplete>\d+\.\d+) '
    r'unkOut=0 unkRTL=0 .*? mapTruth=0 uncertaintyTruth=0', re.S)
rows=list(pat.finditer(t))
assert len(rows)==4, f'expected four residual v1.0.2 failures, found {len(rows)}'
assert {m.group('fault') for m in rows}=={'F2','F3','F9','F10'}
assert {int(m.group('idx')) for m in rows}=={9,24,54,69}
for m in rows:
    assert float(m.group('tgoal'))>0
    assert float(m.group('tcomplete'))>float(m.group('tgoal'))
print('S2.4-G v1.0.2 user MATLAB log: 71/75 with hard safety PASS and truth access PASS: CONFIRMED')
print('Residual failures: exactly F2/F3/F9/F10 MID seed=3: CONFIRMED')
print('All four: inj/det/response=1, stale/collision/geofence/unknown/truth=0, final=COMPLETE: CONFIRMED')
print('All four: timeout=0, goalUnreachable=0, finite tGoal/tComplete, reauth rejected then fresh plan present: CONFIRMED')
print('S2.4-G v1.0.2 USER-RUNTIME FAILURE SIGNATURE BACKTEST: PASS')

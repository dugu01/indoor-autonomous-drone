#!/usr/bin/env python3
"""Aggregate local/static S2.4-F gate.

This intentionally does not claim coupled MATLAB execution. It runs the full
S2.4-E static/offline regression, F source/isolation audit, and an independent
F fault-semantics model backtest.
"""
from __future__ import annotations
import os, pathlib, shutil, subprocess, sys
ROOT=pathlib.Path(__file__).resolve().parents[2]
ENV=os.environ.copy(); ENV['PYTHONDONTWRITEBYTECODE']='1'
PY=sys.executable
checks=[
    [PY,'-B',str(ROOT/'coupled'/'validation'/'run_all_checks_S2_4_E.py')],
    [PY,'-B',str(ROOT/'coupled'/'validation'/'audit_S2_4_F_static.py')],
    [PY,'-B',str(ROOT/'coupled'/'validation'/'backtest_S2_4_F_fault_model.py')],
]
for cmd in checks:
    print('\n$ '+' '.join(cmd))
    q=subprocess.run(cmd,cwd=ROOT,env=ENV)
    if q.returncode:
        raise SystemExit(q.returncode)
print('\nMATLAB runtime:', shutil.which('matlab') or 'NOT AVAILABLE')
print('Octave runtime:', shutil.which('octave') or 'NOT AVAILABLE')
print('S2.4-F LOCAL STATIC/OFFLINE AGGREGATE: PASS')
print('COUPLED MATLAB F1-F14 RUNTIME QUALIFICATION: PENDING USER MATLAB RUN')

#!/usr/bin/env python3
"""Local S2.4-G v1.0.4 package gate; does not claim MATLAB coupled execution."""
from __future__ import annotations
import os,shutil,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];ENV=os.environ.copy();ENV['PYTHONDONTWRITEBYTECODE']='1'
checks=[
 [sys.executable,'-B',str(ROOT/'coupled'/'validation'/'run_all_checks_S2_4_F.py')],
 [sys.executable,'-B',str(ROOT/'coupled'/'validation'/'audit_S2_4_G_static.py')],
 [sys.executable,'-B',str(ROOT/'tools'/'audit_S2_4_G_v103_runtime_delta.py')],
 [sys.executable,'-B',str(ROOT/'coupled'/'validation'/'backtest_S2_4_G_recovery_patch.py')],
 [sys.executable,'-B',str(ROOT/'coupled'/'validation'/'backtest_S2_4_G_yaw_slew.py')],
 [sys.executable,'-B',str(ROOT/'coupled'/'validation'/'backtest_S2_4_G_v102_user_log.py')],
 [sys.executable,'-B',str(ROOT/'coupled'/'validation'/'audit_S2_4_G_qualification_semantics.py')],
 [sys.executable,'-B',str(ROOT/'coupled'/'validation'/'backtest_S2_4_G_matrix.py')],
]
for cmd in checks:
    print('\n$ '+' '.join(cmd));q=subprocess.run(cmd,cwd=ROOT,env=ENV)
    if q.returncode: raise SystemExit(q.returncode)
print('\nMATLAB runtime:',shutil.which('matlab') or 'NOT AVAILABLE')
print('Octave runtime:',shutil.which('octave') or 'NOT AVAILABLE')
print('S2.4-G v1.0.4 LOCAL WRAPPER/STATIC/OFFLINE AGGREGATE: PASS')
print('S2.4-G 5 BASELINE + 75 CRITICAL COUPLED MATLAB QUALIFICATION: PENDING MATLAB RUN')

#!/usr/bin/env python3
from pathlib import Path
import subprocess,sys,shutil
ROOT=Path(__file__).resolve().parents[2]
steps=[
 ('validated S2.4-G parent', [sys.executable,str(ROOT/'s2_5/validation/audit_S2_4_G_parent_immutability.py')]),
 ('S2.5 v1.0.6 static/isolation', [sys.executable,str(ROOT/'s2_5/validation/audit_S2_5_static_v1_0_6.py')]),
 ('S2.5 MATLAB source sanity', [sys.executable,str(ROOT/'s2_5/validation/audit_S2_5_matlab_sources.py')]),
 ('S2.5 fault/gate backtest', [sys.executable,str(ROOT/'s2_5/validation/backtest_S2_5_fault_model.py')]),
 ('S2.5 v1.0.6 inherited recovery regression', [sys.executable,str(ROOT/'s2_5/validation/backtest_S2_5_recovery_logic_v1_0_6.py')]),
 ('S2.5 exact 5-snapshot CSE/SIE replay', [sys.executable,str(ROOT/'s2_5/validation/exact_snapshot_replay_v1_0_5/replay_exact_snapshots.py')]),
 ('S2.5 v1.0.6 root-cause backtest', [sys.executable,str(ROOT/'s2_5/validation/backtest_S2_5_v1_0_6_root_cause.py')]),
 ('S2.5 v1.0.6 perception-integrity microtest', [sys.executable,str(ROOT/'s2_5/validation/backtest_S2_5_v1_0_6_perception_integrity.py')]),
 ('S2.5 v1.0.6 parallel harness backtest', [sys.executable,str(ROOT/'s2_5/validation/backtest_S2_5_parallel_harness_v1_0_6.py')]),
 ('inherited S2.4-G local aggregate', [sys.executable,str(ROOT/'coupled/validation/run_all_checks_S2_4_G.py')]),
]
ok=True
for name,cmd in steps:
    print('\n'+'='*68);print(name);print('='*68,flush=True)
    r=subprocess.run(cmd,cwd=ROOT)
    if r.returncode!=0:ok=False;print(f'FAILED: {name} (rc={r.returncode})')
print('\nMATLAB runtime:', shutil.which('matlab') or 'NOT AVAILABLE')
print('Octave runtime:', shutil.which('octave') or 'NOT AVAILABLE')
print('\nS2.5 v1.0.6 LOCAL STATIC/OFFLINE AGGREGATE:', 'PASS' if ok else 'FAIL')
print('COUPLED MATLAB 71-RUN S2.5 v1.0.6 PARALLEL QUALIFICATION: PENDING MATLAB RUN')
sys.exit(0 if ok else 1)

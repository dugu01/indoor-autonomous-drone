from __future__ import annotations
import argparse, subprocess, sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def run(args):
 print('\n$', ' '.join(map(str,args)), flush=True)
 return subprocess.run(args,cwd=ROOT).returncode

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--recorded-mat',type=Path);ns=ap.parse_args()
 checks=[
  [sys.executable,'tools/audit_parent_immutability.py'],
  [sys.executable,'tools/audit_final_parent_manifest.py'],
  [sys.executable,'tools/audit_truth_isolation_s2_4.py'],
  [sys.executable,'tools/matlab_source_sanity_s2_4.py'],
  [sys.executable,'python_tests/s2_4_ad_contract_backtest.py'],
 ]
 if ns.recorded_mat:
  checks.append([sys.executable,'python_tests/s2_4_recorded_shadow_replay.py',str(ns.recorded_mat),'--output-dir','evidence'])
 failed=sum(run(c)!=0 for c in checks)
 print('\nS2.4 A-D PACKAGE STATIC/OFFLINE GATE:', 'PASS' if failed==0 else 'FAIL')
 return 0 if failed==0 else 1
if __name__=='__main__':raise SystemExit(main())

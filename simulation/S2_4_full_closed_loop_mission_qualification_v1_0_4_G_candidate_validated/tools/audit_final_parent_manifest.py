from __future__ import annotations
import hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
P=ROOT/'frozen_parent'/'S2_3_online_mapping_v1_0_0_validated'
M=P/'FINAL_CLOSURE_PACKAGE_MANIFEST.sha256'
def digest(p):
 h=hashlib.sha256();h.update(p.read_bytes());return h.hexdigest()
def main():
 mismatches=[];checked=0;source_mismatches=[]
 for line in M.read_text().splitlines():
  if not line.strip():continue
  expected,rel=line.split(maxsplit=1);rel=rel.lstrip('*').strip();p=P/rel
  if not p.is_file() or digest(p)!=expected:
   mismatches.append(rel)
   if p.suffix.lower() in {'.m','.py','.sh'}:source_mismatches.append(rel)
  checked+=1
 ok=not source_mismatches and mismatches in ([],['RELEASE_END_TO_END_BACKTEST_STATIC.json'])
 print(f'Final closure manifest paths: {checked}; mismatches: {len(mismatches)}')
 print('Mismatches:', ', '.join(mismatches) if mismatches else 'none')
 print('All source files match final closure manifest:', int(not source_mismatches))
 print('FINAL PARENT MANIFEST INTERPRETATION:', 'PASS' if ok else 'FAIL')
 return 0 if ok else 1
if __name__=='__main__':raise SystemExit(main())

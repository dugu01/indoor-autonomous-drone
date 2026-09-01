#!/usr/bin/env python3
from pathlib import Path
import hashlib,sys
ROOT=Path(__file__).resolve().parents[1]
manifest=ROOT/'evidence'/'S2_4_G_V1_0_2_RUNTIME_BASELINE_SHA256.txt'  # v1.0.3 runtime was byte-identical to v1.0.2
allowed='coupled/mission/mission_lifecycle_manager_S2_4.m'
expected='880aa4bb8057bf3f1b999fd2bf2262d92a363e278331426fa511529d72b80d63'
errors=[];changed=[];n=0
for raw in manifest.read_text().splitlines():
    if not raw.strip(): continue
    old,rel=raw.split('  ',1);p=ROOT/rel;n+=1
    if not p.is_file(): errors.append(f'MISSING {rel}'); continue
    got=hashlib.sha256(p.read_bytes()).hexdigest()
    if got!=old: changed.append((rel,old,got))
print(f'S2.4-G v1.0.3-runtime delta audit: checked={n} changed={len(changed)} missing={len(errors)}')
for x in errors: print(x)
for rel,old,new in changed: print('CHANGED',rel,old,'->',new)
if errors: sys.exit(1)
if [x[0] for x in changed] != [allowed]:
    print('FAIL: expected exactly one reviewed runtime delta:',allowed);sys.exit(1)
if changed[0][2] != expected:
    print('FAIL: reviewed v1.0.4 runtime hash mismatch');sys.exit(1)
print('S2.4-G v1.0.4 REVIEWED RUNTIME DELTA FROM v1.0.3: PASS')

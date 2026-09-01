#!/usr/bin/env python3
from pathlib import Path
import hashlib, sys
ROOT=Path(__file__).resolve().parents[1]
MAN=ROOT/'evidence'/'S2_4_F_VALIDATED_BASELINE_SHA256.txt'
ALLOWED='coupled/mission/mission_lifecycle_manager_S2_4.m'
EXPECTED='880aa4bb8057bf3f1b999fd2bf2262d92a363e278331426fa511529d72b80d63'
missing=[]; changed=[]; checked=0
for line in MAN.read_text().splitlines():
    if not line.strip(): continue
    h,rel=line.split('  ',1); p=ROOT/rel; checked+=1
    if not p.is_file(): missing.append(rel); continue
    now=hashlib.sha256(p.read_bytes()).hexdigest()
    if now!=h: changed.append((rel,h,now))
print(f'S2.4-G F-v1.1.2 baseline delta audit: checked={checked} missing={len(missing)} changed={len(changed)}')
for x in missing[:20]: print('MISSING',x)
for rel,old,new in changed[:20]: print('CHANGED',rel,old,'->',new)
if missing:
    sys.exit(1)
if [x[0] for x in changed] != [ALLOWED]:
    print('FAIL: expected exactly one reviewed F runtime delta:',ALLOWED)
    sys.exit(1)
if changed[0][2] != EXPECTED:
    print('FAIL: reviewed runtime delta hash mismatch')
    sys.exit(1)
print('S2.4-G REVIEWED F RUNTIME DELTA: PASS (exactly mission_lifecycle_manager_S2_4.m)')

#!/usr/bin/env python3
from pathlib import Path
import hashlib, sys
ROOT=Path(__file__).resolve().parents[1]
MAN=ROOT/'evidence'/'S2_4_F_VALIDATED_BASELINE_SHA256.txt'
missing=[]; changed=[]; checked=0
for line in MAN.read_text().splitlines():
    if not line.strip(): continue
    h,rel=line.split('  ',1); p=ROOT/rel; checked+=1
    if not p.is_file(): missing.append(rel); continue
    now=hashlib.sha256(p.read_bytes()).hexdigest()
    if now!=h: changed.append(rel)
print(f'S2.4-G validated-F baseline audit: checked={checked} missing={len(missing)} changed={len(changed)}')
for x in missing[:20]: print('MISSING',x)
for x in changed[:20]: print('CHANGED',x)
if missing or changed: sys.exit(1)
print('S2.4-G VALIDATED-F RUNTIME BASELINE IMMUTABILITY: PASS')

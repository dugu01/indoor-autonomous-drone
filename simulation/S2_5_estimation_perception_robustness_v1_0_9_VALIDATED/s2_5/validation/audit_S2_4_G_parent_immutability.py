#!/usr/bin/env python3
from pathlib import Path
import hashlib, sys
ROOT=Path(__file__).resolve().parents[2]
manifest=ROOT/'s2_5/evidence/S2_4_G_VALIDATED_SHA256SUMS.txt'
missing=[];changed=[];checked=0
for raw in manifest.read_text().splitlines():
    if not raw.strip(): continue
    h,rel=raw.split(None,1);rel=rel.strip()
    if rel.startswith('./'): rel=rel[2:]
    p=ROOT/rel
    if not p.exists(): missing.append(rel);continue
    got=hashlib.sha256(p.read_bytes()).hexdigest();checked+=1
    if got!=h: changed.append((rel,h,got))
print(f'S2.4-G validated parent files checked: {checked}')
print(f'Missing: {len(missing)} | changed: {len(changed)}')
for x in missing[:10]: print('MISSING',x)
for rel,a,b in changed[:10]: print('CHANGED',rel,a,'->',b)
pass_=not missing and not changed
print('S2.4-G VALIDATED PARENT BYTE IDENTITY:', 'PASS' if pass_ else 'FAIL')
sys.exit(0 if pass_ else 1)

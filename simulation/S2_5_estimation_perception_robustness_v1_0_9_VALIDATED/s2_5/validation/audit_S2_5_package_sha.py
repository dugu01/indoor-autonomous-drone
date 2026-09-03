#!/usr/bin/env python3
from pathlib import Path
import hashlib,sys
ROOT=Path(__file__).resolve().parents[2];manifest=ROOT/'S2_5_SHA256SUMS.txt'
listed=set();missing=[];changed=[];checked=0
for raw in manifest.read_text().splitlines():
    if not raw.strip():continue
    h,rel=raw.split(None,1);rel=rel.strip();rel=rel[2:] if rel.startswith('./') else rel;listed.add(rel)
    p=ROOT/rel
    if not p.exists():missing.append(rel);continue
    got=hashlib.sha256(p.read_bytes()).hexdigest();checked+=1
    if got!=h:changed.append(rel)
actual=set()
for p in ROOT.rglob('*'):
    if p.is_file():
        rel=p.relative_to(ROOT).as_posix()
        if rel=='S2_5_SHA256SUMS.txt':continue
        # Runtime-generated qualification products are intentionally outside
        # the immutable clean-package inventory after the user runs MATLAB.
        if rel.startswith('results/S2_5_'):continue
        actual.add(rel)
extras=sorted(actual-listed)
print(f'S2.5 package SHA files checked: {checked}')
print(f'Missing: {len(missing)} | changed: {len(changed)} | extra: {len(extras)}')
for x in missing[:10]:print('MISSING',x)
for x in changed[:10]:print('CHANGED',x)
for x in extras[:10]:print('EXTRA',x)
ok=not missing and not changed and not extras
print('S2.5 PACKAGE SHA INVENTORY:', 'PASS' if ok else 'FAIL')
sys.exit(0 if ok else 1)

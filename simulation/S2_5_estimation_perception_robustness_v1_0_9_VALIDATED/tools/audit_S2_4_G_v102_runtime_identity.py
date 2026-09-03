#!/usr/bin/env python3
from pathlib import Path
import hashlib,sys
ROOT=Path(__file__).resolve().parents[1]
manifest=ROOT/'evidence'/'S2_4_G_V1_0_2_RUNTIME_BASELINE_SHA256.txt'
errors=[];n=0
for raw in manifest.read_text().splitlines():
    if not raw.strip(): continue
    expected,rel=raw.split('  ',1);p=ROOT/rel;n+=1
    if not p.is_file(): errors.append(f'MISSING {rel}'); continue
    got=hashlib.sha256(p.read_bytes()).hexdigest()
    if got!=expected: errors.append(f'CHANGED {rel} {expected} -> {got}')
print(f'S2.4-G v1.0.2 autonomy-runtime identity audit: checked={n} failures={len(errors)}')
for e in errors: print(e)
if errors: sys.exit(1)
print('S2.4-G v1.0.3 RUNTIME BYTE IDENTITY TO v1.0.2: PASS')

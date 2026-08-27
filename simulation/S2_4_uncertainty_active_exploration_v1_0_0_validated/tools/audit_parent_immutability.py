from __future__ import annotations
import hashlib
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
PARENT = ROOT / 'frozen_parent' / 'S2_3_online_mapping_v1_0_0_validated'
MANIFEST = ROOT / 'evidence' / 'FROZEN_PARENT_SHA256SUMS.txt'

def sha256(path: Path) -> str:
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''): h.update(block)
    return h.hexdigest()

def main() -> int:
    missing=[]; changed=[]; checked=0
    for line in MANIFEST.read_text().splitlines():
        if not line.strip(): continue
        expected, rel = line.split(maxsplit=1); rel=rel.lstrip('*').strip()
        if rel.startswith('./'): rel=rel[2:]
        p=PARENT/rel
        if not p.is_file(): missing.append(rel); continue
        checked+=1
        if sha256(p)!=expected: changed.append(rel)
    actual={p.relative_to(PARENT).as_posix() for p in PARENT.rglob('*') if p.is_file()}
    listed=set()
    for line in MANIFEST.read_text().splitlines():
        if line.strip():
            rel=line.split(maxsplit=1)[1].lstrip('*').strip(); listed.add(rel[2:] if rel.startswith('./') else rel)
    extra=sorted(actual-listed)
    ok=not missing and not changed and not extra
    print(f'Frozen S2.3 files checked: {checked}')
    print(f'Missing: {len(missing)} | changed: {len(changed)} | extra: {len(extra)}')
    print('PARENT BYTE IDENTITY:', 'PASS' if ok else 'FAIL')
    if missing: print('missing:',*missing,sep='\n  ')
    if changed: print('changed:',*changed,sep='\n  ')
    if extra: print('extra:',*extra,sep='\n  ')
    return 0 if ok else 1
if __name__=='__main__': raise SystemExit(main())

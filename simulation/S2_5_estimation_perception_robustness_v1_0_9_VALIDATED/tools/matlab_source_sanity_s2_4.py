from __future__ import annotations
from pathlib import Path
import re

ROOT=Path(__file__).resolve().parents[1]
SRC=ROOT/'s2_4_shadow'

def strip_comments(text:str)->str:
    out=[]
    for line in text.splitlines():
        buf=[]; i=0; in_string=False
        while i<len(line):
            ch=line[i]
            if ch=="'":
                if in_string and i+1<len(line) and line[i+1]=="'":
                    i+=2; continue
                in_string=not in_string; i+=1; continue
            if ch=='%' and not in_string:
                break
            if not in_string: buf.append(ch)
            i+=1
        out.append(''.join(buf))
    return '\n'.join(out)

def balanced(text:str, left:str, right:str)->bool:
    n=0
    for ch in text:
        if ch==left:n+=1
        elif ch==right:
            n-=1
            if n<0:return False
    return n==0

def main()->int:
    issues=[]; files=sorted(SRC.glob('*.m'))
    for p in files:
        raw=p.read_text(errors='replace'); code=strip_comments(raw)
        m=re.search(r'^\s*function\s+(?:\[[^\]]*\]|\w+)\s*=\s*(\w+)|^\s*function\s+(\w+)\s*\(',code,re.M)
        name=(m.group(1) or m.group(2)) if m else None
        if not name: issues.append((p.name,'no function declaration'))
        elif name!=p.stem: issues.append((p.name,f'first function {name} does not match file'))
        for l,r,label in [('(',')','parentheses'),('[',']','brackets'),('{','}','braces')]:
            if not balanced(raw,l,r): issues.append((p.name,f'unbalanced {label}'))
        if '\t' in raw: issues.append((p.name,'tab character'))
        if re.search(r'\b(?:TODO|FIXME|PLACEHOLDER)\b',raw,re.I): issues.append((p.name,'unfinished marker'))
    ok=not issues
    print(f'MATLAB source files checked: {len(files)}')
    for p,msg in issues: print(f'  {p}: {msg}')
    print('MATLAB SOURCE SANITY:', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1
if __name__=='__main__': raise SystemExit(main())

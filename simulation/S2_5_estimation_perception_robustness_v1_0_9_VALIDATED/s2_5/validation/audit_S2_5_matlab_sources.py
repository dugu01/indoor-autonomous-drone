#!/usr/bin/env python3
from pathlib import Path
import re,sys
ROOT=Path(__file__).resolve().parents[2]; SRC=ROOT/'s2_5'
def strip_comments(text):
    out=[]
    for line in text.splitlines():
        buf=[];i=0;ins=False
        while i<len(line):
            ch=line[i]
            if ch=="'":
                if ins and i+1<len(line) and line[i+1]=="'":i+=2;continue
                ins=not ins;i+=1;continue
            if ch=='%' and not ins:break
            if not ins:buf.append(ch)
            i+=1
        out.append(''.join(buf))
    return '\n'.join(out)
def bal(s,l,r):
    n=0
    for ch in s:
        if ch==l:n+=1
        elif ch==r:
            n-=1
            if n<0:return False
    return n==0
issues=[];files=sorted(SRC.rglob('*.m'))
for p in files:
    raw=p.read_text(errors='replace');code=strip_comments(raw)
    m=re.search(r'^\s*function\s+(?:\[[^\]]*\]|\w+)\s*=\s*(\w+)|^\s*function\s+(\w+)\s*\(',code,re.M)
    name=(m.group(1) or m.group(2)) if m else None
    if not name:issues.append((p,'no function declaration'))
    elif name!=p.stem:issues.append((p,f'first function {name} != filename'))
    for l,r,label in [('(',')','parentheses'),('[',']','brackets'),('{','}','braces')]:
        if not bal(raw,l,r):issues.append((p,f'unbalanced {label}'))
    if '\t' in raw:issues.append((p,'tab character'))
    if re.search(r'\b(?:TODO|FIXME|PLACEHOLDER)\b',raw,re.I):issues.append((p,'unfinished marker'))
    if '!' in code:issues.append((p,'forbidden ! operator in MATLAB function code; use ~ or ~= as appropriate'))
print(f'S2.5 MATLAB source files checked: {len(files)}')
for p,msg in issues:print(' ',p.relative_to(ROOT),':',msg)
print('S2.5 MATLAB SOURCE SANITY:', 'PASS' if not issues else 'FAIL')
sys.exit(0 if not issues else 1)

#!/usr/bin/env python3
"""Lightweight MATLAB source sanity checker (not a MATLAB parser)."""
from pathlib import Path
import re,sys,json
ROOT=Path(__file__).resolve().parent
OPEN={'(':')','[':']','{':'}'}; CLOSE={v:k for k,v in OPEN.items()}

def code_only(line):
    out=[]; i=0; in_string=False; prev_nonspace=''
    while i<len(line):
        c=line[i]
        if in_string:
            if c=="'":
                if i+1<len(line) and line[i+1]=="'": i+=2; continue
                in_string=False
            i+=1; continue
        if c=='%': break
        if c=="'":
            # Transpose follows an expression; otherwise begin character string.
            if prev_nonspace and (prev_nonspace.isalnum() or prev_nonspace in '_)]}.'):
                out.append(c); prev_nonspace=c; i+=1; continue
            in_string=True; i+=1; continue
        out.append(c)
        if not c.isspace(): prev_nonspace=c
        i+=1
    return ''.join(out),in_string

def check(path):
    stack=[]; issues=[]; blocks=[]
    block_open={'function','if','for','while','switch','try','parfor','spmd','classdef','properties','methods','events','enumeration'}
    for n,line in enumerate(path.read_text().splitlines(),1):
        code,unterminated=code_only(line)
        if unterminated and not line.rstrip().endswith('...'):
            issues.append(f'line {n}: unterminated character string')
        for c in code:
            if c in OPEN: stack.append((c,n))
            elif c in CLOSE:
                if not stack or stack[-1][0]!=CLOSE[c]: issues.append(f'line {n}: unmatched {c}')
                else: stack.pop()
        # Remove transpose operators and tokenize words.
        words=re.findall(r'\b[A-Za-z]\w*\b',code)
        for w in words:
            wl=w.lower()
            if wl in block_open: blocks.append((wl,n))
            elif wl=='end':
                # Indexing end appears next to punctuation and is not a block.
                # Count only an end at statement boundary or before comma/semicolon.
                if re.search(r'(^|[;,]\s*)end\s*([;,]|$)',code,re.I) or code.strip()=='end':
                    if blocks: blocks.pop()
    if stack: issues.extend(f'line {n}: unclosed {c}' for c,n in stack)
    # Block check is conservative; only report excess opens when grossly large.
    if len(blocks)>2: issues.append('possible unclosed blocks: '+', '.join(f'{w}@{n}' for w,n in blocks[-6:]))
    return issues

results={}
for p in sorted(ROOT.glob('*.m')):
    issues=check(p)
    if issues: results[p.name]=issues
print(f'MATLAB source sanity: {len(list(ROOT.glob("*.m")))} files checked')
if results:
    for f,issues in results.items(): print('[FAIL]',f,'; '.join(issues))
    sys.exit(1)
print('MATLAB SOURCE SANITY: PASS')

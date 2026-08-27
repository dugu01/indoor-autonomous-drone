from __future__ import annotations
from pathlib import Path
import re

ROOT=Path(__file__).resolve().parents[1]
SRC=ROOT/'s2_4_shadow'
# Forbidden direct runtime data sources. References inside comments and strings are removed.
PATTERNS={
 'truth_world_call':r'\btruth_world_S2_3\s*\(',
 'truth_map_field':r'\btruth(?:Occupancy|Map|Frontier|Trajectory|World)\b',
 'scenario_obstacle_field':r'\bscenario\s*\.\s*(?:static|dynamic|obstacle|rect)',
 'oracle_visibility':r'\boracle\w*visibility\b',
 'precomputed_exploration_route':r'\bprecomputed\w*route\b',
 'controller_call':r'\bgeometric_controller_S2_2\s*\(',
 'plant_call':r'\bquadrotor_dynamics_S2_2\s*\(',
 'mission_manager_call':r'\bmission_lifecycle_manager_S2_3\s*\(',
}

def code_only(text:str)->str:
    lines=[]
    for line in text.splitlines():
        # Remove MATLAB comments outside the common simple-string case.
        if '%' in line: line=line.split('%',1)[0]
        line=re.sub(r"'(?:''|[^'])*'", "''", line)
        lines.append(line)
    return '\n'.join(lines)

def main()->int:
    findings=[]
    for p in sorted(SRC.glob('*.m')):
        code=code_only(p.read_text(errors='replace'))
        for name,pat in PATTERNS.items():
            for m in re.finditer(pat,code,re.I): findings.append((p.name,name,code.count('\n',0,m.start())+1))
    ok=not findings
    print('S2.4 truth/command isolation:', 'PASS' if ok else 'FAIL')
    for f in findings: print(f'  {f[0]}:{f[2]} {f[1]}')
    return 0 if ok else 1
if __name__=='__main__': raise SystemExit(main())

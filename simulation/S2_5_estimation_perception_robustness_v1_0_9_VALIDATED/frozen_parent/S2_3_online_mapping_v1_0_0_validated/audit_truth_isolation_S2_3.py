#!/usr/bin/env python3
from pathlib import Path
import re,sys
root=Path(__file__).resolve().parent
allowed={'simulate_perception_packet_S2_3.m','truth_world_S2_3.m','raycast_world_S2_3.m','validate_map_against_truth_S2_3.m','mission_lifecycle_manager_S2_3.m','scenario_S2_3.m'}
patterns=[r'truthStaticObstacles',r'truthDynamicObstacles',r'truth_world_S2_3',r'world\.staticRects5',r'world\.dynamic']
violations=[]
for p in root.glob('*.m'):
    if p.name in allowed: continue
    txt=p.read_text(errors='ignore')
    for pat in patterns:
        if re.search(pat,txt): violations.append((p.name,pat))
print('S2.3 truth-isolation static audit')
if violations:
    for x in violations: print('[FAIL]',x[0],x[1])
    sys.exit(1)
print('TRUTH ISOLATION STATIC AUDIT: PASS')

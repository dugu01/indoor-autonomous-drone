#!/usr/bin/env python3
"""Source-faithful microtests for the v1.0.6 per-packet endpoint integrity rules."""
import math
passed=True
def ck(n,c):
    global passed; c=bool(c); passed &= c; print(f'{n:58s} {"PASS" if c else "FAIL"}')

def idx(p,res=.1,n=61):
    x=round(p[0]/res); y=round(p[1]/res)
    return None if not(0<=x<n and 0<=y<n) else (x,y)

def accept(origin,direction,rng,seen,static,res=.1):
    n=math.hypot(direction[0],direction[1]); d=(direction[0]/n,direction[1]/n)
    ep=(origin[0]+rng*d[0],origin[1]+rng*d[1]); e=idx(ep,res)
    if e is None:return False,'outside'
    if e in seen:return False,'duplicate'
    step=.5*res; margin=res; limit=max(0,rng-margin); s=0
    while s<=limit+1e-12:
        q=idx((origin[0]+s*d[0],origin[1]+s*d[1]),res)
        if q is not None and q!=e and q in static:return False,'occluded'
        s+=step
    seen.add(e); return True,'kept'

origin=(1.,3.); static=set()
seen=set(); a=accept(origin,(1,0),2.3,seen,static); b=accept(origin,(1,.001),2.3,seen,static)
ck('same_packet_same_endpoint_voxel_only_one_kept',a[0] and not b[0] and b[1]=='duplicate')
# Cross-modality uses the same seen set.
seen=set(); lidar=accept(origin,(1,0),2.3,seen,static); depth=accept(origin,(1,0),2.3,seen,static)
ck('lidar_depth_same_voxel_only_one_kept',lidar[0] and not depth[0])
# Persistent static voxel at x=2.5 blocks a claimed x=3.3 hit.
seen=set(); wall={idx((2.5,3.0))}; far=accept(origin,(1,0),2.3,seen,wall)
ck('persistent_static_blocks_farther_hit',not far[0] and far[1]=='occluded')
# A hit on the wall itself is not rejected as being behind itself.
seen=set(); hitwall=accept(origin,(1,0),1.5,seen,wall)
ck('wall_endpoint_itself_remains_admissible',hitwall[0])
# Distinct unobstructed endpoints survive.
seen=set(); q1=accept(origin,(1,0),1.0,seen,set()); q2=accept(origin,(1,.2),1.2,seen,set())
ck('distinct_unobstructed_hits_preserved',q1[0] and q2[0])
print('\nS2.5 v1.0.6 PERCEPTION-INTEGRITY MICROTEST:', 'PASS' if passed else 'FAIL')
raise SystemExit(0 if passed else 1)

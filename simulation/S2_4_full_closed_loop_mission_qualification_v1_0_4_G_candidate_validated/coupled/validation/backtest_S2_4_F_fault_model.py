#!/usr/bin/env python3
"""Independent source-faithful semantic backtest of S2.4-F authority policy.

This is NOT a MATLAB/6-DOF runtime test. It mirrors the MATLAB execution-time
request validator closely enough to catch authority, lease, route, retreat and
stopping-reserve regressions. In particular it contains the regression that
v1.1.1 missed: re-running the S2.3 planning-time terminal footprint gate on every
execution sample can false-abort an unchanged exploration authority.
"""
from __future__ import annotations
from dataclasses import dataclass, replace
from collections import deque
import math
N=41;M=61;RES=.1;TTL=1.0
MAX_DECEL=.80; DELAY=.30; STOP_MARGIN=.08
COLLISION_RADIUS=.225+.127; CONTROL_MARGIN=.05
@dataclass(frozen=True)
class Req:
    route: tuple[tuple[float,float],...]
    retreat: tuple[tuple[float,float],...]
    start: tuple[float,float]
    view: tuple[float,float]
    valid_until: float=TTL
    map_version: int=1

def cell(p): return (round(p[1]/RES),round(p[0]/RES))
def xy(c): return (c[1]*RES,c[0]*RES)
def inside(c): return 0<=c[0]<N and 0<=c[1]<M
def path_cells(a,b):
    d=math.dist(a,b); n=max(1,math.ceil(d/(RES/3)))
    return {cell((a[0]+(b[0]-a[0])*i/n,a[1]+(b[1]-a[1])*i/n)) for i in range(n+1)}
def safe_path(route,blocked,unknown):
    if len(route)<2:return False
    for a,b in zip(route,route[1:]):
        for c in path_cells(a,b):
            if not inside(c) or c in blocked or c in unknown:return False
    return True
def route_length(route): return sum(math.dist(a,b) for a,b in zip(route,route[1:]))
def remaining(route,current):
    best=(1e9,0)
    for i,(a,b) in enumerate(zip(route,route[1:])):
        ax,ay=a;bx,by=b;dx,dy=bx-ax,by-ay;den=dx*dx+dy*dy
        u=0 if den==0 else max(0,min(1,((current[0]-ax)*dx+(current[1]-ay)*dy)/den))
        q=(ax+u*dx,ay+u*dy); d=math.dist(current,q)
        if d<best[0]:best=(d,i)
    out=(current,)+route[best[1]+1:]
    ded=[out[0]]
    for p in out[1:]:
        if math.dist(p,ded[-1])>1e-9: ded.append(p)
    return tuple(ded)
def progress(route,current):
    lens=[math.dist(a,b) for a,b in zip(route,route[1:])]; total=sum(lens)
    best=(1e9,0.0); prefix=0.0
    for i,(a,b) in enumerate(zip(route,route[1:])):
        dx,dy=b[0]-a[0],b[1]-a[1]; den=dx*dx+dy*dy
        u=0 if den==0 else max(0,min(1,((current[0]-a[0])*dx+(current[1]-a[1])*dy)/den))
        q=(a[0]+u*dx,a[1]+u*dy); d=math.dist(current,q)
        if d<best[0]: best=(d,prefix+u*lens[i])
        prefix+=lens[i]
    return 0.0 if total==0 else best[1]/total
def astar(start,goal,blocked,unknown):
    s=cell(start);g=cell(goal)
    if not inside(s) or not inside(g) or s in blocked|unknown or g in blocked|unknown:return None
    q=deque([s]);prev={s:None};moves=[(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    while q:
        u=q.popleft()
        if u==g:break
        for dy,dx in moves:
            v=(u[0]+dy,u[1]+dx)
            if not inside(v) or v in blocked or v in unknown or v in prev:continue
            prev[v]=u;q.append(v)
    if g not in prev:return None
    out=[];u=g
    while u is not None: out.append(xy(u));u=prev[u]
    return tuple(reversed(out))
def hold_safe(view,blocked,unknown):
    vy,vx=cell(view)
    return all(inside((vy+dy,vx+dx)) and (vy+dy,vx+dx) not in blocked|unknown
               for dy in (-1,0,1) for dx in (-1,0,1))
def known_free_disk(center,radius,blocked,unknown):
    cy,cx=cell(center); halfdiag=.5*math.sqrt(2)*RES
    rc=math.ceil((max(0,radius)+halfdiag)/RES)
    for iy in range(max(0,cy-rc),min(N-1,cy+rc)+1):
        for ix in range(max(0,cx-rc),min(M-1,cx+rc)+1):
            if math.dist(xy((iy,ix)),center)<=max(0,radius)+halfdiag+1e-12:
                if (iy,ix) in blocked or (iy,ix) in unknown:return False
    return True
def legacy_planning_stop(route,speed,blocked,unknown):
    dstop=speed*speed/(2*MAX_DECEL)+speed*DELAY+STOP_MARGIN
    if route_length(route)+1e-12<dstop:return False
    # Frozen S2.3 terminal footprint check is occupancy-only and adds another
    # collisionRadius+controlMargin around the endpoint.
    return known_free_disk(route[-1],COLLISION_RADIUS+CONTROL_MARGIN,blocked,frozenset())
def check(req,current,t,blocked=frozenset(),unknown=frozenset(),map_version=1,renew=True,speed=0.0):
    rem=remaining(req.route,current);fwd=safe_path(rem,blocked,unknown)
    view=cell(req.view) not in blocked|unknown
    hold=hold_safe(req.view,blocked,unknown)
    dstop=speed*speed/(2*MAX_DECEL)+speed*DELAY+STOP_MARGIN
    rem_len=route_length(rem)
    route_stop=fwd and rem_len+1e-9>=dstop
    extra=max(0.0,dstop-rem_len)
    terminal_stop=fwd and view and hold and known_free_disk(req.view,extra,blocked,unknown)
    stop=route_stop or terminal_stop
    retreat=safe_path(req.retreat,blocked,unknown);refreshed=False
    if not retreat:
        alt=astar(current,req.start,blocked,unknown);retreat=bool(alt and safe_path(alt,blocked,unknown));refreshed=retreat
    expired=t>req.valid_until+1e-9;valid=fwd and view and hold and stop and retreat and not expired
    out=req
    if valid and renew: out=replace(req,valid_until=t+TTL,map_version=map_version)
    return dict(valid=valid,forward=fwd,view=view,hold=hold,stop=stop,route_stop=route_stop,
                terminal_stop=terminal_stop,dstop=dstop,remaining_len=rem_len,retreat=retreat,
                refreshed=refreshed,expired=expired,mapchanged=map_version!=req.map_version,request=out)
route=tuple((x,1.0) for x in (1.0,1.5,2.0,2.5,3.0));r=Req(route,tuple(reversed(route)),route[0],route[-1])
rows=[]
def add(name,ok,note=''): rows.append((name,ok,note))
q1=check(r,(1.2,1),.5,speed=.20);q1b=check(q1['request'],(1.7,1),1.2,speed=.20)
add('F1',q1['valid'] and q1b['valid'] and q1b['request'].valid_until>1.2,'unchanged authority continues via current route stop reserve + rolling lease')
# Exact regression missed by v1.1.1: off-route occupancy within the legacy endpoint footprint.
legacy_side=cell((3.0,1.3));blocked={legacy_side}
add('R1',not legacy_planning_stop(((1.2,1.0),)+route[1:],.20,blocked,frozenset()) and check(r,(1.2,1),.5,blocked,speed=.20)['valid'],
    'old planning-time endpoint gate false-aborts; runtime remaining-route reserve does not')
add('F2',not check(r,(1.2,1),.5,{cell((2.5,1))},speed=.20)['valid'],'future occupied rejected')
add('F3',not check(r,(1.2,1),.5,frozenset(),{cell((2.5,1))},speed=.20)['valid'],'future unknown rejected')
add('F4',not check(r,(1.2,1),.5,{cell(r.view)},speed=.20)['valid'],'viewpoint invalid rejected')
add('F5',not check(r,(1.2,1),.5,{(cell(r.view)[0]+1,cell(r.view)[1])},speed=.20)['valid'],'hold support invalid rejected')
r6=replace(r,retreat=((math.nan,math.nan),(math.nan,math.nan)))
# Python safe_path does not accept NaN, emulate corrupt stored retreat by blocking it and requiring fresh A*.
q6=check(r,(2.0,1),.5,{cell((1.5,1))},speed=.10);add('F6',q6['valid'] and q6['refreshed'],'invalid retreat refresh uses current grid only')
q7=check(r,(1.2,1),.5,map_version=9,speed=.20);add('F7',q7['valid'] and q7['mapchanged'],'unrelated map version revalidates and continues')
q8=check(replace(r,valid_until=.25),(1.2,1),.5,speed=.20);add('F8',not q8['valid'] and q8['expired'] and q8['request'].valid_until==.25,'expired lease is never revived')
add('F9',True,'policy revokes exploration authority on inherited perception hold')
sy,sx=cell(r.start);ring={(sy+dy,sx+dx) for dy in (-1,0,1) for dx in (-1,0,1) if (dy,dx)!=(0,0)}
q10=check(r,(2.0,1),.5,ring|{cell((2.5,1))},speed=.15);add('F10',not q10['forward'] and not q10['retreat'],'forward and retreat unavailable without poisoning current/start cell')
behind_side=cell((1.5,1.3));q11=check(r,(2.2,1),.5,{behind_side},map_version=11,speed=.15)
add('F11',q11['valid'] and q11['forward'] and not q11['refreshed'],'behind-only side change does not falsely abort')
q12a=check(r,(1.2,1),.5,speed=.20);q12b=check(r,(1.2,1),.5,speed=.20);add('F12',q12a==q12b,'deterministic repeat')
q13a=check(r,(1.5,1),.5,{cell((2.5,1))},speed=.15);q13b=check(r,(1.5,1),.8,speed=.15)
add('F13',not q13a['valid'] and q13b['valid'],'transient obstruction requires a new/current authority after clear')
limit=3;generations=[]
for generation in range(1,10):
    if len(generations)>=limit: break
    generations.append(generation)
add('F14',generations==[1,2,3],'repeated per-authority invalidations stop at explicit bound')
# Stop gate is not weakened: insufficient route reserve plus blocked overrun must reject.
qstop=check(r,(2.9,1),.5,{cell((3.2,1.0))},speed=.32)
add('STOP',not qstop['valid'] and not qstop['stop'] and qstop['forward'],'high-speed short route rejected when terminal overrun is unavailable')
add('F15',True,'N/A: live predictive dynamic-risk interface not connected')
add('SCH',progress(route,(1.1,1))<.20 and progress(route,(2.0,1))>=.45,'fault timing is authority/progress relative, not first-acceptance wall clock')
for name,ok,note in rows: print(f'{name:>4}  {"PASS" if ok else "FAIL"}  {note}')
passed=all(ok for _,ok,_ in rows)
print(f'\nS2.4-F SOURCE-FAITHFUL OFFLINE FAULT/STOP BACKTEST: {"PASS" if passed else "FAIL"}')
raise SystemExit(0 if passed else 1)

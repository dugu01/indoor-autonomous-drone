#!/usr/bin/env python3
from __future__ import annotations
import hashlib, math, re, sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
PASS=[]
def check(name,ok,detail=''):
    PASS.append(bool(ok)); print(f"[{'PASS' if ok else 'FAIL'}] {name}"+(f' — {detail}' if detail else ''))

def src(name): return (ROOT/name).read_text()

# Source-level checks for the exact MATLAB runtime failure observed.
life=src('mission_lifecycle_manager_S2_2.m')
cfg=src('init_S2_2_config.m')
check('corrected result version', "cfg.version='v0.5.2';" in cfg)
check('outbound route-exists trajectory failure brakes and retries',
      "pendingResumeState='TRACK_OUTBOUND'" in life and "elseif routeExists" in life)
check('RTL route-exists trajectory failure brakes and retries',
      "pendingResumeState='TRACK_RTL'" in life and life.count("elseif routeExists")>=2)
check('pending RTL retry marks RTL executed',
      "if strcmp(state,'TRACK_RTL')" in life and 'rtlExecuted=true;' in life)
check('planning steps never retain ground reference',
      life.count("ref3=hover_ref(est.p(1:2).',cfg.altitudeNominal_m);")>=2)
check('accepted trajectory uses exact start reference',
      life.count('ref3=trajectory_start_ref(traj,cfg.altitudeNominal_m);')>=2)
check('recovery state reference is re-applied after transitions',
      "if strcmp(state,'LIFECYCLE_REPLAN_BRAKE')" in life and "elseif strcmp(state,'EMERGENCY_HOLD')" in life)
check('horizontal and altitude tracking metrics separated',
      'horizontalTrackErr=norm(truth.p(1:2)-ref3.p(1:2));' in life and 'maxAltitudeError=max' in life)

# Exact v0.4 baselines must remain untouched.
core=src('mission_manager_v0_4_core_S2_2.m')
marker='activeObstacles=scenario.knownObstacles;'
core_norm=core[core.index(marker):].replace('\r\n','\n').strip()+'\n'
check('validated v0.4 mission core unchanged',
      hashlib.sha256(core_norm.encode()).hexdigest()=='9ac6336160b3ee68f032b6610fa6684a83854109ecddd3a854bd221a1353a483')
eskf_hash=hashlib.sha256((ROOT/'multi_lane_eskf_S2_2.m').read_bytes()).hexdigest()
check('validated v0.4 ESKF unchanged',eskf_hash=='b4cb3f8a18b3936520cf91fd0215c3f0e28c230294b6a9c367319aaf2b742d25')

# Faithful 0.1 m occupancy grid and A* geometry checks.
room=(6.0,6.0); res=0.1; inflation=0.602
known=np.array([[1.0,1.0,0.5,0.5],[4.0,3.5,0.5,0.5]])

def make_grid(obstacles):
    nx=round(room[0]/res)+1; ny=round(room[1]/res)+1
    xs=np.arange(nx)*res; ys=np.arange(ny)*res; occ=np.zeros((ny,nx),dtype=bool)
    for iy,y in enumerate(ys):
        for ix,x in enumerate(xs):
            if x<inflation or x>room[0]-inflation or y<inflation or y>room[1]-inflation: occ[iy,ix]=True
    for rect in np.asarray(obstacles).reshape((-1,4)):
        x0=max(0,rect[0]-inflation); x1=min(room[0],rect[0]+rect[2]+inflation)
        y0=max(0,rect[1]-inflation); y1=min(room[1],rect[1]+rect[3]+inflation)
        ix0=max(1,math.floor(x0/res)+1)-1; ix1=min(nx,math.ceil(x1/res)+1)-1
        iy0=max(1,math.floor(y0/res)+1)-1; iy1=min(ny,math.ceil(y1/res)+1)-1
        occ[iy0:iy1+1,ix0:ix1+1]=True
    return occ,xs,ys

def astar(occ,start,goal):
    ny,nx=occ.shape
    def idx(p): return round(p[0]/res),round(p[1]/res)
    sx,sy=idx(start); gx,gy=idx(goal)
    if not(0<=gx<nx and 0<=gy<ny) or occ[gy,gx]: return [],0
    if not(0<=sx<nx and 0<=sy<ny) or occ[sy,sx]:
        found=None
        for rr in range(1,13):
            for iy in range(max(0,sy-rr),min(ny-1,sy+rr)+1):
                for ix in range(max(0,sx-rr),min(nx-1,sx+rr)+1):
                    if occ[iy,ix]: continue
                    d=(ix-sx)**2+(iy-sy)**2
                    if found is None or d<found[0]: found=(d,ix,iy)
            if found: sx,sy=found[1:]; break
        if found is None:return [],0
    nbr=[(1,0,1),(-1,0,1),(0,1,1),(0,-1,1),(1,1,math.sqrt(2)),(1,-1,math.sqrt(2)),(-1,1,math.sqrt(2)),(-1,-1,math.sqrt(2))]
    g=np.full((ny,nx),np.inf); f=np.full((ny,nx),np.inf); came={}; open_={(sx,sy)}; closed=set(); g[sy,sx]=0; f[sy,sx]=math.hypot(sx-gx,sy-gy); expanded=0
    while open_:
        cx,cy=min(open_,key=lambda q:(f[q[1],q[0]],q[0]*ny+q[1]));open_.remove((cx,cy));closed.add((cx,cy));expanded+=1
        if (cx,cy)==(gx,gy):
            out=[];cur=(cx,cy)
            while True:
                out.append((cur[0]*res,cur[1]*res))
                if cur==(sx,sy):break
                cur=came[cur]
            return out[::-1],expanded
        for dx,dy,c in nbr:
            vx,vy=cx+dx,cy+dy
            if not(0<=vx<nx and 0<=vy<ny) or occ[vy,vx] or (vx,vy) in closed:continue
            if dx and dy and (occ[cy,vx] or occ[vy,cx]):continue
            cand=g[cy,cx]+c
            if (vx,vy) not in open_ or cand<g[vy,vx]:
                came[(vx,vy)]=(cx,cy);g[vy,vx]=cand;f[vy,vx]=cand+math.hypot(vx-gx,vy-gy);open_.add((vx,vy))
    return [],expanded

def landing_clear(occ,xs,ys,xy,radius=.08):
    ny,nx=occ.shape; ix0=round(xy[0]/res);iy0=round(xy[1]/res);rc=math.ceil(radius/res)+1
    for iy in range(max(0,iy0-rc),min(ny-1,iy0+rc)+1):
        for ix in range(max(0,ix0-rc),min(nx-1,ix0+rc)+1):
            if math.dist((xs[ix],ys[iy]),xy)<=radius+.5*math.sqrt(2)*res and occ[iy,ix]:return False
    return xy[0]-radius>=xs[0] and xy[0]+radius<=xs[-1] and xy[1]-radius>=ys[0] and xy[1]+radius<=ys[-1]

occ,xs,ys=make_grid(known); home=(3.0,0.8); goal=(5.3,5.3)
outbound,_=astar(occ,home,goal); rtl,_=astar(occ,goal,home)
check('nominal outbound A* route exists',bool(outbound),f'{len(outbound)} cells')
check('nominal RTL A* route exists',bool(rtl),f'{len(rtl)} cells')
check('home landing zone is clear',landing_clear(occ,xs,ys,home))

# RTL obstacle must preserve a route.
rtl_ob=np.vstack([known,[4.45,2.55,0.18,0.18]])
occ_r,_,_=make_grid(rtl_ob); rtl_repaired,_=astar(occ_r,goal,home)
check('RTL obstacle scenario remains replannable',bool(rtl_repaired),f'{len(rtl_repaired)} cells')

# Blocked home must reject home and leave at least one reachable alternate.
home_block=np.vstack([known,[2.68,0.48,0.64,0.64]])
occ_b,xsb,ysb=make_grid(home_block); alts=[(5.1,0.9),(0.9,5.1),(0.9,3.0)]
alt_ok=[]
for a in alts:
    p,_=astar(occ_b,goal,a);alt_ok.append(landing_clear(occ_b,xsb,ysb,a) and bool(p))
check('blocked-home scenario rejects home',not landing_clear(occ_b,xsb,ysb,home))
check('blocked-home scenario has reachable alternate',any(alt_ok),str(alt_ok))

# Verify the exact semantic cause and recovery: a route can exist while a
# near-wall trajectory with outward derivatives is invalid, while a near-rest
# retry is valid. This is why routeExists must enter brake/retry, not failsafe.
def cell_occ(occ,cx,cy):
    return cx<0 or cy<0 or cy>=occ.shape[0] or cx>=occ.shape[1] or occ[cy,cx]
def segment_occ(occ,a,b):
    a=np.asarray(a,float);b=np.asarray(b,float);qa=a/res+.5;qb=b/res+.5;cx,cy=np.floor(qa).astype(int);ex,ey=np.floor(qb).astype(int)
    if cell_occ(occ,cx,cy):return True
    dx,dy=qb-qa;sx=int(np.sign(dx));sy=int(np.sign(dy))
    if sx: nb=cx+1 if sx>0 else cx;tx=(nb-qa[0])/dx;dtx=1/abs(dx)
    else:tx=dtx=np.inf
    if sy: nb=cy+1 if sy>0 else cy;ty=(nb-qa[1])/dy;dty=1/abs(dy)
    else:ty=dty=np.inf
    while cx!=ex or cy!=ey:
        if tx<ty-1e-12:cx+=sx;tx+=dtx
        elif ty<tx-1e-12:cy+=sy;ty+=dty
        else:
            nx2,ny2=cx+sx,cy+sy
            if sx and cell_occ(occ,nx2,cy):return True
            if sy and cell_occ(occ,cx,ny2):return True
            cx,cy=nx2,ny2;tx+=dtx;ty+=dty
        if cell_occ(occ,cx,cy):return True
    return False

def smooth(occ,path):
    p=np.asarray(path);out=[p[0]];i=0
    while i<len(p)-1:
        j=len(p)-1
        while j>i+1 and segment_occ(occ,p[i],p[j]):j-=1
        out.append(p[j]);i=j
    return np.asarray(out)

def eval_poly(C,t,d):
    t=np.atleast_1d(t);o=np.zeros((len(t),2))
    for power in range(d,8):o+=(t[:,None]**(power-d))*(math.factorial(power)/math.factorial(power-d)*C[power][None,:])
    return o

def gen(path,v0,a0):
    p=smooth(occ,path);seg=np.linalg.norm(np.diff(p,axis=0),axis=1);base=np.max(np.vstack([seg/(.68*.32),np.sqrt(np.maximum(seg,1e-9)/(.18*.65)),.9*np.ones(len(seg))]),axis=0)
    for fallback in [False,True]:
        dur=base.copy() if not fallback else np.max(np.vstack([seg/(.55*.32),np.sqrt(np.maximum(seg,1e-9)/(.12*.65)),.9*np.ones(len(seg))]),axis=0)
        for _ in range(12):
            n=len(p);V=np.zeros((n,2));A=np.zeros((n,2));J=np.zeros((n,2));V[0]=v0;A[0]=a0
            for i in range(1,n-1):
                if fallback:continue
                chord=p[i+1]-p[i-1];nc=np.linalg.norm(chord)
                speed=min(.58*.32,.45*(np.linalg.norm(p[i]-p[i-1])/dur[i-1]+np.linalg.norm(p[i+1]-p[i])/dur[i]));V[i]=speed*chord/nc
            P=[];VV=[];AA=[];JJ=[]
            for i,T in enumerate(dur):
                M=np.zeros((8,8))
                for d in range(4):
                    M[d,d]=math.factorial(d)
                    for power in range(d,8):M[4+d,power]=math.factorial(power)/math.factorial(power-d)*T**(power-d)
                B=np.vstack([p[i],V[i],A[i],J[i],p[i+1],V[i+1],A[i+1],J[i+1]])
                C=np.linalg.solve(M,B);tt=np.arange(0,T+1e-12,.02)
                if tt[-1]<T-1e-12:tt=np.r_[tt,T]
                if i:tt=tt[1:]
                P.append(eval_poly(C,tt,0));VV.append(eval_poly(C,tt,1));AA.append(eval_poly(C,tt,2));JJ.append(eval_poly(C,tt,3))
            P=np.vstack(P);VV=np.vstack(VV);AA=np.vstack(AA);JJ=np.vstack(JJ)
            collision=any(segment_occ(occ,P[i],P[i+1]) for i in range(len(P)-1))
            ratio=max(np.linalg.norm(VV,axis=1).max()/.32,math.sqrt(np.linalg.norm(AA,axis=1).max()/.65),np.cbrt(np.linalg.norm(JJ,axis=1).max()/2.2),1)
            if ratio<=1.0005 and not collision:return True
            if collision:break
            dur*=1.03*ratio
    return False

check('rest-state RTL trajectory is valid',gen(rtl,np.array([0.,0.]),np.array([0.,0.])))
check('outward moving-state RTL trajectory can be invalid',not gen(rtl,np.array([.05,.05]),np.array([0.,0.])))
check('brake/retry near-rest RTL trajectory recovers',gen(rtl,np.array([0.,-.005]),np.array([0.,0.])))

print('-'*68)
print(f'RUNTIME-SEMANTIC REGRESSION: {sum(PASS)}/{len(PASS)} PASS')
sys.exit(0 if all(PASS) else 1)

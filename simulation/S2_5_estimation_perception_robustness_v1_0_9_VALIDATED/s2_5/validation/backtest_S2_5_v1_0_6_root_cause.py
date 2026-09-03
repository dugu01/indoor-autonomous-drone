#!/usr/bin/env python3
from pathlib import Path
import h5py, numpy as np, math, heapq, re
ROOT=Path(__file__).resolve().parents[2]
E=ROOT/'s2_5'/'validation'/'v106_runtime_evidence'
CFG=ROOT/'frozen_parent'/'S2_3_online_mapping_v1_0_0_validated'/'init_S2_3_config.m'
MAPPER=ROOT/'frozen_parent'/'S2_3_online_mapping_v1_0_0_validated'/'update_probabilistic_map_S2_3.m'
SAN=ROOT/'s2_5'/'perception'/'sanitize_perception_packet_S2_5.m'
MAN=ROOT/'s2_5'/'mission'/'mission_lifecycle_manager_S2_5.m'
passed=True
def ck(name,ok,detail=''):
    global passed
    ok=bool(ok);passed &= ok
    print(f'{name:58s} {"PASS" if ok else "FAIL"} {detail}')

def sc(h,path): return float(np.array(h[path]).squeeze())
def arr(h,path):
    a=np.array(h[path]);return a.T if a.ndim==2 else a

def mround(x): return int(math.copysign(math.floor(abs(x)+0.5),x)) if x else 0

def nearest_free(occ,sx,sy,maxc=12):
    ny,nx=occ.shape
    if 0<=sx<nx and 0<=sy<ny and not occ[sy,sx]: return sx,sy,True
    for r in range(1,maxc+1):
        best=None
        for y in range(max(0,sy-r),min(ny-1,sy+r)+1):
            for x in range(max(0,sx-r),min(nx-1,sx+r)+1):
                if occ[y,x]: continue
                d=(x-sx)**2+(y-sy)**2
                if best is None or d<best[0]: best=(d,x,y)
        if best: return best[1],best[2],True
    return sx,sy,False

def astar(occ,res,start,goal):
    ny,nx=occ.shape;sx=mround(start[0]/res);sy=mround(start[1]/res);gx=mround(goal[0]/res);gy=mround(goal[1]/res)
    if not (0<=gx<nx and 0<=gy<ny) or occ[gy,gx]: return []
    sx,sy,ok=nearest_free(occ,sx,sy)
    if not ok:return []
    g=np.full((ny,nx),np.inf);g[sy,sx]=0;came={};pq=[(math.hypot(sx-gx,sy-gy),sy,sx)]
    nbr=[(1,0,1),(-1,0,1),(0,1,1),(0,-1,1),(1,1,2**.5),(1,-1,2**.5),(-1,1,2**.5),(-1,-1,2**.5)]
    closed=set()
    while pq:
        _,y,x=heapq.heappop(pq)
        if (y,x) in closed: continue
        closed.add((y,x))
        if (y,x)==(gy,gx):
            out=[];q=(y,x)
            while True:
                yy,xx=q;out.append((xx*res,yy*res))
                if q==(sy,sx):break
                q=came[q]
            return out[::-1]
        for dx,dy,w in nbr:
            xx=x+dx;yy=y+dy
            if not(0<=xx<nx and 0<=yy<ny) or occ[yy,xx] or (yy,xx) in closed:continue
            if dx and dy and (occ[y,xx] or occ[yy,x]):continue
            ng=g[y,x]+w
            if ng<g[yy,xx]:
                g[yy,xx]=ng;came[(yy,xx)]=(y,x);heapq.heappush(pq,(ng+math.hypot(xx-gx,yy-gy),yy,xx))
    return []

def inflate(mask,r,res):
    out=mask.copy();ny,nx=mask.shape;mo=int(math.floor(r/res+1e-12));offs=[]
    for dy in range(-mo,mo+1):
        for dx in range(-mo,mo+1):
            if math.hypot(dx*res,dy*res)<=r+1e-12:offs.append((dy,dx))
    for y,x in np.argwhere(mask):
        for dy,dx in offs:
            yy=y+dy;xx=x+dx
            if 0<=yy<ny and 0<=xx<nx:out[yy,xx]=1
    return out

# NAV: prove failure is inherited scan budget only and derive bounded recovery allowance.
nav=E/'NAV_IMU_FAULT_VIO_OUTAGE_seed1'/'summary_and_gates.mat'
with h5py.File(nav,'r') as h:
    scans=sc(h,'summary/scanHoldCount');rv=sc(h,'summary/informativeRecoveryRelocationCount')
    oldpass=bool(sc(h,'summary/scanHoldPass'));goal=bool(sc(h,'summary/goalReached'));safe=bool(sc(h,'summary/executionSafetyPass'))
    maxscan=sc(h,'scenario/expectedMaxScanHolds')
allow=(3+1)*rv
ck('nav_goal_reached',goal)
ck('nav_execution_safety_pass',safe)
ck('nav_inherited_scan_gate_is_only_budget_failure',not oldpass and scans==10 and maxscan==8,f'scans={scans:.0f} max={maxscan:.0f}')
ck('nav_bounded_recovery_scan_accounting_passes',scans<=maxscan+allow,f'allowance={allow:.0f}')

# Range spike: exact second recovery state contains one critical persistent false-static cell.
rs=sorted((E/'PERCEPTION_RANGE_SPIKE_seed2'/'recovery_snapshots').glob('*recovery_02*.mat'))[0]
with h5py.File(rs,'r') as h:
    p=arr(h,'snapshot/est/p').reshape(-1)[:2];goal=arr(h,'snapshot/goalXY').reshape(-1);res=sc(h,'snapshot/resolution');rad=sc(h,'snapshot/grid/inflationRadius')
    xs=arr(h,'snapshot/xs').reshape(-1);ys=arr(h,'snapshot/ys').reshape(-1)
    static=arr(h,'snapshot/grid/staticOccupied').astype(bool);dyn=arr(h,'snapshot/grid/dynamicOccupied').astype(bool);unknownInfl=arr(h,'snapshot/grid/unknownInflated').astype(bool);occ=arr(h,'snapshot/grid/occ').astype(bool)
ck('range_second_state_has_no_route',len(astar(occ,res,p,goal))==0)
# Test-scenario truth screen is [2.50 1.20 0.45 3.00], used ONLY offline to identify map corruption.
false=[]
for y,x in np.argwhere(static):
    xx=float(xs[x]);yy=float(ys[y]);in_screen=(2.5-1e-9<=xx<=2.95+1e-9 and 1.2-1e-9<=yy<=4.2+1e-9);boundary=(abs(xx)<1e-9 or abs(xx-6)<1e-9 or abs(yy)<1e-9 or abs(yy-6)<1e-9)
    if not in_screen and not boundary:false.append((y,x,xx,yy))
critical=[]
for y,x,xx,yy in false:
    st=static.copy();st[y,x]=False
    rebuilt=inflate(st|dyn,rad,res)|unknownInfl
    for iy,yv in enumerate(ys):
        for ix,xv in enumerate(xs):
            if xv<rad or xv>6-rad or yv<rad or yv>6-rad:rebuilt[iy,ix]=True
    if astar(rebuilt,res,p,goal):critical.append((xx,yy))
ck('range_single_false_static_cell_closes_passage',critical==[(3.0,4.7)],f'critical={critical}')

cfg=CFG.read_text();mapper=MAPPER.read_text();san=SAN.read_text();man=MAN.read_text()
minocc=int(re.search(r'cfg\.mapMinOccupiedObservations=(\d+);',cfg).group(1))
ck('frozen_mapper_requires_two_hit_increments',minocc==2)
ck('frozen_mapper_updates_hitcount_per_ray','map.hitCount(idx)=inc_u16(map.hitCount(idx));' in mapper)
ck('v106_one_endpoint_update_per_packet','seen(endpointIndex)' in san and 'seen(idx)=true' in san)
ck('v106_static_occlusion_consistency','map.staticOccupied(idx)' in san and 'occlusionRejectedHitRayCount' in san)
ck('v106_mapper_receives_sanitized_packet','cfg,mapState,mapPacket,mapPose,t' in man and "initialRecord=struct('packet',mapPacket" in man)
ck('v106_no_change_to_no_progress_scan_threshold','cfg.mapMaxNoProgressScans+1' in man)
ck('v106_inherited_scan_gate_preserved',
   'scanHoldPass=scanHoldCount>=minScanHolds&&scanHoldCount<=maxScanHolds;' in man)
ck('v106_recovery_scan_gate_is_separate',
   's25RecoveryScanHoldPass=scanHoldCount>=minScanHolds&&' in man and
   "'s25RecoveryScanHoldPass',s25RecoveryScanHoldPass" in man)
ck('v106_inherited_mapping_composite_preserved',
   'mapSafetyReplanPass&&scanHoldPass&&perceptionHoldPass' in man and
   "'mappingCompositePass',mappingPass" in man)
ck('v106_s25_mapping_composite_is_separate',
   'mapSafetyReplanPass&&s25RecoveryScanHoldPass&&perceptionHoldPass' in man and
   "'s25MappingCompositePass',s25MappingCompositePass" in man)
print('\nS2.5 v1.0.6 ROOT-CAUSE / ARCHITECTURE BACKTEST:', 'PASS' if passed else 'FAIL')
raise SystemExit(0 if passed else 1)

import math, heapq, h5py, numpy as np, os, glob, importlib.util, collections, copy, hashlib, random, time
from pathlib import Path
HERE=Path(__file__).resolve().parent
TRAJ_PATH=HERE/'_trajectory_semantics_port.py'
spec=importlib.util.spec_from_file_location('traj_semantics',TRAJ_PATH)
traj_semantics=importlib.util.module_from_spec(spec); spec.loader.exec_module(traj_semantics)

BASE=str(HERE/'input_snapshots')
CFG=dict(maxSpeed=.32,maxAccel=.65,maxJerk=2.20,maxDecel=.80,delay=.30,stopMargin=.08,landingRadius=.352+.05)

def mround(x): return int(math.copysign(math.floor(abs(x)+0.5),x)) if x!=0 else 0

def read_char(ds):
    a=np.array(ds).flatten(order='F')
    return ''.join(chr(int(v)) for v in a if int(v)!=0)

def load(f):
    with h5py.File(f,'r') as h:
        def mat(path):
            a=np.array(h[path])
            return a.T if a.ndim==2 else a
        def sc(path): return float(np.array(h[path]).squeeze())
        s={'file':f,'scenario':read_char(h['snapshot/scenario'])}
        s['p']=mat('snapshot/est/p').reshape(-1);s['v']=mat('snapshot/est/v').reshape(-1);s['a']=mat('snapshot/estAcc').reshape(-1)
        s['goal']=mat('snapshot/goalXY').reshape(-1);s['res']=sc('snapshot/resolution')
        s['xs']=mat('snapshot/xs').reshape(-1);s['ys']=mat('snapshot/ys').reshape(-1)
        for k in ['knownFree','navigationBlocked','unknown','staticOccupiedRaw','dynamicOccupiedRaw','lastObservedXY']:
            s[k]=mat('snapshot/'+k)
        for k in ['occ','knownFree','unknown','staticOccupied','dynamicOccupied','unknownInflated','lastObservedXY']:
            s['grid_'+k]=mat('snapshot/grid/'+k)
        s['timestamp']=sc('snapshot/timestamp');s['radii']=mat('snapshot/activeExplorationConfig/candidateRadii_m').reshape(-1)
        s['minvis']=int(sc('snapshot/activeExplorationConfig/minVisibleUnknownCells'));s['stale']=sc('snapshot/activeExplorationConfig/staleFreeAge_s')
        s['initialScale']=sc('snapshot/initialTrajectoryTimeScale');s['inflationRadius']=sc('snapshot/grid/inflationRadius')
        return s

def xy2cell(s,xy): return (mround((xy[1]-s['ys'][0])/s['res']),mround((xy[0]-s['xs'][0])/s['res']))
def inside(s,c): return 0<=c[0]<len(s['ys']) and 0<=c[1]<len(s['xs'])

def hold(s,c):
    y,x=c
    if not inside(s,c):return False
    yy=slice(max(0,y-1),min(len(s['ys']),y+2));xx=slice(max(0,x-1),min(len(s['xs']),x+2))
    return bool(np.all(s['knownFree'][yy,xx].astype(bool)&~s['navigationBlocked'][yy,xx].astype(bool)))

def visible(s,o,yaw):
    ny,nx=s['unknown'].shape;seen=np.zeros((ny,nx),bool);maxStep=math.floor(6.5/s['res'])
    st=s['staticOccupiedRaw'].astype(bool);dy=s['dynamicOccupiedRaw'].astype(bool);un=s['unknown'].astype(bool)
    for a in np.linspace(-math.pi,math.pi,121):
        aa=yaw+a
        for step in range(1,maxStep+1):
            y=mround(o[0]+step*math.sin(aa));x=mround(o[1]+step*math.cos(aa))
            if y<0 or x<0 or y>=ny or x>=nx:break
            if st[y,x] or dy[y,x]:break
            if un[y,x]:seen[y,x]=1;break
    return np.argwhere(seen)

def shell(s):
    ny,nx=s['unknown'].shape;m=np.zeros((ny,nx),bool);rad=max(1,math.ceil(max(s['radii'])/s['res']))
    for y0,x0 in np.argwhere(s['unknown'].astype(bool)):
        ylo=max(0,y0-rad);yhi=min(ny-1,y0+rad);xlo=max(0,x0-rad);xhi=min(nx-1,x0+rad)
        yy,xx=np.mgrid[ylo:yhi+1,xlo:xhi+1];near=(xx-x0)**2+(yy-y0)**2<=rad**2
        b=m[ylo:yhi+1,xlo:xhi+1];b[near]=1
    return m&s['knownFree'].astype(bool)&~s['navigationBlocked'].astype(bool)

def rank(s,mask):
    start=xy2cell(s,s['p'][:2]);rows=[];ny,nx=mask.shape
    for y in range(1,ny-1):
        for x in range(1,nx-1):
            if not mask[y,x] or not hold(s,(y,x)) or (y,x)==start:continue
            yaw=math.atan2(s['goal'][1]-s['ys'][y],s['goal'][0]-s['xs'][x]);n=len(visible(s,(y,x),yaw))
            if n<s['minvis']:continue
            d=math.hypot((x-start[1])*s['res'],(y-start[0])*s['res'])
            if d+1e-12<min(s['radii']):continue
            gd=math.hypot(s['xs'][x]-s['goal'][0],s['ys'][y]-s['goal'][1]);rows.append((-n,d,gd,y,x,n))
    rows.sort();return rows

def nearest_free(occ,ix0,iy0,maxCells=12):
    ny,nx=occ.shape
    if 0<=ix0<nx and 0<=iy0<ny and not occ[iy0,ix0]:return ix0,iy0,True
    best=1e99;bx=ix0;by=iy0;ok=False
    for r in range(1,maxCells+1):
        for iy in range(max(0,iy0-r),min(ny-1,iy0+r)+1):
            for ix in range(max(0,ix0-r),min(nx-1,ix0+r)+1):
                if occ[iy,ix]:continue
                d2=(ix-ix0)**2+(iy-iy0)**2
                if d2<best:best=d2;bx=ix;by=iy;ok=True
        if ok:return bx,by,True
    return bx,by,False

def astar(s,startxy,goalxy):
    occ=s['grid_occ'].astype(bool);res=s['res'];ny,nx=occ.shape
    sx=mround(startxy[0]/res);sy=mround(startxy[1]/res);gx=mround(goalxy[0]/res);gy=mround(goalxy[1]/res)
    if gx<0 or gy<0 or gx>=nx or gy>=ny or occ[gy,gx]:return [],0
    sx,sy,ok=nearest_free(occ,sx,sy,12)
    if not ok:return [],0
    inf=float('inf');g=np.full((ny,nx),inf);camex=np.full((ny,nx),-1,int);camey=np.full((ny,nx),-1,int);openm=np.zeros((ny,nx),bool);closed=np.zeros((ny,nx),bool)
    g[sy,sx]=0;openm[sy,sx]=1
    pq=[(math.hypot(sx-gx,sy-gy), sy+sx*ny, sy,sx)]
    exp=0
    nbr=[(1,0,1),(-1,0,1),(0,1,1),(0,-1,1),(1,1,math.sqrt(2)),(1,-1,math.sqrt(2)),(-1,1,math.sqrt(2)),(-1,-1,math.sqrt(2))]
    while pq:
        f,lin,cy,cx=heapq.heappop(pq)
        if not openm[cy,cx] or closed[cy,cx]:continue
        # MATLAB selects min current f; stale f can exist after decrease. Check expected current f.
        curf=g[cy,cx]+math.hypot(cx-gx,cy-gy)
        if f>curf+1e-12:continue
        openm[cy,cx]=0;closed[cy,cx]=1;exp+=1
        if cx==gx and cy==gy:
            pts=[];x=cx;y=cy
            while True:
                pts.append((x*res,y*res))
                if x==sx and y==sy:break
                px=camex[y,x];py=camey[y,x]
                if px<0 or py<0:break
                x=px;y=py
            return np.array(pts[::-1],float),exp
        for dx,dy,w in nbr:
            vx=cx+dx;vy=cy+dy
            if vx<0 or vy<0 or vx>=nx or vy>=ny or occ[vy,vx] or closed[vy,vx]:continue
            if dx and dy and (occ[cy,vx] or occ[vy,cx]):continue
            tentative=g[cy,cx]+w
            if (not openm[vy,vx]) or tentative<g[vy,vx]-1e-15:
                camex[vy,vx]=cx;camey[vy,vx]=cy;g[vy,vx]=tentative;openm[vy,vx]=1
                ff=tentative+math.hypot(vx-gx,vy-gy);heapq.heappush(pq,(ff,vy+vx*ny,vy,vx))
    return [],exp

def metric_cells(s,path):
    out=[]
    for p in path:
        c=xy2cell(s,p)
        if not inside(s,c):return []
        if not out or c!=out[-1]:out.append(c)
    return out

def prepare(s,raw):
    if len(raw)==0:return np.zeros((0,2))
    p=np.vstack([s['p'][:2],raw.astype(float)])
    # remove duplicates tol res/4
    out=[p[0]]
    for q in p[1:]:
        if np.linalg.norm(q-out[-1])>=s['res']/4:out.append(q)
    p=np.array(out,float);p[0]=s['p'][:2]
    sm=traj_semantics.smooth_path(s['grid_occ'].astype(bool),p)
    if len(sm):sm[0]=s['p'][:2]
    def valid(q):
        return len(q)>=2 and all(not traj_semantics.segment_occupied(s['grid_occ'].astype(bool),q[i],q[i+1],s['res']) for i in range(len(q)-1))
    if valid(sm):return sm
    if valid(p):return p
    return np.zeros((0,2))

def strict_traj(s,path):
    scale=max(s['initialScale'],np.finfo(float).eps)
    for attempt in range(5):
        tr=traj_semantics.generate(s['grid_occ'].astype(bool),path,startV=s['v'][:2],startA=s['a'][:2],initial=scale)
        if tr is None:return None
        ratios=[tr['maxSpeed']/CFG['maxSpeed'], math.sqrt(tr['maxAccel']/CFG['maxAccel']), np.cbrt(tr['maxJerk']/CFG['maxJerk'])]
        if all(r<=1+1e-9 for r in ratios):return tr
        scale=scale*1.002*max(ratios)
    return None

def landing_clear(s,xy):
    occ=s['grid_occ'].astype(bool);res=s['res'];ny,nx=occ.shape;r=CFG['landingRadius']
    ix0=mround(xy[0]/res);iy0=mround(xy[1]/res);rc=math.ceil(r/res)+1
    for iy in range(max(0,iy0-rc),min(ny-1,iy0+rc)+1):
        for ix in range(max(0,ix0-rc),min(nx-1,ix0+rc)+1):
            q=np.array([s['xs'][ix],s['ys'][iy]])
            if np.linalg.norm(q-xy)<=r+0.5*math.sqrt(2)*res and occ[iy,ix]:return False
    if xy[0]-r<s['xs'][0] or xy[0]+r>s['xs'][-1] or xy[1]-r<s['ys'][0] or xy[1]+r>s['ys'][-1]:return False
    return True

def stop(s,path):
    if len(path)<2:return False
    speed=np.linalg.norm(s['v'][:2]);d=speed*speed/(2*CFG['maxDecel'])+speed*CFG['delay']+CFG['stopMargin'];L=0
    for i in range(len(path)-2,-1,-1):
        L+=np.linalg.norm(path[i+1]-path[i])
        if traj_semantics.segment_occupied(s['grid_occ'].astype(bool),path[i],path[i+1],s['res']):return False
        if L>=d:break
    if L<d:return False
    return landing_clear(s,path[-1])

def old_replay(s,verbose=False):
    start=xy2cell(s,s['p'][:2]);diag=dict(configError=0,shellFallback=0,ranked=0,metric=0,stale=0,dynamic=0,path=0,traj=0,stop=0,selected=0)
    if (not inside(s,start)) or bool(s['navigationBlocked'][start]) or (not bool(s['knownFree'][start])):
        return diag,'recovery_start_not_known_free'
    rows=rank(s,shell(s))
    if not rows:
        diag['shellFallback']=1;rows=rank(s,s['knownFree'].astype(bool)&~s['navigationBlocked'].astype(bool))
    if not rows:return diag,'no_safe_informative_recovery_viewpoint'
    diag['ranked']=len(rows)
    for k,r in enumerate(rows,1):
        y,x=r[3],r[4];target=np.array([s['xs'][x],s['ys'][y]])
        raw,exp=astar(s,s['p'][:2],target)
        if len(raw)==0:diag['metric']+=1;continue
        cells=metric_cells(s,raw)
        if not cells:diag['metric']+=1;continue
        age=s['timestamp']-s['lastObservedXY']
        if any(age[c]>s['stale'] for c in cells):diag['stale']+=1;continue
        if any(bool(s['dynamicOccupiedRaw'][c]) for c in cells):diag['dynamic']+=1;continue
        pp=prepare(s,raw)
        if len(pp)==0:diag['path']+=1;continue
        tr=strict_traj(s,pp)
        if tr is None:diag['traj']+=1;continue
        if not stop(s,pp):diag['stop']+=1;continue
        diag['selected']=k;return diag,'s25_safe_informative_relocation'
    return diag,'recovery_candidates_failed_full_execution_checks'

def point_seg_dist(p,a,b):
    p=np.asarray(p,float);a=np.asarray(a,float);b=np.asarray(b,float);d=b-a
    if np.dot(d,d)<=1e-18:return float(np.linalg.norm(p-a))
    t=max(0.0,min(1.0,float(np.dot(p-a,d)/np.dot(d,d))))
    return float(np.linalg.norm(p-(a+t*d)))

def continuous_segment_safe(s,a,b,radius=None):
    if radius is None:
        with h5py.File(s['file'],'r') as h: radius=float(np.array(h['snapshot/grid/inflationRadius']).squeeze())
    a=np.asarray(a,float);b=np.asarray(b,float)
    # Convex room inset: if both endpoints are within it, the full segment is within it.
    lo=np.array([s['xs'][0]+radius,s['ys'][0]+radius]); hi=np.array([s['xs'][-1]-radius,s['ys'][-1]-radius])
    if np.any(a<lo-1e-12) or np.any(a>hi+1e-12) or np.any(b<lo-1e-12) or np.any(b>hi+1e-12):return False
    raw=s['grid_staticOccupied'].astype(bool)|s['grid_dynamicOccupied'].astype(bool)|s['grid_unknown'].astype(bool)
    cells=np.argwhere(raw)
    for y,x in cells:
        q=np.array([s['xs'][x],s['ys'][y]])
        if point_seg_dist(q,a,b)<=radius+1e-12:return False
    return True

def trajectory_hybrid_safe(s,tr,occ_orig):
    if tr is None:return False
    P=np.asarray(tr['P'])
    for i in range(len(P)-1):
        if not traj_semantics.segment_occupied(occ_orig,P[i],P[i+1],s['res']):continue
        if not continuous_segment_safe(s,P[i],P[i+1]):return False
    return True

def prepare_egress(s,raw):
    if len(raw)==0:return np.zeros((0,2)),None
    st=xy2cell(s,s['p'][:2]);occ=s['grid_occ'].astype(bool).copy()
    # Only current rounded cell may be exempt, and only after continuous start-to-first-free proof.
    q0=np.asarray(raw[0],float)
    if bool(occ[st]):
        if not continuous_segment_safe(s,s['p'][:2],q0):return np.zeros((0,2)),None
        occ[st]=False
    p=np.vstack([s['p'][:2],raw.astype(float)])
    out=[p[0]]
    for q in p[1:]:
        if np.linalg.norm(q-out[-1])>=s['res']/4:out.append(q)
    p=np.array(out,float);p[0]=s['p'][:2]
    sm=traj_semantics.smooth_path(occ,p)
    if len(sm):sm[0]=s['p'][:2]
    def valid(q):return len(q)>=2 and all(not traj_semantics.segment_occupied(occ,q[i],q[i+1],s['res']) for i in range(len(q)-1))
    path=sm if valid(sm) else (p if valid(p) else np.zeros((0,2)))
    if len(path)==0:return path,None
    # Any segment blocked on original grid must be continuously metric-safe.
    orig=s['grid_occ'].astype(bool)
    for i in range(len(path)-1):
        if traj_semantics.segment_occupied(orig,path[i],path[i+1],s['res']) and not continuous_segment_safe(s,path[i],path[i+1]):
            return np.zeros((0,2)),None
    return path,occ

def strict_traj_occ(s,path,occ):
    scale=max(s['initialScale'],np.finfo(float).eps)
    for _ in range(5):
        tr=traj_semantics.generate(occ,path,startV=s['v'][:2],startA=s['a'][:2],initial=scale)
        if tr is None:return None
        ratios=[tr['maxSpeed']/CFG['maxSpeed'],math.sqrt(tr['maxAccel']/CFG['maxAccel']),np.cbrt(tr['maxJerk']/CFG['maxJerk'])]
        if all(rr<=1+1e-9 for rr in ratios):
            if trajectory_hybrid_safe(s,tr,s['grid_occ'].astype(bool)):return tr
            return None
        scale=scale*1.002*max(ratios)
    return None

def endpoint_with_egress(s):
    start=xy2cell(s,s['p'][:2]);diag={'ranked':0,'metric':0,'stale':0,'dynamic':0,'path':0,'traj':0,'stop':0,'selected':0,'egress':False}
    # if current cell blocked, require actual continuous start certificate
    if bool(s['navigationBlocked'][start]):
        # start point must be known free raw and continuously clear under intended metric radius
        if not bool(s['knownFree'][start]):return diag,None
        # A connector will be certified per candidate A* start.
        diag['egress']=True
    elif not bool(s['knownFree'][start]):return diag,None
    rows=rank(s,shell(s))
    if not rows:rows=rank(s,s['knownFree'].astype(bool)&~s['navigationBlocked'].astype(bool))
    diag['ranked']=len(rows)
    for k,rr in enumerate(rows,1):
        y,x=rr[3],rr[4];target=np.array([s['xs'][x],s['ys'][y]])
        raw,_=astar(s,s['p'][:2],target)
        if len(raw)==0:diag['metric']+=1;continue
        cells=metric_cells(s,raw)
        if not cells:diag['metric']+=1;continue
        age=s['timestamp']-s['lastObservedXY']
        if any(age[c]>s['stale'] for c in cells):diag['stale']+=1;continue
        if any(bool(s['dynamicOccupiedRaw'][c]) for c in cells):diag['dynamic']+=1;continue
        pp,occ=prepare_egress(s,raw)
        if len(pp)==0:diag['path']+=1;continue
        tr=strict_traj_occ(s,pp,occ)
        if tr is None:diag['traj']+=1;continue
        if not stop(s,pp):diag['stop']+=1;continue
        diag['selected']=k
        return diag,{'target':target,'cell':(y,x),'visible':rr[5],'path':pp,'traj':tr}
    return diag,None

# ---------------------------------------------------------------------------
# v0.2.0 exact-snapshot architecture: CSE + hierarchical SIE
# ---------------------------------------------------------------------------

def hazard_coords_fast(s):
    raw=s['grid_staticOccupied'].astype(bool)|s['grid_dynamicOccupied'].astype(bool)|s['grid_unknown'].astype(bool)
    cells=np.argwhere(raw)
    if len(cells)==0:return np.zeros((0,2))
    return np.column_stack([s['xs'][cells[:,1]],s['ys'][cells[:,0]]])

def continuous_segment_safe_fast(s,a,b,haz=None):
    radius=float(s['inflationRadius']);a=np.asarray(a,float);b=np.asarray(b,float)
    lo=np.array([s['xs'][0]+radius,s['ys'][0]+radius]);hi=np.array([s['xs'][-1]-radius,s['ys'][-1]-radius])
    if np.any(a<lo-1e-12) or np.any(a>hi+1e-12) or np.any(b<lo-1e-12) or np.any(b>hi+1e-12):return False
    if haz is None:haz=hazard_coords_fast(s)
    if len(haz)==0:return True
    d=b-a;den=float(np.dot(d,d))
    if den<=1e-18:
        md=float(np.min(np.linalg.norm(haz-a,axis=1)))
    else:
        t=np.clip(((haz-a)@d)/den,0.0,1.0)
        proj=a+t[:,None]*d
        md=float(np.min(np.linalg.norm(haz-proj,axis=1)))
    # project_map_to_planner_S2_3 marks raw-node distance <= radius occupied.
    return md>radius+1e-12

def connector_certificate(s):
    st=xy2cell(s,s['p'][:2]);orig=s['grid_occ'].astype(bool)
    if not orig[st]:return True,None,0.0
    if not bool(s['knownFree'][st]):return False,None,float('inf')
    ix0=mround(s['p'][0]/s['res']);iy0=mround(s['p'][1]/s['res'])
    bx,by,ok=nearest_free(orig,ix0,iy0,12)
    if not ok:return False,None,float('inf')
    q=np.array([bx*s['res'],by*s['res']],float)
    ok=continuous_segment_safe_fast(s,s['p'][:2],q)
    return bool(ok),q,float(np.linalg.norm(q-s['p'][:2]))

def landing_clear_cell(s,c):
    return landing_clear(s,np.array([s['xs'][c[1]],s['ys'][c[0]]],float))

def route_cells_current(s,raw):
    cells=metric_cells(s,raw)
    if not cells:return False
    age=s['timestamp']-s['lastObservedXY']
    return all(age[c]<=s['stale']+1e-12 for c in cells) and all(not bool(s['dynamicOccupiedRaw'][c]) for c in cells)

def prepare_preserve_anchor_exact(s,raw,anchor_cell):
    if len(raw)<1:return None,None
    cells=[xy2cell(s,p) for p in raw]
    if anchor_cell not in cells:return None,None
    idx=cells.index(anchor_cell)
    st=xy2cell(s,s['p'][:2]);orig=s['grid_occ'].astype(bool);occ=orig.copy();haz=hazard_coords_fast(s)
    if orig[st]:
        if not continuous_segment_safe_fast(s,s['p'][:2],raw[0],haz):return None,None
        # This is the ONLY occupancy exemption. It exists solely for the current
        # rounded cell, and every trajectory segment touching it is re-certified
        # against the same continuous metric inflation geometry below.
        occ[st]=False
    def rm(p):
        p=np.asarray(p,float);out=[p[0]]
        for q in p[1:]:
            if np.linalg.norm(q-out[-1])>=s['res']/4:out.append(q)
        return np.asarray(out,float)
    leg1raw=rm(np.vstack([s['p'][:2],raw[:idx+1]]))
    leg2raw=rm(raw[idx:])
    if len(leg1raw)>=2:
        leg1=traj_semantics.smooth_path(occ,leg1raw);leg1[0]=s['p'][:2]
    else:leg1=leg1raw
    leg2=traj_semantics.smooth_path(occ,leg2raw) if len(leg2raw)>=2 else leg2raw
    if len(leg1)<2 and len(leg2)<2:return None,None
    if len(leg1)<2:path=leg2
    elif len(leg2)<2:path=leg1
    else:path=np.vstack([leg1,leg2[1:]])
    for i in range(len(path)-1):
        if traj_semantics.segment_occupied(occ,path[i],path[i+1],s['res']):return None,None
        if traj_semantics.segment_occupied(orig,path[i],path[i+1],s['res']):
            if not continuous_segment_safe_fast(s,path[i],path[i+1],haz):return None,None
    return path,occ

def strict_hybrid_exact(s,path,occ):
    scale=max(s['initialScale'],np.finfo(float).eps);orig=s['grid_occ'].astype(bool);haz=hazard_coords_fast(s)
    for _ in range(5):
        tr=traj_semantics.generate(occ,path,startV=s['v'][:2],startA=s['a'][:2],initial=scale)
        if tr is None:return None
        ratios=[tr['maxSpeed']/CFG['maxSpeed'],math.sqrt(tr['maxAccel']/CFG['maxAccel']),np.cbrt(tr['maxJerk']/CFG['maxJerk'])]
        if all(r<=1+1e-9 for r in ratios):
            P=np.asarray(tr['P'])
            for i in range(len(P)-1):
                if traj_semantics.segment_occupied(orig,P[i],P[i+1],s['res']):
                    if not continuous_segment_safe_fast(s,P[i],P[i+1],haz):return None
            return tr
        scale=scale*1.002*max(ratios)
    return None

def visible_count_cached(s,c,cache):
    if c not in cache:cache[c]=len(visible(s,c,0.0))
    return cache[c]

def stop_safe_terminals(s):
    allowed=s['knownFree'].astype(bool)&~s['navigationBlocked'].astype(bool);out=[]
    for y,x in np.argwhere(allowed):
        c=(int(y),int(x))
        if hold(s,c) and landing_clear_cell(s,c):out.append(c)
    return out

def build_path_via_anchor(s,raw1,anchor,raw2=None):
    if raw2 is None or len(raw2)<=1:raw=np.asarray(raw1,float)
    else:raw=np.vstack([raw1,np.asarray(raw2,float)[1:]])
    return prepare_preserve_anchor_exact(s,raw,anchor)

def path_len(path):
    return float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()) if path is not None and len(path)>=2 else 0.0

def terminal_xy(s,c):return np.array([s['xs'][c[1]],s['ys'][c[0]]],float)

def hierarchical_sie(s):
    if not np.any(s['unknown']):
        return {'success':False,'stage':'NO_INFORMATION','reason':'no_unknown_cells_visible_in_belief'}
    cert,q,connectorLen=connector_certificate(s)
    if not cert:return {'success':False,'stage':'CSE_REJECT','reason':'continuous_start_connector_invalid'}
    start=xy2cell(s,s['p'][:2]);terms=stop_safe_terminals(s);cache={}
    # Stage 1: choose a fully stop-safe terminal whose CURRENT A* route itself
    # contains at least one informative transit cell. Preserve the best transit
    # cell when smoothing so the information opportunity cannot be shortcut away.
    stage1=[]
    for term in terms:
        raw,_=astar(s,s['p'][:2],terminal_xy(s,term))
        if len(raw)<2 or not route_cells_current(s,raw):continue
        bestGain=0;anchor=None
        for p in raw:
            c=xy2cell(s,p);g=visible_count_cached(s,c,cache)
            if g>bestGain:bestGain=g;anchor=c
        if bestGain<s['minvis'] or anchor is None:continue
        L=float(np.linalg.norm(np.diff(raw,axis=0),axis=1).sum())+connectorLen
        gd=float(np.linalg.norm(terminal_xy(s,term)-s['goal']))
        stage1.append((-bestGain,L,gd,term,anchor,raw))
    stage1.sort(key=lambda z:(z[0],z[1],z[2],z[3],z[4]))
    reject1=collections.Counter()
    for rank,item in enumerate(stage1,1):
        negGain,L,gd,term,anchor,raw=item
        path,occ=build_path_via_anchor(s,raw,anchor)
        if path is None:reject1['path']+=1;continue
        if path_len(path)+1e-12<min(s['radii']):reject1['relocation']+=1;continue
        tr=strict_hybrid_exact(s,path,occ)
        if tr is None:reject1['trajectory']+=1;continue
        if not stop(s,path):reject1['stop']+=1;continue
        return {'success':True,'stage':'SIE_STAGE1_STOP_FIRST','rank':rank,'gain':-negGain,'anchor':anchor,'terminal':term,
                'path':path,'traj':tr,'connectorLen':connectorLen,'startCellBlocked':bool(s['grid_occ'][start]),
                'stage1Candidates':len(stage1),'stage1Rejects':dict(reject1),'terminals':len(terms)}
    # Stage 2: no stop-safe route was informative enough. Search an informative
    # reachable transit anchor, then detour to a separate stop-safe terminal.
    allowed=s['knownFree'].astype(bool)&~s['navigationBlocked'].astype(bool);anchors=[]
    for y,x in np.argwhere(allowed):
        a=(int(y),int(x));gain=visible_count_cached(s,a,cache)
        if gain<s['minvis']:continue
        raw1,_=astar(s,s['p'][:2],terminal_xy(s,a))
        if len(raw1)<2 or not route_cells_current(s,raw1):continue
        L=float(np.linalg.norm(np.diff(raw1,axis=0),axis=1).sum())+connectorLen
        gd=float(np.linalg.norm(terminal_xy(s,a)-s['goal']))
        anchors.append((-gain,L,gd,a,raw1))
    anchors.sort(key=lambda z:(z[0],z[1],z[2],z[3]));reject2=collections.Counter()
    for arank,item in enumerate(anchors,1):
        negGain,L,gd,anchor,raw1=item;axy=terminal_xy(s,anchor)
        order=sorted(terms,key=lambda t:(math.hypot(t[0]-anchor[0],t[1]-anchor[1]),float(np.linalg.norm(terminal_xy(s,t)-s['goal'])),t))
        for trank,term in enumerate(order,1):
            raw2,_=astar(s,axy,terminal_xy(s,term))
            if len(raw2)==0:reject2['terminal_metric']+=1;continue
            if len(raw2)>1 and not route_cells_current(s,raw2):reject2['terminal_route']+=1;continue
            path,occ=build_path_via_anchor(s,raw1,anchor,raw2)
            if path is None:reject2['path']+=1;continue
            if path_len(path)+1e-12<min(s['radii']):reject2['relocation']+=1;continue
            tr=strict_hybrid_exact(s,path,occ)
            if tr is None:reject2['trajectory']+=1;continue
            if not stop(s,path):reject2['stop']+=1;continue
            return {'success':True,'stage':'SIE_STAGE2_TRANSIT_DETOUR','anchorRank':arank,'terminalRank':trank,'gain':-negGain,
                    'anchor':anchor,'terminal':term,'path':path,'traj':tr,'connectorLen':connectorLen,
                    'startCellBlocked':bool(s['grid_occ'][start]),'stage1Candidates':len(stage1),'stage1Rejects':dict(reject1),
                    'anchors':len(anchors),'stage2Rejects':dict(reject2),'terminals':len(terms)}
    return {'success':False,'stage':'NO_EXECUTABLE_SIE','stage1Candidates':len(stage1),'stage1Rejects':dict(reject1),
            'anchors':len(anchors),'stage2Rejects':dict(reject2),'terminals':len(terms)}

def load_capture_diagnostics(report_path):
    out={}
    with h5py.File(report_path,'r') as h:
        refs=np.array(h['report/rows']).flatten(order='F')
        for ref in refs:
            row=h[ref];sc=read_char(row['summary/scenario']);d=row['summary/lastInformativeRecoveryDiagnostics']
            out[sc]={k:(bool(np.array(d[k]).squeeze()) if np.array(d[k]).dtype==np.uint8 else float(np.array(d[k]).squeeze())) for k in d.keys()}
    return out

def compare_old_to_matlab(s,matdiag):
    d,reason=old_replay(s)
    mapping={'configError':'configError','shellFallback':'shellFallbackToAllKnownFree','ranked':'rankedCandidates',
             'metric':'metricRouteRejected','stale':'staleRouteRejected','dynamic':'dynamicRouteRejected','path':'preparedPathRejected',
             'traj':'trajectoryRejected','stop':'stopRejected','selected':'selectedRank'}
    mism=[]
    for py,mk in mapping.items():
        if float(d[py])!=float(matdiag[mk]):mism.append((py,d[py],mk,matdiag[mk]))
    return d,reason,mism

def safety_certificate(s,result):
    assert result['success'];path=result['path'];tr=result['traj'];st=xy2cell(s,s['p'][:2]);orig=s['grid_occ'].astype(bool);haz=hazard_coords_fast(s)
    # Every path segment is either standard-grid executable or belongs to the
    # single continuously certified current-cell egress exception.
    hybrid=0
    for i in range(len(path)-1):
        if traj_semantics.segment_occupied(orig,path[i],path[i+1],s['res']):
            hybrid+=1;assert continuous_segment_safe_fast(s,path[i],path[i+1],haz)
    # The terminal is still subject to inherited hold and landing gates.
    assert hold(s,result['terminal']) and landing_clear_cell(s,result['terminal']) and stop(s,path)
    assert tr['maxSpeed']<=CFG['maxSpeed']*(1+1e-9)
    assert tr['maxAccel']<=CFG['maxAccel']*(1+1e-9)
    assert tr['maxJerk']<=CFG['maxJerk']*(1+1e-9)
    # No trajectory segment may use any blocked cell other than the one rounded
    # current cell, and those segments must be continuously metric-clear.
    for i in range(len(tr['P'])-1):
        if traj_semantics.segment_occupied(orig,tr['P'][i],tr['P'][i+1],s['res']):
            assert bool(orig[st])
            assert continuous_segment_safe_fast(s,tr['P'][i],tr['P'][i+1],haz)
    return hybrid

def scenario_short(sc):
    return {'S25_NAV_PRIMARY_IMU_FAULT_VIO_OUTAGE':'NAV_IMU_FAULT_VIO_OUTAGE',
            'S25_PERCEPTION_BRIEF_DUAL_DROPOUT':'PERCEPTION_DUAL_BRIEF',
            'S25_PERCEPTION_STALE_PACKET_BURST':'PERCEPTION_STALE_BURST',
            'S25_PERCEPTION_RANGE_SPIKE':'PERCEPTION_RANGE_SPIKE',
            'S25_COUPLED_IMU_FAULT_PERCEPTION_DROPOUT':'COUPLED_IMU_PERCEPTION'}.get(sc,sc)

def main():
    mats=sorted(glob.glob(os.path.join(BASE,'s25_*.mat')));assert len(mats)==5
    report=os.path.join(BASE,'S2_5_recovery_snapshot_capture_report.mat');matdiag=load_capture_diagnostics(report)
    print('='*88);print('S2.5 EXACT MATLAB SNAPSHOT REPLAY — CSE + HIERARCHICAL SAFE INFORMATIVE EXCURSION');print('='*88)
    exact=[]
    for f in mats:
        s=load(f);old,oldReason,mism=compare_old_to_matlab(s,matdiag[s['scenario']]);assert not mism,(s['scenario'],mism)
        result=hierarchical_sie(s);assert result['success'],s['scenario'];hybrid=safety_certificate(s,result)
        start=xy2cell(s,s['p'][:2]);cert,q,conn=connector_certificate(s)
        wall=min(s['p'][0]-s['xs'][0],s['xs'][-1]-s['p'][0],s['p'][1]-s['ys'][0],s['ys'][-1]-s['p'][1])
        row={'scenario':scenario_short(s['scenario']),'oldReason':oldReason,'old':old,'newStage':result['stage'],'gain':result['gain'],
             'anchor':result['anchor'],'terminal':result['terminal'],'pathLength_m':path_len(result['path']),'connector_m':conn,
             'startBlocked':bool(s['grid_occ'][start]),'wallClearance_m':wall,'inflation_m':s['inflationRadius'],
             'maxV':result['traj']['maxSpeed'],'maxA':result['traj']['maxAccel'],'maxJ':result['traj']['maxJerk'],'hybridPathSegments':hybrid}
        exact.append(row)
        print(f"\n[{row['scenario']}] old={oldReason}")
        print('  MATLAB/Python old diagnostics: MATCH',old)
        print(f"  startBlocked={int(row['startBlocked'])} wall={wall:.6f} m inflation={s['inflationRadius']:.3f} m connector={conn:.6f} m")
        print(f"  NEW {row['newStage']} gain={row['gain']} anchor={row['anchor']} terminal={row['terminal']} path={row['pathLength_m']:.3f} m")
        print(f"  trajectory vmax={row['maxV']:.6f} amax={row['maxA']:.6f} jmax={row['maxJ']:.6f} | safety=PASS")
    print('\nEXACT FIVE-CASE RESULT: PASS 5/5')

    # Negative controls.
    print('\n'+'-'*88);print('NEGATIVE CONTROLS')
    neg=0
    for f in mats:
        s=load(f);start=xy2cell(s,s['p'][:2])
        if bool(s['grid_occ'][start]):
            t=copy.deepcopy(s);t['p']=t['p'].copy();t['p'][0]=t['inflationRadius']-0.005
            ok,_,_=connector_certificate(t);assert not ok;neg+=1
            print(f"  {scenario_short(s['scenario'])}: inside-inflation start connector rejected PASS")
        t=copy.deepcopy(s);t['unknown']=np.zeros_like(t['unknown'])
        out=hierarchical_sie(t);assert not out['success'];neg+=1
        print(f"  {scenario_short(s['scenario'])}: zero-information map rejected PASS")
        t=copy.deepcopy(s);t['lastObservedXY']=np.full_like(t['lastObservedXY'],t['timestamp']-(t['stale']+1.0),dtype=float)
        out=hierarchical_sie(t);assert not out['success'];neg+=1
        print(f"  {scenario_short(s['scenario'])}: stale-route universe rejected PASS")
    print(f'NEGATIVE CONTROLS: PASS ({neg}/{neg})')

    # Randomized state perturbations around the exact captured snapshots.
    print('\n'+'-'*88);print('RANDOMIZED LOCAL STATE PERTURBATIONS (2 per case; fixed RNG seed 2505)')
    rng=np.random.default_rng(2505);passed=0;total=0
    for f in mats:
        base=load(f);casePass=0
        for j in range(2):
            s=copy.deepcopy(base);s['p']=s['p'].copy();s['v']=s['v'].copy();s['a']=s['a'].copy()
            s['p'][:2]+=rng.uniform(-0.003,0.003,2);s['v'][:2]+=rng.uniform(-0.001,0.001,2);s['a'][:2]+=rng.uniform(-0.002,0.002,2)
            result=hierarchical_sie(s);total+=1
            if result['success']:
                safety_certificate(s,result);passed+=1;casePass+=1
        print(f"  {scenario_short(base['scenario'])}: {casePass}/2")
    assert passed==total,(passed,total)
    print(f'RANDOMIZED PERTURBATIONS: PASS ({passed}/{total})')
    print('\nOVERALL PYTHON PHASE-C EXACT-REPLAY GATE: PASS')
    return exact

if __name__=='__main__':
    main()

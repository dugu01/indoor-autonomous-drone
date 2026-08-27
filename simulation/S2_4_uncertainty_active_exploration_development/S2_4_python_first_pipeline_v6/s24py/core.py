from __future__ import annotations
from dataclasses import dataclass, field, asdict
import heapq, math
import numpy as np
from scipy.ndimage import label, distance_transform_edt, binary_dilation

@dataclass(frozen=True)
class Config:
    room: tuple[float,float]=(6.0,6.0)
    resolution: float=0.1
    inflation: float=0.602
    lidar_range: float=6.5
    lidar_beams: int=360
    view_range: float=4.0
    view_fov_deg: float=100.0
    view_rays: int=101
    candidate_radii: tuple[float,...]=(0.7,1.0,1.3)
    candidate_angles: int=16
    min_frontier_cells: int=2
    max_frontier_extent_cells: int=18
    target_corridor_radius_cells: int=3
    unknown_cost: float=5.0
    min_visible_unknown: int=6
    progress_cos_min: float=0.25

@dataclass
class Grid:
    xs: np.ndarray
    ys: np.ndarray
    # truth_occ is SENSOR-SIMULATION ONLY. Decision functions below must not read it.
    truth_occ: np.ndarray
    known_free: np.ndarray
    known_occ: np.ndarray
    unknown: np.ndarray
    executable: np.ndarray
    resolution: float
    inflation: float

    def inside(self, rc: tuple[int,int]) -> bool:
        r,c=rc
        return 0<=r<self.known_free.shape[0] and 0<=c<self.known_free.shape[1]

@dataclass
class Frontier:
    track_id: int
    cells: np.ndarray
    centroid_rc: np.ndarray
    centroid_xy: np.ndarray

@dataclass
class Candidate:
    candidate_id: int
    frontier_track_id: int
    rc: tuple[int,int]
    xy: tuple[float,float]
    yaw: float
    path: list[tuple[int,int]] = field(default_factory=list)
    visible_unknown: list[tuple[int,int]] = field(default_factory=list)
    # Structural proxy only; NOT claimed numerically equal to MATLAB entropy IG.
    visible_unknown_proxy: float = 0.0
    target_relevance_proxy: float = 0.0
    tier: int = 3
    utility_proxy: float = -math.inf
    safety_reasons: list[str] = field(default_factory=list)
    policy_reasons: list[str] = field(default_factory=list)
    safety_feasible: bool = False
    accepted: bool = False

    def to_dict(self): return asdict(self)


def xy_to_rc(xy, xs, ys):
    x,y=xy
    c=int(np.argmin(np.abs(xs-x))); r=int(np.argmin(np.abs(ys-y)))
    return r,c

def rc_to_xy(rc, xs, ys):
    r,c=rc; return float(xs[c]),float(ys[r])

def point_in_rect(x,y,rect):
    rx,ry,w,h=rect
    return rx<=x<=rx+w and ry<=y<=ry+h

def build_truth(cfg:Config, rects:list[list[float]]) -> tuple[np.ndarray,np.ndarray,np.ndarray]:
    nx=int(round(cfg.room[0]/cfg.resolution))+1
    ny=int(round(cfg.room[1]/cfg.resolution))+1
    xs=np.arange(nx)*cfg.resolution; ys=np.arange(ny)*cfg.resolution
    occ=np.zeros((ny,nx),bool)
    occ[0,:]=occ[-1,:]=True; occ[:,0]=occ[:,-1]=True
    for r,y in enumerate(ys):
        for c,x in enumerate(xs):
            if any(point_in_rect(float(x),float(y),q) for q in rects): occ[r,c]=True
    return xs,ys,occ

def bresenham(a:tuple[int,int],b:tuple[int,int]):
    r0,c0=a; r1,c1=b
    dr=abs(r1-r0); dc=abs(c1-c0)
    sr=1 if r0<r1 else -1; sc=1 if c0<c1 else -1
    r,c=r0,c0; pts=[]
    if dc>dr:
        err=dc/2
        while c!=c1:
            pts.append((r,c)); err-=dr
            if err<0: r+=sr; err+=dc
            c+=sc
    else:
        err=dr/2
        while r!=r1:
            pts.append((r,c)); err-=dc
            if err<0: c+=sc; err+=dr
            r+=sr
    pts.append((r1,c1)); return pts

def ray_cells(origin_xy, angle, max_range, xs, ys):
    ex=origin_xy[0]+max_range*math.cos(angle); ey=origin_xy[1]+max_range*math.sin(angle)
    return bresenham(xy_to_rc(origin_xy,xs,ys),xy_to_rc((ex,ey),xs,ys))

def scan_truth(cfg:Config, xs,ys,truth_occ, poses:list[tuple[float,float]], yaws:list[float]|None=None):
    """Simulate range sensing from truth and expose only the estimated binary map."""
    known_free=np.zeros_like(truth_occ); known_occ=np.zeros_like(truth_occ)
    for pose in poses:
        for theta in np.linspace(-math.pi,math.pi,cfg.lidar_beams,endpoint=False):
            for rc in ray_cells(pose,theta,cfg.lidar_range,xs,ys):
                r,c=rc
                if not (0<=r<truth_occ.shape[0] and 0<=c<truth_occ.shape[1]): break
                if truth_occ[r,c]:
                    known_occ[r,c]=True; break
                known_free[r,c]=True
    unknown=~(known_free|known_occ)
    dist=distance_transform_edt(known_free)*cfg.resolution
    executable=known_free & (dist>=cfg.inflation-1e-9)
    return Grid(xs,ys,truth_occ,known_free,known_occ,unknown,executable,cfg.resolution,cfg.inflation)

def frontier_mask(grid:Grid):
    return grid.known_free & binary_dilation(grid.unknown,structure=np.ones((3,3),bool))

def split_large_cells(cells:np.ndarray,max_extent:int):
    if len(cells)==0:return []
    span=np.ptp(cells,axis=0)
    if max(span)<=max_extent:return [cells]
    axis=int(np.argmax(span)); cells=cells[np.argsort(cells[:,axis],kind='stable')]
    chunks=[]; start=0
    while start<len(cells):
        base=cells[start,axis]; end=start
        while end<len(cells) and cells[end,axis]-base<=max_extent: end+=1
        chunks.append(cells[start:end]); start=end
    return chunks

def extract_frontiers(grid:Grid,cfg:Config):
    lab,n=label(frontier_mask(grid),np.ones((3,3),int)); out=[];tid=1
    for i in range(1,n+1):
        cells=np.argwhere(lab==i)
        if len(cells)<cfg.min_frontier_cells:continue
        for sub in split_large_cells(cells,cfg.max_frontier_extent_cells):
            if len(sub)<cfg.min_frontier_cells:continue
            cent=sub.mean(axis=0); rc=np.round(cent).astype(int)
            out.append(Frontier(tid,sub,cent,np.array(rc_to_xy(tuple(rc),grid.xs,grid.ys)))); tid+=1
    return out

def astar(mask,start,goal,cost_unknown=None):
    ny,nx=mask.shape
    if not(0<=start[0]<ny and 0<=start[1]<nx and 0<=goal[0]<ny and 0<=goal[1]<nx):return []
    if cost_unknown is None and (not mask[start] or not mask[goal]):return []
    pq=[(0.0,0.0,start)]; g={start:0.0}; parent={}
    while pq:
        _,gc,cur=heapq.heappop(pq)
        if gc!=g.get(cur):continue
        if cur==goal:
            p=[cur]
            while cur in parent:cur=parent[cur];p.append(cur)
            return p[::-1]
        r,c=cur
        for dr,dc in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            rr,cc=r+dr,c+dc
            if not(0<=rr<ny and 0<=cc<nx):continue
            if cost_unknown is None:
                if not mask[rr,cc]:continue
                if dr and dc and (not mask[r,cc] or not mask[rr,c]):continue
                step=math.sqrt(2) if dr and dc else 1.0
            else:
                val=float(cost_unknown[rr,cc])
                if not math.isfinite(val):continue
                step=(math.sqrt(2) if dr and dc else 1.0)*val
            ng=gc+step; nxt=(rr,cc)
            if ng<g.get(nxt,math.inf):
                g[nxt]=ng;parent[nxt]=cur
                heapq.heappush(pq,(ng+math.hypot(rr-goal[0],cc-goal[1]),ng,nxt))
    return []

def target_corridor(grid:Grid,start,goal,cfg:Config):
    costs=np.full(grid.unknown.shape,math.inf,float)
    costs[grid.known_free]=1.0; costs[grid.unknown]=cfg.unknown_cost; costs[grid.known_occ]=math.inf
    p=astar(np.ones_like(grid.unknown,bool),start,goal,cost_unknown=costs)
    mask=np.zeros_like(grid.unknown)
    for rc in p:
        if grid.unknown[rc]:mask[rc]=True
    if cfg.target_corridor_radius_cells>0:mask=binary_dilation(mask,iterations=cfg.target_corridor_radius_cells)
    return mask,p

def visible_unknown_from(grid:Grid,xy,yaw,cfg:Config):
    """Conservative structural visibility from the ESTIMATED map only.

    Rays stop at known occupied cells and also at the first unknown cell. This
    intentionally avoids using truth to infer visibility behind unmapped space.
    The output is a structural frontier/visibility proxy, not MATLAB entropy IG.
    """
    cells=set(); half=math.radians(cfg.view_fov_deg)/2
    for a in np.linspace(yaw-half,yaw+half,cfg.view_rays):
        for rc in ray_cells(xy,a,cfg.view_range,grid.xs,grid.ys):
            r,c=rc
            if not(0<=r<grid.unknown.shape[0] and 0<=c<grid.unknown.shape[1]):break
            if grid.known_occ[r,c]:break
            if grid.unknown[r,c]:
                cells.add(rc)
                break
            if not grid.known_free[r,c]:break
    return sorted(cells)

def local_support(executable,rc):
    r,c=rc
    if r<1 or c<1 or r>=executable.shape[0]-1 or c>=executable.shape[1]-1:return False
    return bool(executable[r-1:r+2,c-1:c+2].all())

def candidate_points(frontier:Frontier,grid:Grid,cfg:Config):
    fxy=frontier.centroid_xy; pts=[]
    for rad in cfg.candidate_radii:
        for ang in np.linspace(0,2*math.pi,cfg.candidate_angles,endpoint=False):
            pts.append(xy_to_rc((float(fxy[0]+rad*math.cos(ang)),float(fxy[1]+rad*math.sin(ang))),grid.xs,grid.ys))
    if len(frontier.cells):
        idx=np.unique(np.linspace(0,len(frontier.cells)-1,min(12,len(frontier.cells))).round().astype(int))
        pts.extend(tuple(map(int,frontier.cells[i])) for i in idx)
    seen=set();out=[]
    for rc in pts:
        if rc not in seen:seen.add(rc);out.append(rc)
    return out

def evaluate_candidates(grid:Grid,frontiers,start_xy,goal_xy,cfg:Config):
    """Evaluate candidates without consulting grid.truth_occ."""
    start=xy_to_rc(start_xy,grid.xs,grid.ys); goal=xy_to_rc(goal_xy,grid.xs,grid.ys)
    corridor,_=target_corridor(grid,start,goal,cfg)
    goalvec=np.array(goal_xy)-np.array(start_xy); gn=np.linalg.norm(goalvec)
    out=[];cid=1
    for f in frontiers:
        for rc in candidate_points(f,grid,cfg):
            xy=rc_to_xy(rc,grid.xs,grid.ys); yaw=math.atan2(f.centroid_xy[1]-xy[1],f.centroid_xy[0]-xy[0])
            q=Candidate(cid,f.track_id,rc,xy,yaw);cid+=1
            if not grid.inside(rc):q.safety_reasons.append('OUTSIDE_MAP')
            elif not grid.executable[rc]:q.safety_reasons.append('POSITION_OCCUPIED_OR_UNKNOWN_INFLATED')
            if grid.inside(rc) and not local_support(grid.executable,rc):q.safety_reasons.append('STOPPING_SUPPORT_INVALID')
            path=astar(grid.executable,start,rc); q.path=path
            if not path:q.safety_reasons.append('UNREACHABLE_KNOWN_FREE')
            elif len(path)<2:q.safety_reasons.append('RETREAT_ROUTE_INVALID')
            if not q.safety_reasons:
                vis=visible_unknown_from(grid,xy,yaw,cfg);q.visible_unknown=vis
                q.visible_unknown_proxy=float(len(vis))
                if len(vis)<cfg.min_visible_unknown:q.safety_reasons.append('INSUFFICIENT_VISIBLE_UNKNOWN')
                q.target_relevance_proxy=float(sum(1 for cell in vis if corridor[cell]))
            if q.target_relevance_proxy>0:q.tier=1
            else:
                dv=f.centroid_xy-np.array(start_xy); dn=np.linalg.norm(dv)
                progress=(np.dot(dv,goalvec)/(dn*gn)) if dn>1e-9 and gn>1e-9 else -1
                q.tier=2 if (q.visible_unknown_proxy>=cfg.min_visible_unknown and progress>=cfg.progress_cos_min) else 3
            if q.tier==3:q.policy_reasons.append('IRRELEVANT_EXPLORATION')
            q.safety_feasible=not q.safety_reasons
            q.accepted=q.safety_feasible and not q.policy_reasons
            I=min(q.visible_unknown_proxy/20.0,1.5);G=min(q.target_relevance_proxy/10.0,1.5)
            T=(len(path)*cfg.resolution/10.0) if path else 1.5; Y=abs(math.atan2(math.sin(yaw),math.cos(yaw)))/math.pi
            q.utility_proxy=0.25*I+0.35*G-0.10*T-0.02*Y
            out.append(q)
    return out,corridor

def select_candidate(candidates:list[Candidate]):
    eligible=[q for q in candidates if q.accepted]
    if not eligible:return None
    return sorted(eligible,key=lambda q:(q.tier,-q.utility_proxy,-q.target_relevance_proxy,-q.visible_unknown_proxy,q.frontier_track_id,q.candidate_id))[0]

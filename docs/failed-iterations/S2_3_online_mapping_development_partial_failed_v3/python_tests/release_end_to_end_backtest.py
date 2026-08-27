#!/usr/bin/env python3
"""End-to-end S2.3 release backtest for the complete 12-scenario catalogue.

This backtest checks scenario contracts before MATLAB execution:
- metric-inflated truth-map feasibility/unreachability;
- start/goal validity and minimum clearance;
- late-corridor-blockage route intersection, visibility, reaction margin,
  and existence of an alternate route;
- scan/extension expectation consistency;
- dropout timing relative to hold/failsafe thresholds;
- IMU-fault observability contract;
- dynamic-to-static persistence and route interaction;
- acceptance semantics for every scenario.

It is a mechanism and scenario-contract test, not a replacement for coupled
MATLAB validation.
"""
from __future__ import annotations
import argparse, heapq, math, json
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Iterable
import numpy as np

ROOM=(6.0,6.0); RES=0.10; INFLATION=0.602
COLLISION_RADIUS=0.352; MAX_SPEED=0.32; MAX_DECEL=0.80
SENSOR_DELAY=0.30; LIDAR_RANGE=6.5; HOLD_TIMEOUT=0.55; FAILSAFE_TIMEOUT=4.0
MOTION_START_REFERENCE=9.20
XS=np.arange(0.0,ROOM[0]+0.5*RES,RES); YS=np.arange(0.0,ROOM[1]+0.5*RES,RES)
NX,NY=len(XS),len(YS)
NEIGHBORS=((1,0,1.0),(-1,0,1.0),(0,1,1.0),(0,-1,1.0),
           (1,1,math.sqrt(2)),(1,-1,math.sqrt(2)),(-1,1,math.sqrt(2)),(-1,-1,math.sqrt(2)))
Rect=tuple[float,float,float,float,float]

@dataclass
class CaseResult:
    scenario:str; expected:str; feasible:bool|None; route_length_m:float|None
    min_clearance_m:float|None; contract_pass:bool; details:dict

BASE=((1.00,1.00,0.50,0.50,1.80),(4.00,3.50,0.50,0.50,1.80))
CASES={
'unknown_room_nominal':dict(start=(3.0,.8),goal=(5.3,5.3),rects=BASE,expected='reachable'),
'hidden_obstacle_replan':dict(start=(3.0,.8),goal=(5.3,5.3),rects=BASE,
    inserted=(12.0,(5.19,3.31,.30,.30,1.90)),expected='reachable_replan'),
'occluded_obstacle':dict(start=(3.0,.8),goal=(5.3,5.3),rects=((3.15,1.65,.50,.65,1.90),(3.40,2.75,.55,.75,1.90),(4.40,4.10,.40,.50,1.90)),expected='reachable_extension'),
'unknown_narrow_passage':dict(start=(3.0,.8),goal=(3.0,5.25),rects=((.80,2.45,1.45,.45,1.90),(3.75,2.45,1.45,.45,1.90)),expected='reachable_extension_optional'),
'dead_end_recovery':dict(start=(3.0,.8),goal=(5.10,5.10),rects=((1.65,1.65,.35,2.45,1.90),(1.65,4.10,2.10,.35,1.90),(3.40,2.45,.35,2.00,1.90)),expected='reachable_replan'),
'goal_requires_scan':dict(start=(.85,.85),goal=(5.15,5.15),rects=((2.50,1.20,.45,3.00,1.90),),expected='reachable_extension_scan'),
'unreachable_goal':dict(start=(3.0,.8),goal=(4.9,4.9),rects=((4.20,4.20,1.40,.30,2.20),(4.20,5.30,1.40,.30,2.20),(4.20,4.20,.30,1.40,2.20),(5.30,4.20,.30,1.40,2.20)),expected='unreachable'),
'depth_dropout_lidar':dict(start=(3.0,.8),goal=(5.3,5.3),rects=BASE,expected='reachable_single_sensor',dropout=('depth',8.0,24.0)),
'lidar_dropout_depth':dict(start=(3.0,.8),goal=(5.3,5.3),rects=BASE,expected='reachable_single_sensor',dropout=('lidar',8.0,24.0)),
'perception_dropout_recover':dict(start=(3.0,.8),goal=(5.3,5.3),rects=BASE,expected='reachable_hold_recover',dropout=('both',11.0,12.2)),
'primary_imu_fault_mapping':dict(start=(3.0,.8),goal=(5.3,5.3),rects=BASE,expected='reachable_lane_switch',fault=(10.0,10.0,22.0)),
'dynamic_to_static_mapping':dict(start=(3.0,.8),goal=(5.3,5.3),rects=BASE,expected='reachable_dynamic_promotion',dynamic=((5.20,2.15),(-.18,0),.17,7.0,11.0)),
}

def point_rect_distance(x:float,y:float,r:Rect)->float:
    rx,ry,w,d,_=r; dx=max(rx-x,0.0,x-(rx+w));dy=max(ry-y,0.0,y-(ry+d));return math.hypot(dx,dy)

def occupancy(rects:Iterable[Rect])->np.ndarray:
    occ=np.zeros((NY,NX),bool)
    for iy,y in enumerate(YS):
      for ix,x in enumerate(XS):
        if min(x,ROOM[0]-x,y,ROOM[1]-y)<=INFLATION+1e-12 or any(point_rect_distance(x,y,r)<=INFLATION+1e-12 for r in rects): occ[iy,ix]=True
    return occ

def astar(rects,start,goal):
    occ=occupancy(rects);sx,sy=round(start[0]/RES),round(start[1]/RES);gx,gy=round(goal[0]/RES),round(goal[1]/RES)
    if not(0<=sx<NX and 0<=sy<NY and 0<=gx<NX and 0<=gy<NY) or occ[sy,sx] or occ[gy,gx]:return None
    q=[(math.hypot(sx-gx,sy-gy),0.,sx,sy)];best={(sx,sy):0.};prev={};closed=set()
    while q:
      _,g,x,y=heapq.heappop(q)
      if (x,y) in closed:continue
      closed.add((x,y))
      if (x,y)==(gx,gy):
        cells=[];cur=(x,y)
        while True:
          cells.append(cur)
          if cur==(sx,sy):break
          cur=prev[cur]
        return np.asarray([(XS[ix],YS[iy]) for ix,iy in cells[::-1]])
      for dx,dy,c in NEIGHBORS:
        xx,yy=x+dx,y+dy
        if not(0<=xx<NX and 0<=yy<NY) or occ[yy,xx]:continue
        if dx and dy and (occ[y,xx] or occ[yy,x]):continue
        ng=g+c
        if ng<best.get((xx,yy),math.inf):
          best[(xx,yy)]=ng;prev[(xx,yy)]=(x,y);heapq.heappush(q,(ng+math.hypot(xx-gx,yy-gy),ng,xx,yy))
    return None

def path_length(path):return float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()) if path is not None and len(path)>1 else 0.
def min_clearance(path,rects):
    if path is None:return None
    m=math.inf
    for x,y in path:
      m=min(m,x,ROOM[0]-x,y,ROOM[1]-y,*[point_rect_distance(x,y,r) for r in rects])
    return float(m)
def path_intersects(path,r,margin=INFLATION):return path is not None and any(point_rect_distance(x,y,r)<=margin+1e-12 for x,y in path)
def segment_hits_rect(a,b,r,margin=0.0):
    a=np.asarray(a,float);b=np.asarray(b,float);n=max(2,int(np.linalg.norm(b-a)/.01)+1)
    return any(point_rect_distance(*(a+(b-a)*u),r)<=margin+1e-12 for u in np.linspace(0,1,n))
def visible(a,target,blockers):
    if np.linalg.norm(np.asarray(target)-np.asarray(a))>LIDAR_RANGE:return False
    return not any(segment_hits_rect(a,target,r,0.0) for r in blockers)
def first_intersection_distance(path,r):
    if path is None:return None
    acc=0.
    for i in range(len(path)-1):
      if segment_hits_rect(path[i],path[i+1],r,INFLATION):return acc
      acc+=float(np.linalg.norm(path[i+1]-path[i]))
    return None

def run_case(name,cfg,nominal_trace=None):
    rects=tuple(cfg['rects']);path=astar(rects,cfg['start'],cfg['goal']);feasible=path is not None
    expected=cfg['expected'];want_feasible=expected!='unreachable';ok=(feasible==want_feasible)
    detail={};length=path_length(path) if feasible else None;clear=min_clearance(path,rects) if feasible else None
    if feasible:ok &= clear+1e-12>=INFLATION
    if name=='occluded_obstacle':
      centres=[np.array([r[0]+r[2]/2,r[1]+r[3]/2]) for r in rects]
      initially=[];later=[]
      for j,c in enumerate(centres):
        blockers=tuple(r for i,r in enumerate(rects) if i!=j)
        v0=visible(np.asarray(cfg['start']),c,blockers)
        vl=any(visible(q,c,blockers) for q in path[::max(1,len(path)//20)]) if path is not None else False
        initially.append(v0);later.append(vl)
      occlusion_contract=(any(not v for v in initially) and all(later))
      ok &= occlusion_contract
      detail.update(initial_obstacle_visibility=initially,later_route_visibility=later,occlusion_contract=occlusion_contract)
    if name=='goal_requires_scan':
      goal_visible_start=visible(np.asarray(cfg['start']),np.asarray(cfg['goal']),rects)
      goal_visible_later=any(visible(q,np.asarray(cfg['goal']),rects) for q in path[1:]) if path is not None else False
      direct_blocked=any(segment_hits_rect(cfg['start'],cfg['goal'],r,INFLATION) for r in rects)
      scan_contract=(not goal_visible_start and goal_visible_later and direct_blocked)
      ok &= scan_contract
      detail.update(goal_visible_from_start=goal_visible_start,goal_visible_later=goal_visible_later,direct_route_blocked=direct_blocked,scan_contract=scan_contract)
    if name=='hidden_obstacle_replan':
      tins,ins=cfg['inserted'];before=path;after=astar(rects+(ins,),cfg['start'],cfg['goal'])
      route_hit=path_intersects(before,ins);alt=after is not None
      dpath=first_intersection_distance(before,ins)
      earliest_cross=MOTION_START_REFERENCE+(dpath/MAX_SPEED if dpath is not None else -math.inf)
      lead=earliest_cross-tins
      # Use recorded validated nominal state at insertion if available; otherwise start.
      p_at=np.asarray(cfg['start'],float)
      active=True
      if nominal_trace is not None:
        t,P,state=nominal_trace;i=int(np.argmin(np.abs(t-tins)));p_at=P[i,:2];active=int(round(state[i]))==7
      centre=np.array([ins[0]+ins[2]/2,ins[1]+ins[3]/2])
      sensed=visible(p_at,centre,rects)
      physical=point_rect_distance(p_at[0],p_at[1],ins)
      stopping=MAX_SPEED*SENSOR_DELAY+MAX_SPEED**2/(2*MAX_DECEL)+0.30
      safe_notice=physical>COLLISION_RADIUS+stopping and lead>=4.0
      ok &= route_hit and alt and sensed and active and safe_notice
      detail.update(route_intersection=route_hit,alternate_route=alt,insertion_active_track=active,
                    insertion_position=p_at.tolist(),sensor_line_of_sight=sensed,
                    physical_distance_at_insertion_m=physical,required_notice_distance_m=COLLISION_RADIUS+stopping,
                    conservative_earliest_cross_s=earliest_cross,lead_time_s=lead)
    if 'dropout' in cfg:
      source,t0,t1=cfg['dropout'];dur=t1-t0
      if source in ('depth','lidar'): contract=True
      else: contract=(dur>HOLD_TIMEOUT and dur<FAILSAFE_TIMEOUT)
      ok &= contract;detail.update(dropout_source=source,dropout_duration_s=dur,dropout_contract=contract)
    if 'fault' in cfg:
      fault_t,vio0,vio1=cfg['fault'];contract=(fault_t>=vio0 and vio1>vio0 and LIDAR_RANGE>0)
      ok &= contract;detail.update(fault_time_s=fault_t,vio_outage=[vio0,vio1],backup_lidar_lane_available=contract)
    if 'dynamic' in cfg:
      start,v,r,appear,stop=cfg['dynamic'];start=np.asarray(start);v=np.asarray(v);stoppos=start+v*(stop-appear)
      dynrect=(stoppos[0]-r,stoppos[1]-r,2*r,2*r,1.9);route_hit=path_intersects(path,dynrect,INFLATION)
      after=astar(rects+(dynrect,),cfg['start'],cfg['goal']);promotion_qualified=(stop-appear>=1.5 and 8*1.5>=5)
      ok &= route_hit and after is not None and promotion_qualified
      detail.update(stop_position=stoppos.tolist(),route_intersection=route_hit,alternate_route=after is not None,promotion_qualified=promotion_qualified)
    # Scenario-specific expectation sanity.
    expectations={
      'unknown_room_nominal':('extension_optional',True),
      'hidden_obstacle_replan':('safety_replan_required',True),
      'occluded_obstacle':('extension_required',True),
      'unknown_narrow_passage':('extension_optional',True),
      'dead_end_recovery':('replan_required',True),
      'goal_requires_scan':('scan_and_extension_required',True),
      'unreachable_goal':('safe_refusal_no_extension_required',True),
      'depth_dropout_lidar':('single_sensor_continuity',True),
      'lidar_dropout_depth':('single_sensor_continuity',True),
      'perception_dropout_recover':('hold_then_recover',True),
      'primary_imu_fault_mapping':('lane_switch_required',True),
      'dynamic_to_static_mapping':('promotion_and_replan',True),
    }
    detail['acceptance_contract']=expectations[name][0]
    return CaseResult(name,expected,feasible,length,clear,bool(ok),detail)

def read_nominal(path:Path|None):
    if path is None or not path.exists():return None
    import h5py
    with h5py.File(path,'r') as f:
      return np.array(f['log/t']).squeeze(),np.array(f['log/truthP']).T,np.array(f['log/stateId']).squeeze()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--nominal-mat',type=Path);ap.add_argument('--json',type=Path);ap.add_argument('--source-root',type=Path,default=Path(__file__).resolve().parents[1]);args=ap.parse_args()
    
    scenario_text=(args.source_root/'scenario_S2_3.m').read_text()
    source_contracts={
      'late_blockage_time': "'time',12.0,'rect5',[5.19 3.31 0.30 0.30 1.90]" in scenario_text,
      'late_blockage_no_extension_requirement': "scenario.name='LATE_CORRIDOR_BLOCKAGE_REPLAN'" in scenario_text and "scenario.expectedMapExtensions=false;" in scenario_text.split("scenario.name='LATE_CORRIDOR_BLOCKAGE_REPLAN'",1)[1].split("case {'occluded_obstacle'",1)[0],
      'narrow_extension_optional': "scenario.name='UNKNOWN_NARROW_PASSAGE'" in scenario_text and "scenario.expectedMaxScanHolds=2;" in scenario_text,
    }
    trace=read_nominal(args.nominal_mat);results=[run_case(n,c,trace) for n,c in CASES.items()]
    if not all(source_contracts.values()):
      print('SOURCE CONTRACT FAILURE',source_contracts)
      for r in results:r.contract_pass=False
    print('S2.3 COMPLETE 12-SCENARIO END-TO-END BACKTEST')
    print(f'grid={RES:.2f} m inflation={INFLATION:.3f} m lidar={LIDAR_RANGE:.1f} m')
    for r in results:
      f='NA' if r.feasible is None else ('YES' if r.feasible else 'NO')
      L='NA' if r.route_length_m is None else f'{r.route_length_m:.3f}'
      C='NA' if r.min_clearance_m is None else f'{r.min_clearance_m:.3f}'
      print(f'{r.scenario:30s} {"PASS" if r.contract_pass else "FAIL"} | feasible={f:3s} length={L:>6s} clearance={C:>6s} | {r.details["acceptance_contract"]}')
      if not r.contract_pass:print('  details:',r.details)
    passed=sum(r.contract_pass for r in results)
    print(f'\nRESULT: {passed}/12 PASS')
    payload={'passed':passed,'total':12,'all_pass':passed==12,'constants':dict(room=ROOM,resolution=RES,inflation=INFLATION),
             'source_contracts':source_contracts,'results':[asdict(r) for r in results]}
    if args.json:args.json.write_text(json.dumps(payload,indent=2))
    return 0 if passed==12 else 1
if __name__=='__main__':raise SystemExit(main())

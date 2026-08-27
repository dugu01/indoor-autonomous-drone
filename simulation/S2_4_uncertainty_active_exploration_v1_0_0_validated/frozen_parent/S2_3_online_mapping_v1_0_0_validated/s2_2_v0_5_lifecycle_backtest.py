#!/usr/bin/env python3
"""Focused Python backtest for Stage S2.2 v0.5 mission lifecycle.

This is an independent high-level state-machine and grid-planning regression.
It does not reproduce the MATLAB 6-DOF dynamics or four-lane ESKF.
"""
from __future__ import annotations
import argparse, heapq, json, math, random
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Iterable

ROOM=(6.0,6.0); RES=0.10; INFLATION=0.602; GROUND=0.03; CRUISE_Z=1.15
KNOWN=[(1.0,1.0,0.5,0.5),(4.0,3.5,0.5,0.5)]
HOME=(3.0,0.8); GOAL=(5.3,5.3); ALTERNATES=[(5.1,0.9),(0.9,5.1),(0.9,3.0)]
DT=0.05

@dataclass
class RunResult:
    scenario: str; seed: int; passed: bool
    armed: bool=False; takeoff: bool=False; goal: bool=False; rtl: bool=False
    landed: bool=False; disarmed: bool=False; preflight_reject: bool=False
    emergency_land: bool=False; alternate_land: bool=False; rtl_replans: int=0
    collision: int=0; min_clearance: float=math.inf; final_xy: tuple[float,float]=(math.nan,math.nan)
    selected_landing: tuple[float,float]=(math.nan,math.nan); duration_s: float=0.0

class Grid:
    def __init__(self, obstacles: Iterable[tuple[float,float,float,float]]):
        self.obstacles=list(obstacles); self.nx=round(ROOM[0]/RES)+1; self.ny=round(ROOM[1]/RES)+1
        self.occ=[[False]*self.nx for _ in range(self.ny)]
        for iy in range(self.ny):
            y=iy*RES
            for ix in range(self.nx):
                x=ix*RES
                if x<INFLATION or x>ROOM[0]-INFLATION or y<INFLATION or y>ROOM[1]-INFLATION:
                    self.occ[iy][ix]=True; continue
                for r in self.obstacles:
                    if dist_rect((x,y),r)<=INFLATION:
                        self.occ[iy][ix]=True; break
    def cell(self,p): return (round(p[1]/RES),round(p[0]/RES))
    def blocked(self,p):
        iy,ix=self.cell(p)
        return not(0<=iy<self.ny and 0<=ix<self.nx) or self.occ[iy][ix]

def dist_rect(p,r):
    x,y=p; rx,ry,w,h=r
    dx=max(rx-x,0.0,x-(rx+w)); dy=max(ry-y,0.0,y-(ry+h))
    return math.hypot(dx,dy)

def astar(grid:Grid,start,goal):
    s=grid.cell(start); g=grid.cell(goal)
    if grid.blocked(start) or grid.blocked(goal): return []
    pq=[(0.0,s)]; cost={s:0.0}; parent={}
    nbr=[(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    while pq:
        _,u=heapq.heappop(pq)
        if u==g: break
        for dy,dx in nbr:
            v=(u[0]+dy,u[1]+dx)
            if not(0<=v[0]<grid.ny and 0<=v[1]<grid.nx) or grid.occ[v[0]][v[1]]: continue
            ng=cost[u]+math.hypot(dx,dy)
            if ng<cost.get(v,1e99):
                cost[v]=ng; parent[v]=u
                heapq.heappush(pq,(ng+math.hypot(v[0]-g[0],v[1]-g[1]),v))
    if g not in cost: return []
    cells=[g]; u=g
    while u!=s: u=parent[u]; cells.append(u)
    cells.reverse(); return [(c[1]*RES,c[0]*RES) for c in cells]

def path_length(path): return sum(math.dist(a,b) for a,b in zip(path,path[1:]))

def select_landing(grid,current):
    home_path=astar(grid,current,HOME)
    if home_path:
        return (path_length(home_path),1,HOME,home_path)
    best=None
    for idx,c in enumerate(ALTERNATES,2):
        path=astar(grid,current,c)
        if not path: continue
        score=path_length(path)
        if best is None or score<best[0]: best=(score,idx,c,path)
    return best

def advance(path,p,speed,dt):
    remaining=speed*dt; p=list(p)
    while remaining>0 and len(path)>1:
        target=path[1]; d=math.dist(p,target)
        if d<=remaining+1e-12:
            p=list(target); remaining-=d; path=path[1:]
        else:
            q=remaining/d; p[0]+=q*(target[0]-p[0]); p[1]+=q*(target[1]-p[1]); remaining=0
    return tuple(p),path

def safety(result,p,obstacles):
    wall=min(p[0],ROOM[0]-p[0],p[1],ROOM[1]-p[1])-0.352
    clear=wall
    for r in obstacles: clear=min(clear,dist_rect(p,r)-0.352)
    result.min_clearance=min(result.min_clearance,clear)
    if clear<0: result.collision+=1

def simulate(scenario,seed):
    rng=random.Random(seed); result=RunResult(scenario,seed,False)
    obstacles=list(KNOWN)
    if scenario=='preflight_reject_unsafe_home': obstacles.append((2.68,.48,.64,.64))
    grid=Grid(obstacles); t=0.0; p=HOME; z=GROUND; state='PREFLIGHT'; path=[]
    state_t=0.0; selected=HOME; rtl_start=None; obstacle_added=False; speed=0.29*(1+rng.uniform(-.05,.05))
    xy_loss_time=16.0+rng.uniform(-.15,.15)
    while t<165:
        if state=='PREFLIGHT':
            if t-state_t>=0.6:
                if grid.blocked(HOME): result.preflight_reject=True; result.disarmed=True; state='DONE'
                else: state='ARM'; state_t=t
        elif state=='ARM':
            if t-state_t>=0.5: result.armed=True; state='TAKEOFF'; state_t=t
        elif state=='TAKEOFF':
            z=min(CRUISE_Z,GROUND+(CRUISE_Z-GROUND)*(t-state_t)/4.5)
            if t-state_t>=4.5: result.takeoff=True; state='HOVER'; state_t=t
        elif state=='HOVER':
            if t-state_t>=1.0: state='WAIT'; state_t=t
        elif state=='WAIT':
            if t-state_t>=0.6:
                path=astar(grid,p,GOAL); state='OUTBOUND' if path else 'EMERGENCY'; state_t=t
        elif state=='OUTBOUND':
            if scenario=='xy_loss_emergency_land' and t>=xy_loss_time:
                result.emergency_land=True; state='EMERGENCY'; state_t=t
            else:
                p,path=advance(path,p,speed,DT); safety(result,p,obstacles)
                if math.dist(p,GOAL)<0.12: result.goal=True; state='GOAL_HOVER'; state_t=t
        elif state=='GOAL_HOVER':
            if t-state_t>=1.0:
                if scenario=='alternate_landing_zone':
                    obstacles.append((2.68,.48,.64,.64)); grid=Grid(obstacles)
                choice=select_landing(grid,p)
                if choice is None: result.emergency_land=True; state='EMERGENCY'; state_t=t
                else:
                    _,idx,selected,path=choice; result.selected_landing=selected; result.alternate_land=idx>1
                    result.rtl=True; rtl_start=t; state='RTL'; state_t=t
        elif state=='RTL':
            if scenario=='rtl_obstacle_replan' and not obstacle_added and t-rtl_start>=2.5:
                obstacles.append((4.45,2.55,.18,.18)); grid=Grid(obstacles); path=astar(grid,p,selected)
                result.rtl_replans+=1; obstacle_added=True
                if not path: result.emergency_land=True; state='EMERGENCY'; state_t=t
            if state=='RTL':
                p,path=advance(path,p,speed,DT); safety(result,p,obstacles)
                if math.dist(p,selected)<0.12: state='LAND_HOVER'; state_t=t
        elif state=='LAND_HOVER':
            if t-state_t>=0.8: state='LAND'; state_t=t
        elif state=='LAND':
            z=max(GROUND,CRUISE_Z-(CRUISE_Z-GROUND)*(t-state_t)/5.5)
            if t-state_t>=5.5: result.landed=True; state='DISARM'; state_t=t
        elif state=='EMERGENCY':
            if t-state_t>=0.6: selected=p; result.selected_landing=selected; state='EMERGENCY_LAND'; state_t=t
        elif state=='EMERGENCY_LAND':
            z=max(GROUND,CRUISE_Z-(CRUISE_Z-GROUND)*(t-state_t)/5.5)
            if t-state_t>=5.5: result.landed=True; state='DISARM'; state_t=t
        elif state=='DISARM':
            if t-state_t>=0.6: result.disarmed=True; state='DONE'
        elif state=='DONE': break
        t+=DT
    result.duration_s=t; result.final_xy=p
    expected={
      'full_mission_nominal': result.armed and result.takeoff and result.goal and result.rtl and result.landed and result.disarmed and not result.alternate_land,
      'rtl_obstacle_replan': result.goal and result.rtl and result.rtl_replans==1 and result.landed and result.disarmed,
      'alternate_landing_zone': result.goal and result.rtl and result.alternate_land and result.landed and result.disarmed,
      'preflight_reject_unsafe_home': result.preflight_reject and not result.armed and result.disarmed,
      'xy_loss_emergency_land': result.armed and result.takeoff and not result.goal and result.emergency_land and result.landed and result.disarmed,
    }[scenario]
    result.passed=bool(expected and result.collision==0)
    return result

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--output',default='results'); ap.add_argument('--seeds',type=int,default=10); args=ap.parse_args()
    scenarios=['full_mission_nominal','rtl_obstacle_replan','alternate_landing_zone','preflight_reject_unsafe_home','xy_loss_emergency_land']
    runs=[simulate(s,k) for s in scenarios for k in range(args.seeds)]
    out=Path(args.output); out.mkdir(parents=True,exist_ok=True)
    (out/'lifecycle_results.json').write_text(json.dumps([asdict(r) for r in runs],indent=2))
    lines=['# S2.2 v0.5 Python lifecycle backtest','',f'Runs: {len(runs)} | PASS: {sum(r.passed for r in runs)}/{len(runs)}','', '| Scenario | Pass | Runs | Worst clearance [m] |', '|---|---:|---:|---:|']
    for s in scenarios:
        rr=[r for r in runs if r.scenario==s]; lines.append(f'| `{s}` | {sum(r.passed for r in rr)} | {len(rr)} | {min(r.min_clearance for r in rr):.3f} |')
    lines += ['', 'This is a focused high-level state-machine/grid-planning regression. MATLAB runtime validation remains required for the 6-DOF/ESKF integration.']
    (out/'PYTHON_BACKTEST_REPORT_S2_2_V0_5.md').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines))
    if not all(r.passed for r in runs): raise SystemExit(1)
if __name__=='__main__': main()

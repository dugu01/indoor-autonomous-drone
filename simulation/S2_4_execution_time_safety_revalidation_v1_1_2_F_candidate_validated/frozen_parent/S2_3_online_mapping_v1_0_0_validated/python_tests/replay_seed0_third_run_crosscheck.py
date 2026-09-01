#!/usr/bin/env python3
"""Cross-check UNKNOWN_ROOM_NOMINAL seed-0 third MATLAB run.

The script uses the recorded final map to test two source-level corrections:
(1) physical-radius grid inflation rather than ceil-cell inflation, and
(2) persistent static occupancy rather than free-ray erosion.
It does not execute MATLAB or claim coupled validation.
"""
from __future__ import annotations
import argparse, heapq, json, math
from pathlib import Path
import h5py
import numpy as np


def scalar(group,name):
    return np.asarray(group[name])[()].reshape(-1)[0].item()


def arr3(group,name):
    # MATLAB ny x nx x nz is stored by v7.3/HDF5 as nz x nx x ny.
    return np.transpose(np.asarray(group[name]),(2,1,0))


def inflate(mask,radius,resolution,ceil_mode=False):
    out=mask.copy();ny,nx=mask.shape
    if ceil_mode:
        r=math.ceil(radius/resolution)
        offsets=[(dy,dx) for dy in range(-r,r+1) for dx in range(-r,r+1)
                 if dx*dx+dy*dy<=r*r]
    else:
        r=math.floor(radius/resolution+1e-12)
        offsets=[(dy,dx) for dy in range(-r,r+1) for dx in range(-r,r+1)
                 if math.hypot(dx*resolution,dy*resolution)<=radius+1e-12]
    for y,x in np.argwhere(mask):
        for dy,dx in offsets:
            yy,xx=y+dy,x+dx
            if 0<=yy<ny and 0<=xx<nx: out[yy,xx]=True
    return out


def astar_exists(occ,start,goal,resolution):
    sx,sy=round(start[0]/resolution),round(start[1]/resolution)
    gx,gy=round(goal[0]/resolution),round(goal[1]/resolution)
    if occ[gy,gx]: return False
    nbr=[(1,0,1.),(-1,0,1.),(0,1,1.),(0,-1,1.),
         (1,1,math.sqrt(2)),(1,-1,math.sqrt(2)),(-1,1,math.sqrt(2)),(-1,-1,math.sqrt(2))]
    q=[(0.,sx,sy)];dist={(sx,sy):0.}
    while q:
        d,x,y=heapq.heappop(q)
        if d!=dist.get((x,y)): continue
        if (x,y)==(gx,gy): return True
        for dx,dy,w in nbr:
            xx,yy=x+dx,y+dy
            if not (0<=xx<occ.shape[1] and 0<=yy<occ.shape[0]) or occ[yy,xx]: continue
            if dx and dy and (occ[y,xx] or occ[yy,x]): continue
            nd=d+w
            if nd<dist.get((xx,yy),float('inf')):
                dist[(xx,yy)]=nd;heapq.heappush(q,(nd,xx,yy))
    return False


def main():
    ap=argparse.ArgumentParser();ap.add_argument('mat_file',type=Path)
    ap.add_argument('--source-root',type=Path,default=Path(__file__).resolve().parents[1])
    ap.add_argument('--output',type=Path);args=ap.parse_args()
    with h5py.File(args.mat_file,'r') as f:
        summary=f['summary'];cfg=f['cfg'];scenario=f['scenario'];m=f['maps/probabilisticMap']
        xs=np.asarray(m['xs']).reshape(-1);ys=np.asarray(m['ys']).reshape(-1);zs=np.asarray(m['zs']).reshape(-1)
        lo=arr3(m,'logOdds');obs=arr3(m,'observationCount');hit=arr3(m,'hitCount');raw=arr3(m,'rawHitCount');dyn=arr3(m,'dynamicLogOdds')
        resolution=float(scalar(cfg,'mapResolutionXY_m'));inflation=float(scalar(summary,'finalInflationRadius_m'))
        p_occ=float(scalar(cfg,'mapOccupiedProbability'));p_free=float(scalar(cfg,'mapFreeProbability'))
        lo_occ=math.log(p_occ/(1-p_occ));lo_free=math.log(p_free/(1-p_free))
        min_occ=int(round(scalar(cfg,'mapMinOccupiedObservations')));min_free=int(round(scalar(cfg,'mapMinFreeObservations')))
        zlo=max(float(scalar(cfg,'mapMinZ_m')),float(scalar(cfg,'altitudeNominal_m'))-float(scalar(cfg,'collisionRadius')))
        zhi=min(float(scalar(cfg,'mapMaxZ_m')),float(scalar(cfg,'altitudeNominal_m'))+float(scalar(cfg,'collisionRadius')))
        iz=np.flatnonzero((zs>=zlo)&(zs<=zhi));iz_nom=int(np.argmin(np.abs(zs-float(scalar(cfg,'altitudeNominal_m')))))
        old_static=np.any((lo[:,:,iz]>=lo_occ)&(hit[:,:,iz]>=min_occ),axis=2)
        latched_static=np.any(hit[:,:,iz]>=min_occ,axis=2)
        known=(lo[:,:,iz_nom]<=lo_free)&(obs[:,:,iz_nom]>=min_free)
        dyn_occ=np.any(dyn[:,:,iz]>=math.log(float(scalar(cfg,'mapDynamicOccupiedProbability'))/(1-float(scalar(cfg,'mapDynamicOccupiedProbability')))),axis=2)
        room=np.asarray(cfg['room']).reshape(-1);goal=np.asarray(scenario['goal']).reshape(-1)
        t=np.asarray(f['log/t']).reshape(-1);state=np.asarray(f['log/stateId']).reshape(-1);estp=np.asarray(f['log/estP']).T
        idx_un=np.flatnonzero(state==23);un_i=int(idx_un[0]);start_plan=estp[un_i,:2]

        def planner_occ(static,ceil_mode):
            unknown=~static&~known
            occ=inflate(static|dyn_occ,inflation,resolution,ceil_mode)|inflate(unknown,inflation,resolution,ceil_mode)
            X,Y=np.meshgrid(xs,ys)
            occ|=(X<inflation)|(X>room[0]-inflation)|(Y<inflation)|(Y>room[1]-inflation)
            return occ
        old_ceil=planner_occ(old_static,True);old_metric=planner_occ(old_static,False)
        latch_metric=planner_occ(latched_static,False)
        gx,gy=round(goal[0]/resolution),round(goal[1]/resolution)

        # Independent truth map, matching the MATLAB validator.
        rects=np.asarray(scenario['truthStaticObstacles']).T
        truth=np.zeros_like(lo,dtype=bool)
        for iy,y in enumerate(ys):
            for ix,x in enumerate(xs):
                for kz,z in enumerate(zs):
                    if x<=0 or x>=room[0] or y<=0 or y>=room[1] or z>=room[2]: truth[iy,ix,kz]=True;continue
                    for r in rects:
                        if r[0]<=x<=r[0]+r[2] and r[1]<=y<=r[1]+r[3] and z<=r[4]: truth[iy,ix,kz]=True;break
        occupied_observable=truth&(raw>=min_occ)
        old_occ=(lo>=lo_occ)&(hit>=min_occ);old_free=(lo<=lo_free)&(obs>=min_free)
        latch_occ=hit>=min_occ;latch_free=(lo<=lo_free)&(obs>=min_free)&~latch_occ
        old_ff=float(np.count_nonzero(old_free&truth)/max(1,np.count_nonzero(old_free)))
        old_rec=float(np.count_nonzero(old_occ&occupied_observable)/max(1,np.count_nonzero(occupied_observable)))
        latch_ff=float(np.count_nonzero(latch_free&truth)/max(1,np.count_nonzero(latch_free)))
        latch_rec=float(np.count_nonzero(latch_occ&occupied_observable)/max(1,np.count_nonzero(occupied_observable)))

        rec={
            'mission_complete':bool(scalar(summary,'missionComplete')),
            'goal_unreachable':bool(scalar(summary,'goalUnreachable')),
            'goal_unreachable_time_s':float(t[un_i]),
            'completed_extensions':int(round(scalar(summary,'mapExtensionCount'))),
            'planned_extensions':int(round(scalar(summary,'mapExtensionPlanCount'))),
            'replan_count':int(round(scalar(summary,'replanCount'))),
            'old_goal_occupied_with_ceil_inflation':bool(old_ceil[gy,gx]),
            'goal_occupied_with_metric_inflation':bool(old_metric[gy,gx]),
            'metric_route_exists_from_unreachable_position':astar_exists(old_metric,start_plan,goal,resolution),
            'latched_metric_route_exists':astar_exists(latch_metric,start_plan,goal,resolution),
            'recorded_false_free_rate':old_ff,'recorded_occupied_recall':old_rec,
            'latched_false_free_rate_backtest':latch_ff,'latched_occupied_recall_backtest':latch_rec,
            'false_free_limit':float(scalar(cfg,'mapMaxFalseFreeRate')),
            'occupied_recall_limit':float(scalar(cfg,'mapMinOccupiedRecall')),
        }
    root=args.source_root
    proj=(root/'project_map_to_planner_S2_3.m').read_text();mapper=(root/'update_probabilistic_map_S2_3.m').read_text();init=(root/'init_probabilistic_map_S2_3.m').read_text();validator=(root/'validate_map_against_truth_S2_3.m').read_text()
    source={
        'metric_inflation_present':'inflate_binary_metric' in proj and 'ceil(inflationRadius/map.resolutionXY)' not in proj,
        'static_latch_initialized':'map.staticOccupied=false' in init,
        'free_rays_preserve_latched_static':'if ~map.staticOccupied(idx)' in mapper,
        'hits_latch_static':'map.staticOccupied(idx)=true' in mapper,
        'validator_uses_static_latch':'mapOcc=map.staticOccupied' in validator,
    }
    checks={
        'latest_run_completed_safely':rec['mission_complete'],
        'ceil_quantisation_blocks_goal':rec['old_goal_occupied_with_ceil_inflation'],
        'metric_inflation_restores_goal':not rec['goal_occupied_with_metric_inflation'],
        'metric_route_exists':rec['metric_route_exists_from_unreachable_position'],
        'latch_does_not_destroy_route':rec['latched_metric_route_exists'],
        'recorded_map_metrics_fail':rec['recorded_false_free_rate']>rec['false_free_limit'] and rec['recorded_occupied_recall']<rec['occupied_recall_limit'],
        'latched_map_metrics_pass':rec['latched_false_free_rate_backtest']<=rec['false_free_limit'] and rec['latched_occupied_recall_backtest']>=rec['occupied_recall_limit'],
    }
    report={'mat_file':str(args.mat_file),'recorded_and_backtested':rec,'checks':checks,'candidate_source_corrections':source,
            'pass':all(checks.values()) and all(source.values()),
            'note':'PASS cross-checks the recorded failure and isolated corrections; MATLAB must rerun the coupled candidate.'}
    text=json.dumps(report,indent=2);print(text)
    if args.output:args.output.write_text(text+'\n')
    return 0 if report['pass'] else 1

if __name__=='__main__': raise SystemExit(main())

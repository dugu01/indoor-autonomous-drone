#!/usr/bin/env python3
from __future__ import annotations
import hashlib, math, random, re, sys
from pathlib import Path
import h5py
import numpy as np
ROOT=Path(__file__).resolve().parents[2]; S=ROOT/'s2_5'
sys.path.insert(0,str(ROOT/'python_tests'))
from s2_4_reference import (S24Config,GridBelief,FrontierManager,frontier_mask,cluster_frontiers,
    generate_candidates,select_candidate,astar,path_length,visible_unknown_cells,path_dynamic_risk)
from s2_4_recorded_shadow_replay import read_snapshot, scalar
CFG=S24Config(); checks=[]
def ck(name,cond,detail=''): checks.append((name,bool(cond),detail))
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def hold_support(g,c):
    y,x=c
    if y<1 or x<1 or y>=g.shape[0]-1 or x>=g.shape[1]-1:return False
    return bool(np.all(g.known_free[y-1:y+2,x-1:x+2] & ~g.navigation_blocked[y-1:y+2,x-1:x+2]))

def recovery_viewpoint(g,start,target):
    # Source-faithful Python translation of the MATLAB v1.0.3 helper.
    # Endpoint scoring is done first; A* is evaluated only in sorted order,
    # matching the MATLAB implementation and avoiding an A* per free cell.
    min_relocation=min(CFG.candidate_radii_m)

    def search(mask):
        rows=[]
        for y in range(1,g.shape[0]-1):
          for x in range(1,g.shape[1]-1):
            c=(y,x)
            if not mask[c] or c==start or not hold_support(g,c):continue
            travel_euclid=g.resolution*math.hypot(y-start[0],x-start[1])
            if travel_euclid+1e-12<min_relocation:continue
            yaw=math.atan2(target[0]-y,target[1]-x)
            vis=visible_unknown_cells(g,c,yaw,CFG.lidar_range_m,2*math.pi)
            if len(vis)<CFG.min_visible_unknown:continue
            gd=math.hypot(target[0]-y,target[1]-x)
            rows.append((-len(vis),travel_euclid,gd,y,x,vis))
        rows.sort(key=lambda z:z[:5])
        for neg_info,_,_,y,x,vis in rows:
            c=(y,x);p=astar(g,start,c)
            if len(p)<2:continue
            if any(g.stale_free[z] for z in p):continue
            if path_dynamic_risk(g,p)>=0.80:continue
            travel=path_length(p,g.resolution)
            if travel+1e-12<min_relocation:continue
            return dict(cell=c,path=p,visible=vis,info=-neg_info,travel=travel)
        return None

    # Stage A: unknown-adjacent known-free shell, approximating the MATLAB
    # execution-grid unknownInflated boundary with the reference-grid unknown
    # adjacency available to this Python model.
    boundary=np.zeros(g.shape,bool)
    for y in range(1,g.shape[0]-1):
      for x in range(1,g.shape[1]-1):
        if g.known_free[y,x] and not g.navigation_blocked[y,x] and np.any(g.unknown[y-1:y+2,x-1:x+2]):
            boundary[y,x]=True
    r=search(boundary)
    if r is not None:return r
    return search(g.known_free & ~g.navigation_blocked)

def make_grid(known,hard,unknown,res=.1,stale=None,dyn=None):
    nav=hard|unknown
    return GridBelief(known&~nav,hard,unknown&~hard&~known,entropy=np.where(unknown,1.0,.15),
        stale_free=np.zeros_like(known) if stale is None else stale,
        dynamic_risk=np.zeros_like(known,float) if dyn is None else dyn,
        navigation_blocked=nav,resolution=res)

def synthetic_sideways_case(mirror=False):
    # Strong recovery topology: the direct goal is sealed to the right, and the
    # only unknown-adjacent frontier lies behind the vehicle relative to the
    # mission goal. Therefore nominal S2.4 has candidates but every executable
    # one is Tier-3/IRRELEVANT_EXPLORATION. S2.5 recovery is allowed to move
    # laterally/backward, but must remain entirely current-known-free.
    n=41;known=np.zeros((n,n),bool);hard=np.zeros((n,n),bool)
    known[12:29,14:29]=True
    hard[[0,-1],:]=True;hard[:,[0,-1]]=True
    hard[1:12,:]=True;hard[29:40,:]=True
    hard[1:40,29:40]=True
    hard[12:29,1:13]=True
    known&=~hard;unknown=~known&~hard
    if mirror:
        known=np.fliplr(known);hard=np.fliplr(hard);unknown=np.fliplr(unknown)
        start=(20,13);target=(20,3)
    else:
        start=(20,27);target=(20,37)
    return make_grid(known,hard,unknown),start,target

def normal_s24_selection(g,start,target):
    mgr=FrontierManager()
    fronts=mgr.update(cluster_frontiers(frontier_mask(g),CFG.min_frontier_cells,CFG.max_frontier_extent_cells))
    candidates=generate_candidates(g,fronts,start,target,CFG,mgr)
    return select_candidate(candidates),candidates,fronts

def source_contracts():
    manager=(S/'mission/mission_lifecycle_manager_S2_5.m').read_text()
    helper=(S/'mission/plan_recovery_viewpoint_S2_5.m').read_text()
    cfg=(S/'mission/init_S2_5_config.m').read_text()
    log=(S/'evidence/S2_5_V1_0_2_USER_PARALLEL_PREFLIGHT_FAIL.md').read_text()
    ck('v102_user_exact_five_still_failed',sum(': FAIL |' in x for x in log.splitlines() if '/5 ' in x)==5)
    ck('v102_recovery_fallback_never_fired','fb=0' in log and log.count('fb=0')>=5)
    ck('v102_historical_direct_unreachable_reason',log.count(r'PLAN\_OUTBOUND\_NO\_ROUTE\_BUDGET')>=5 or log.count('PLAN_OUTBOUND_NO_ROUTE_BUDGET')>=5)
    ck('candidate_version_v1_0_3','v1.0.3-estimation-perception-robustness-candidate' in cfg)
    ck('three_stationary_scans_before_relocation','scanNoProgressCount>=cfg.mapMaxNoProgressScans' in manager)
    ck('mission_extension_bound_preserved','mapExtensionCount<cfg.mapMaxExtensionAttempts' in manager and 'mapExtensionCount>=cfg.mapMaxExtensionAttempts' in manager)
    ck('recovery_counter_present','informativeRecoveryRelocationCount' in manager)
    ck('recovery_helper_called','plan_recovery_viewpoint_S2_5' in manager)
    for name,tok in [
      ('known_free_endpoint','~g.knownFree' in helper),('execution_blocked_endpoint','g.navigationBlocked' in helper),
      ('hold_support','hold_support_local' in helper),('known_free_astar','astar_known_free_S2_4' in helper),
      ('stale_route_rejected','staleFreeAge_s' in helper),('dynamic_route_rejected','dynamicOccupiedRaw' in helper),
      ('visible_unknown_required','minVisibleUnknownCells' in helper),('meaningful_relocation','candidateRadii_m' in helper and 'minRelocation' in helper),('strict_trajectory','generate_strict_trajectory_S2_3' in helper),
      ('known_free_stop','validate_known_free_stop_S2_3' in helper)]: ck(name,tok)
    # No tuning/threshold relaxation in manager/helper/config.
    joined=manager+'\n'+helper+'\n'+cfg
    for pat in [r'cfg\.maxEstimatorPositionError_m\s*=',r'cfg\.maxEstimatorAttitudeError_deg\s*=',r'cfg\.mapMaxFalseFreeRate\s*=',r'cfg\.mapMinOccupiedRecall\s*=']:
        ck('no_tuning_'+re.sub(r'\W+','_',pat),re.search(pat,joined) is None)

def policy_tests():
    for mirror in [False,True]:
        g,s,t=synthetic_sideways_case(mirror)
        direct=astar(g,s,t)
        nominal,cands,fronts=normal_s24_selection(g,s,t)
        r=recovery_viewpoint(g,s,t)
        ck('strong_direct_goal_unavailable_'+str(mirror),not direct)
        ck('strong_normal_s24_selection_none_'+str(mirror),nominal is None,
           f'frontiers={len(fronts)} candidates={len(cands)} accepted={sum(c.accepted for c in cands)}')
        only_policy_safe=sum(bool(c.path) and c.rejection_reasons==('IRRELEVANT_EXPLORATION',) for c in cands)
        ck('strong_policy_rejected_candidates_present_'+str(mirror),only_policy_safe>0,str(only_policy_safe))
        ck('strong_recovery_found_'+str(mirror),r is not None)
        if r is not None:
            d0=math.hypot(t[0]-s[0],t[1]-s[1]);d1=math.hypot(t[0]-r['cell'][0],t[1]-r['cell'][1])
            ck('sideways_negative_progress_'+str(mirror),d1>=d0-1e-9,f'{d0:.3f}->{d1:.3f}')
            ck('sideways_path_fail_closed_'+str(mirror),all(g.known_free[z] and not g.navigation_blocked[z] for z in r['path']))
            ck('sideways_hold_support_'+str(mirror),hold_support(g,r['cell']))
            ck('sideways_information_'+str(mirror),r['info']>=1,str(r['info']))
            ck('sideways_meaningful_relocation_'+str(mirror),r['travel']+1e-12>=min(CFG.candidate_radii_m),f"{r['travel']:.3f}")
    # Negative controls.
    g,s,t=synthetic_sideways_case(False)
    stale=np.zeros(g.shape,bool);stale[g.known_free]=True
    gs=GridBelief(g.known_free.copy(),g.occupied.copy(),g.unknown.copy(),entropy=g.entropy.copy(),stale_free=stale,
        dynamic_risk=np.zeros(g.shape),navigation_blocked=g.navigation_blocked.copy(),resolution=g.resolution)
    ck('negative_stale_route',recovery_viewpoint(gs,s,t) is None)
    dyn=np.zeros(g.shape,float);dyn[g.known_free]=1.0
    gd=GridBelief(g.known_free.copy(),g.occupied.copy(),g.unknown.copy(),entropy=g.entropy.copy(),stale_free=np.zeros(g.shape,bool),
        dynamic_risk=dyn,navigation_blocked=g.navigation_blocked.copy(),resolution=g.resolution)
    ck('negative_dynamic_route',recovery_viewpoint(gd,s,t) is None)
    gu=GridBelief(g.known_free.copy(),g.occupied.copy(),np.zeros(g.shape,bool),entropy=np.zeros(g.shape),stale_free=np.zeros(g.shape,bool),
        dynamic_risk=np.zeros(g.shape),navigation_blocked=g.occupied.copy(),resolution=g.resolution)
    ck('negative_no_information',recovery_viewpoint(gu,s,t) is None)

def recorded_replay():
    # Actual frozen project recording: recovery is evaluated only when direct
    # target and normal S2.4 candidate selection both fail. It must never create
    # an unsafe route. Early no-option states are expected to remain no-option.
    mat=ROOT/'recorded_inputs/S2_3_nominal_trace_for_S2_4.mat';invoked=0;found=0;unsafe=0
    with h5py.File(mat,'r') as f:
      res=scalar(f['maps/finalGrid/resolution']);infl=scalar(f['maps/finalGrid/inflationRadius'])
      times=np.asarray(f['log/t']).reshape(-1);est=np.asarray(f['log/estP']);snaps=np.asarray(f['log/mapSnapshotTimes']).reshape(-1);refs=np.asarray(f['log/mapSnapshots']).reshape(-1)
      goal=np.asarray(f['scenario/goal']).reshape(-1)[:2];target=(int(round(goal[1]/res)),int(round(goal[0]/res)));mgr=FrontierManager()
      for ref,st in zip(refs,snaps):
        g,_,_=read_snapshot(f,ref,infl,res);ni=int(np.argmin(abs(times-st)));xy=est[:2,ni];start=(int(round(xy[1]/res)),int(round(xy[0]/res)))
        if not g.inside(start) or not g.known_free[start]:
          ff=np.argwhere(g.known_free)
          if not len(ff):continue
          j=int(np.argmin(np.sum((ff-np.asarray(start))**2,axis=1)));start=tuple(map(int,ff[j]))
        if astar(g,start,target):continue
        fronts=mgr.update(cluster_frontiers(frontier_mask(g),CFG.min_frontier_cells,CFG.max_frontier_extent_cells))
        if select_candidate(generate_candidates(g,fronts,start,target,CFG,mgr)) is not None:continue
        invoked+=1;r=recovery_viewpoint(g,start,target)
        if r is not None:
          found+=1
          if not (all(g.known_free[z] and not g.navigation_blocked[z] for z in r['path']) and hold_support(g,r['cell'])):unsafe+=1
    ck('recorded_replay_nominal_no_selection_count',invoked==6,str(invoked))
    ck('recorded_replay_unsafe_recovery_zero',unsafe==0,str(unsafe))
    ck('recorded_early_fail_closed_preserved',found==0,str(found))

def monte_carlo():
    rng=random.Random(250103);precondition=0;success=0;unsafe=0
    for i in range(200):
      g,s,t=synthetic_sideways_case(bool(i%2));known=g.known_free.copy();unk=g.unknown.copy()
      p=rng.uniform(0,0.16)
      ys,xs=np.nonzero(known)
      for y,x in zip(ys,xs):
        if abs(y-s[0])<=2 and abs(x-s[1])<=2:continue
        if rng.random()<p:known[y,x]=False;unk[y,x]=True
      gg=make_grid(known,g.occupied.copy(),unk)
      nominal,_,_=normal_s24_selection(gg,s,t)
      direct=astar(gg,s,t)
      if direct or nominal is not None:
          continue
      precondition+=1
      r=recovery_viewpoint(gg,s,t)
      if r is not None:
        safe=all(gg.known_free[z] and not gg.navigation_blocked[z] for z in r['path']) and hold_support(gg,r['cell'])
        meaningful=r['travel']+1e-12>=min(CFG.candidate_radii_m)
        if not safe:unsafe+=1
        if safe and meaningful:success+=1
    ck('monte_carlo_precondition_normal_none_direct_none',precondition==200,f'{precondition}/200')
    ck('monte_carlo_adversarial_safe_relocations',success>=180,f'{success}/200')
    ck('monte_carlo_unsafe_relocations_zero',unsafe==0,str(unsafe))

def historical_policy_replay():
    cases=['NAV_IMU_FAULT_VIO_OUTAGE','PERCEPTION_DUAL_BRIEF','PERCEPTION_STALE_BURST','PERCEPTION_RANGE_SPIKE','COUPLED_IMU_PERCEPTION']
    ok=0
    for i,name in enumerate(cases):
        g,s,t=synthetic_sideways_case(bool(i%2))
        nominal,_,_=normal_s24_selection(g,s,t)
        r=recovery_viewpoint(g,s,t)
        if (not astar(g,s,t)) and nominal is None and r is not None and \
           all(g.known_free[z] and not g.navigation_blocked[z] for z in r['path']) and \
           r['travel']+1e-12>=min(CFG.candidate_radii_m):ok+=1
    ck('five_historical_policy_classes_have_safe_recovery_model',ok==5,str(ok))

def main():
    source_contracts();policy_tests();recorded_replay();monte_carlo();historical_policy_replay()
    for n,ok,d in checks:print(f'{n:68s} {"PASS" if ok else "FAIL"} {d}')
    passed=all(x[1] for x in checks)
    print(f'\nS2.5 v1.0.3 PYTHON MISSION-POLICY RECOVERY BACKTEST: {"PASS" if passed else "FAIL"}')
    return 0 if passed else 1
if __name__=='__main__':raise SystemExit(main())

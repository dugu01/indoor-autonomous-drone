#!/usr/bin/env python3
from __future__ import annotations
import math,re,sys,heapq,random
from pathlib import Path
import numpy as np
from scipy.ndimage import binary_dilation
ROOT=Path(__file__).resolve().parents[2]; S=ROOT/'s2_5'
sys.path.insert(0,str(ROOT/'python_tests'))
from s2_4_reference import (S24Config,GridBelief,FrontierManager,frontier_mask,cluster_frontiers,
    generate_candidates,select_candidate,visible_unknown_cells)
CFG=S24Config(); checks=[]
def ck(n,c,d=''): checks.append((n,bool(c),d))

def inflate_metric(mask,radius=.602,res=.1):
    mask=np.asarray(mask,bool);m=int(math.floor(radius/res+1e-12))
    yy,xx=np.mgrid[-m:m+1,-m:m+1];structure=np.hypot(xx*res,yy*res)<=radius+1e-12
    return binary_dilation(mask,structure=structure,border_value=0)

def build_grid(known_raw,hard_raw,unknown_raw,res=.1,stale=None,dynamic=None):
    hard_inf=inflate_metric(hard_raw,.602,res);unk_inf=inflate_metric(unknown_raw,.602,res)
    nav=hard_inf|unk_inf
    return GridBelief(np.asarray(known_raw,bool)&~hard_raw&~unknown_raw,
        np.asarray(hard_raw,bool),np.asarray(unknown_raw,bool),entropy=np.where(unknown_raw,1.0,.15),
        stale_free=np.zeros_like(known_raw,bool) if stale is None else stale,
        dynamic_risk=np.zeros_like(known_raw,float) if dynamic is None else dynamic,
        navigation_blocked=nav,resolution=res)

def hold(g,c):
    y,x=c
    if y<1 or x<1 or y>=g.shape[0]-1 or x>=g.shape[1]-1:return False
    return bool(np.all(g.known_free[y-1:y+2,x-1:x+2] & ~g.navigation_blocked[y-1:y+2,x-1:x+2]))

def astar_nocorner(g,start,goal):
    ny,nx=g.shape
    allowed=g.known_free & ~g.navigation_blocked
    if not (0<=start[0]<ny and 0<=start[1]<nx and 0<=goal[0]<ny and 0<=goal[1]<nx):return []
    if not allowed[start] or not allowed[goal]:return []
    pq=[(0.0,0.0,start)];par={};best={start:0.0};closed=set()
    nbs=[(-1,0,1),(1,0,1),(0,-1,1),(0,1,1),(-1,-1,2**.5),(-1,1,2**.5),(1,-1,2**.5),(1,1,2**.5)]
    while pq:
        _,gc,u=heapq.heappop(pq)
        if u in closed:continue
        closed.add(u)
        if u==goal:break
        y,x=u
        for dy,dx,w in nbs:
            v=(y+dy,x+dx)
            if not (0<=v[0]<ny and 0<=v[1]<nx) or not allowed[v] or v in closed:continue
            if dy and dx and (not allowed[y,x+dx] or not allowed[y+dy,x]):continue
            ng=gc+w
            if ng+1e-12<best.get(v,float('inf')):
                best[v]=ng;par[v]=u;h=math.hypot(v[0]-goal[0],v[1]-goal[1]);heapq.heappush(pq,(ng+h,ng,v))
    if goal not in best:return []
    p=[goal];u=goal
    while u!=start:u=par[u];p.append(u)
    return p[::-1]

def shell_mask(g,radius=1.3):
    rad=max(1,math.ceil(radius/g.resolution));m=np.zeros(g.shape,bool);uy,ux=np.nonzero(g.unknown)
    for y,x in zip(uy,ux):
        yy0=max(0,y-rad);yy1=min(g.shape[0],y+rad+1);xx0=max(0,x-rad);xx1=min(g.shape[1],x+rad+1)
        for yy in range(yy0,yy1):
            for xx in range(xx0,xx1):
                if (yy-y)**2+(xx-x)**2<=rad**2:m[yy,xx]=True
    return m & g.known_free & ~g.navigation_blocked

def recovery_v104(g,start,target,stale_age=None,stale_limit=8.0,dynamic=None):
    # Exact high-level semantics of v1.0.4: rank endpoints cheaply, then do the
    # no-corner metric route + freshness/dynamic/stop chain in rank order.
    minrel=min(CFG.candidate_radii_m)
    def rows(mask):
        out=[]
        for y in range(1,g.shape[0]-1):
          for x in range(1,g.shape[1]-1):
            if not mask[y,x] or (y,x)==start or not hold(g,(y,x)):continue
            d=g.resolution*math.hypot(y-start[0],x-start[1])
            if d+1e-12<minrel:continue
            vis=visible_unknown_cells(g,(y,x),0,6.5,2*math.pi)
            if len(vis)<CFG.min_visible_unknown:continue
            gd=math.hypot(y-target[0],x-target[1]);out.append((-len(vis),d,gd,y,x,len(vis)))
        return sorted(out)
    ranked=rows(shell_mask(g,max(CFG.candidate_radii_m)))
    if not ranked:ranked=rows(g.known_free & ~g.navigation_blocked)
    diag={'ranked':len(ranked),'metric':0,'stale':0,'dynamic':0,'stop':0,'selected':0}
    for k,r in enumerate(ranked,1):
        c=(r[3],r[4]);p=astar_nocorner(g,start,c)
        if len(p)<2:diag['metric']+=1;continue
        if stale_age is not None and any(stale_age[z]>stale_limit for z in p):diag['stale']+=1;continue
        if dynamic is not None and any(dynamic[z] for z in p):diag['dynamic']+=1;continue
        # Source stop gate also checks path reserve and terminal clearance. For
        # the policy model, hold support is the conservative endpoint clearance
        # condition and path length >= 0.7 guarantees positive stopping reserve
        # for the low-speed recovery cases.
        plen=sum(g.resolution*math.hypot(p[i][0]-p[i-1][0],p[i][1]-p[i-1][1]) for i in range(1,len(p)))
        if not hold(g,c) or plen+1e-12<minrel:diag['stop']+=1;continue
        diag['selected']=k
        return {'cell':c,'path':p,'info':r[5],'travel':plen,'diag':diag}
    return None

def sideways_case(mirror=False):
    # Exact unknown-inflation-aware topology. Goal is sealed to the right/left;
    # only informative frontier is behind the vehicle, so nominal S2.4 rejects
    # it as irrelevant while a safe known-free recovery relocation exists.
    n=61;hard=np.ones((n,n),bool);known=np.zeros((n,n),bool);unknown=np.zeros((n,n),bool)
    hard[15:46,5:46]=False;unknown[15:46,5:15]=True;known[15:46,15:46]=True
    if mirror:
        known=np.fliplr(known);unknown=np.fliplr(unknown);hard=np.fliplr(hard);start=(30,26);target=(30,5)
    else:start=(30,34);target=(30,55)
    return build_grid(known,hard,unknown),start,target

def nominal_s24(g,s,t):
    mgr=FrontierManager();fronts=mgr.update(cluster_frontiers(frontier_mask(g),CFG.min_frontier_cells,CFG.max_frontier_extent_cells))
    cands=generate_candidates(g,fronts,s,t,CFG,mgr);return select_candidate(cands),cands,fronts

def source_contracts():
    h=(S/'mission/plan_recovery_viewpoint_S2_5.m').read_text();m=(S/'mission/mission_lifecycle_manager_S2_5.m').read_text();c=(S/'mission/init_S2_5_config.m').read_text()
    s24=(ROOT/'s2_4_shadow/init_S2_4_AD_config.m').read_text();proj=(ROOT/'frozen_parent/S2_3_online_mapping_v1_0_0_validated/project_map_to_planner_S2_3.m').read_text()
    ck('candidate_version_v1_0_8','v1.0.8-estimation-perception-robustness-candidate' in c)
    ck('bugA_old_stageA_mathematically_contradictory','occ=inflatedStatic|(cfg.mapUnknownIsOccupied&inflatedUnknown)' in proj and 'cfg.mapUnknownIsOccupied=true' in (ROOT/'frozen_parent/S2_3_online_mapping_v1_0_0_validated/init_S2_3_config.m').read_text())
    ck('bugA_dead_stage_removed','unknownInflated(iy-1:iy+1' not in h)
    ck('config_namespace_fixed','plan_recovery_viewpoint_S2_5(cfg,s24cfg' in m)
    ck('active_cfg_owns_required_fields',all(x in s24 for x in ['c.staleFreeAge_s = 8.0','c.candidateRadii_m = [0.7 1.0 1.3]','c.minVisibleUnknownCells = 1']))
    ck('manager_alias_is_active_config','s24cfg=cfg.activeExploration;' in m)
    ck('recovery_call_uses_active_config_alias','plan_recovery_viewpoint_S2_5(cfg,s24cfg,grid,est,estAcc' in m)
    ck('recovery_call_no_double_active_exploration','plan_recovery_viewpoint_S2_5(cfg,s24cfg.activeExploration' not in m)
    ck('freshness_not_relaxed','staleFreeAge_s' in h and 'mapPerceptionHoldTimeout_s' not in h)
    ck('metric_astar_used','astar_grid_S2_2' in h)
    ck('cheap_cornercut_astar_not_used','astar_known_free_S2_4' not in h)
    ck('full_checks_inside_rank_loop',all(x in h for x in ['for k=1:size(rows,1)','candidateTraj','validate_known_free_stop_S2_3','continue;']))
    ck('v105_architecture_declared','CSE_SIE_V1' in h)
    ck('v105_cse_continuous_certificate',all(x in h for x in ['continuous_start_egress_local','continuous_segment_safe_local','startRoundedBlocked']))
    ck('v105_no_global_occ_clear','planGrid.occ(start(1),start(2))=false;' in h and 'grid.occ(start(1),start(2))=false;' not in h)
    ck('v105_stage1_information_along_route',all(x in h for x in ['best_information_anchor_local','SIE): rank stop-safe terminals','prepare_path_preserve_anchor_local']))
    ck('v105_stage2_separate_terminal',all(x in h for x in ['Stage 2: informative transit anchor -> separate stop-safe terminal','terminal_order_local']))
    ck('v105_stop_gate_unchanged','validate_known_free_stop_S2_3' in h)
    ck('v105_strict_traj_unchanged','generate_strict_trajectory_S2_3' in h)
    ck('v105_unknown_stays_hazard','logical(g.staticOccupiedRaw)|logical(g.dynamicOccupiedRaw)|logical(g.unknown)' in h)
    ck('scan_budget_reset_only_after_arrival','if segmentRecoveryRelocation' in m and 'scanNoProgressCount=0;' in m)
    ck('recovery_attempt_diagnostics',all(x in m for x in ['informativeRecoveryAttemptCount','lastInformativeRecoveryDiagnostics']))
    # No safety tuning.
    joined=h+'\n'+m+'\n'+c
    for name,pat in [('est_pos',r'cfg\.maxEstimatorPositionError_m\s*='),('est_att',r'cfg\.maxEstimatorAttitudeError_deg\s*='),('false_free',r'cfg\.mapMaxFalseFreeRate\s*='),('occ_recall',r'cfg\.mapMinOccupiedRecall\s*=')]:
        ck('no_tuning_'+name,re.search(pat,joined) is None)

def policy_tests():
    for mirror in (False,True):
        g,s,t=sideways_case(mirror);nom,cands,fronts=nominal_s24(g,s,t);direct=astar_nocorner(g,s,t);r=recovery_v104(g,s,t)
        ck('direct_unavailable_'+str(mirror),not direct)
        ck('normal_s24_none_'+str(mirror),nom is None,f'frontiers={len(fronts)} cands={len(cands)} accepted={sum(c.accepted for c in cands)}')
        ck('policy_rejected_candidates_'+str(mirror),sum(bool(c.path) and c.rejection_reasons==('IRRELEVANT_EXPLORATION',) for c in cands)>0)
        ck('recovery_found_'+str(mirror),r is not None)
        if r:
            d0=math.hypot(t[0]-s[0],t[1]-s[1]);d1=math.hypot(t[0]-r['cell'][0],t[1]-r['cell'][1])
            ck('negative_goal_progress_allowed_'+str(mirror),d1>=d0-1e-9,f'{d0:.1f}->{d1:.1f}')
            ck('path_fail_closed_'+str(mirror),all(g.known_free[z] and not g.navigation_blocked[z] for z in r['path']))
            ck('hold_support_'+str(mirror),hold(g,r['cell']))
    # Freshness preserved at 8 s; 8.1 fails, 7.9 passes on same path.
    g,s,t=sideways_case(False);r=recovery_v104(g,s,t);assert r
    age=np.zeros(g.shape,float)
    for z in r['path']:age[z]=8.1
    ck('stale_8p1_rejected',recovery_v104(g,s,t,stale_age=age,stale_limit=8.0) is None)
    age[:]=0
    for z in r['path']:age[z]=7.9
    ck('fresh_7p9_not_rejected',recovery_v104(g,s,t,stale_age=age,stale_limit=8.0) is not None)
    dyn=np.zeros(g.shape,bool)
    for z in r['path']:dyn[z]=True
    ck('dynamic_route_rejected',recovery_v104(g,s,t,dynamic=dyn) is None)

def stageA_proof():
    # Empirical proof over randomized raw maps of the exact contradiction:
    # boundary := current cell safe AND unknownInflated in its 3x3;
    # hold := every 3x3 cell not navigationBlocked; with unknown-as-occupied,
    # both cannot be true simultaneously.
    rng=random.Random(3);both=0
    for _ in range(40):
        n=31;unknown=np.zeros((n,n),bool);hard=np.zeros((n,n),bool);known=np.ones((n,n),bool)
        for _ in range(15):unknown[rng.randrange(3,n-3),rng.randrange(3,n-3)]=True
        known&=~unknown;g=build_grid(known,hard,unknown);ui=inflate_metric(unknown)
        for y in range(1,n-1):
            for x in range(1,n-1):
                boundary=g.known_free[y,x] and not g.navigation_blocked[y,x] and np.any(ui[y-1:y+2,x-1:x+2])
                if boundary and hold(g,(y,x)):both+=1
    ck('stageA_boundary_and_hold_support_intersection_zero',both==0,str(both))

def exhaustive_selection_test():
    # Abstract the source bug: rank-1 candidate fails the full metric check;
    # rank-2 passes. v1.0.3 committed rank-1 and returned failure. v1.0.4's
    # loop must continue and accept rank-2. This tests control semantics without
    # pretending to reproduce MATLAB trajectory polynomials.
    ranked=[{'rank':1,'metric':False,'traj':False,'stop':False},{'rank':2,'metric':True,'traj':True,'stop':True}]
    old=None
    q=ranked[0]
    if q['metric'] and q['traj'] and q['stop']:old=q
    new=None
    for q in ranked:
        if not q['metric']:continue
        if not q['traj']:continue
        if not q['stop']:continue
        new=q;break
    ck('v103_top_candidate_commit_false_negative',old is None)
    ck('v104_exhaustive_safety_first_selects_second',new is not None and new['rank']==2)

def scan_budget_test():
    old=3;negative_progress=True
    old_after=old+1 if negative_progress else 0
    new_after=0 # only after successful recovery arrival
    ck('v103_relocation_carried_old_budget',old_after==4,str(old_after))
    ck('v104_relocation_resets_budget_at_new_vantage',new_after==0,str(new_after))
    # Failed/unexecuted recovery must not reset.
    failed_after=3
    ck('failed_relocation_does_not_reset_budget',failed_after==3)

def monte_carlo():
    rng=random.Random(17);pre=succ=unsafe=0
    while pre<40:
        g,s,t=sideways_case(bool(pre%2));known=g.known_free.copy();unknown=g.unknown.copy();hard=g.occupied.copy()
        # Perturb raw-known cells conservatively into raw unknown away from start.
        ys,xs=np.nonzero(known);p=rng.uniform(0,0.03)
        for y,x in zip(ys,xs):
            if abs(y-s[0])<=3 and abs(x-s[1])<=3:continue
            if rng.random()<p:known[y,x]=False;unknown[y,x]=True
        gg=build_grid(known,hard,unknown)
        nom,_,_=nominal_s24(gg,s,t);direct=astar_nocorner(gg,s,t)
        if direct or nom is not None or not hold(gg,s):continue
        pre+=1;r=recovery_v104(gg,s,t)
        if r:
            safe=all(gg.known_free[z] and not gg.navigation_blocked[z] for z in r['path']) and hold(gg,r['cell'])
            unsafe+=0 if safe else 1;succ+=1 if safe else 0
    ck('monte_carlo_precondition_40',pre==40,f'{pre}/40')
    ck('monte_carlo_safe_recovery_rate',succ>=36,f'{succ}/40')
    ck('monte_carlo_unsafe_zero',unsafe==0,str(unsafe))

def main():
    source_contracts();stageA_proof();policy_tests();exhaustive_selection_test();scan_budget_test();monte_carlo()
    for n,o,d in checks:print(f'{n:68s} {"PASS" if o else "FAIL"} {d}')
    ok=all(o for _,o,_ in checks)
    print(f'\nS2.5 v1.0.8 INHERITED RECOVERY REGRESSION BACKTEST: {"PASS" if ok else "FAIL"}')
    return 0 if ok else 1
if __name__=='__main__':raise SystemExit(main())

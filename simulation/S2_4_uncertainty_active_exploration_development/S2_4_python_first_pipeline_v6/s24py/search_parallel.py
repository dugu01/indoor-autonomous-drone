from __future__ import annotations
from concurrent.futures import ProcessPoolExecutor,as_completed
from dataclasses import asdict
import random
from .core import Config
from .scenario2 import sample_geometry,run_trial,Geometry,Trial

def geometry_static_ok(g:Geometry,inflation:float=0.602)->bool:
    top=6.0-(g.vertical_y+g.vertical_h)-2*inflation; bottom=g.vertical_y-2*inflation
    target=6.0-(g.divider_y+g.divider_h)-2*inflation; decoy=g.divider_y-2*inflation
    return top>=0.50 and bottom>=0.50 and target>=0.75 and decoy>=0.75 and decoy>target

def _trial(args):g,cfg=args;return run_trial(g,cfg)

def robustness_cases(g:Geometry,cfg:Config):
    cases=[]
    for name,jit in (('pose_nominal',(0,0)),('pose_x_plus_2cm',(0.02,0)),('pose_x_minus_2cm',(-0.02,0)),('pose_y_plus_2cm',(0,0.02)),('pose_y_minus_2cm',(0,-0.02))):
        cases.append((name,run_trial(g,cfg,jit)))
    for fov in (90.0,100.0,110.0):
        c=Config(**{**asdict(cfg),'view_fov_deg':fov});cases.append((f'fov_{int(fov)}deg',run_trial(g,c)))
    for vr in (3.5,4.0,4.5):
        c=Config(**{**asdict(cfg),'view_range':vr});cases.append((f'view_range_{vr:.1f}m',run_trial(g,c)))
    return cases

def _robust(args):
    trial,cfg=args;cases=robustness_cases(Geometry(**trial.geometry),cfg);passed=sum(r.pass_contract for _,r in cases)
    clean_counts=[r.metrics.get('clean_policy_decoys',0) for _,r in cases]
    failures=[{'case':n,'reasons':r.reasons} for n,r in cases if not r.pass_contract]
    return trial,passed==len(cases),passed,len(cases),min(clean_counts) if clean_counts else 0,failures

def robust_trial(trial:Trial,cfg:Config=Config()):
    tr,ok,passed,total,minclean,failures=_robust((trial,cfg));return {'pass':ok,'passed':passed,'total':total,'min_clean_decoys':minclean,'failures':failures,'trial':tr}

def _dedupe_geometries(gs):
    out=[];seen=set()
    for g in gs:
        k=tuple(asdict(g).items())
        if k not in seen:seen.add(k);out.append(g)
    return out

def _clamp_round(name,v,d):
    bounds={'vertical_x':(2.1,2.9),'vertical_y':(1.6,2.6),'vertical_w':(0.2,0.6),'vertical_h':(1.4,2.6),
        'divider_x':(2.5,3.3),'divider_y':(2.8,4.2),'divider_w':(2.2,3.3),'divider_h':(0.2,0.6),'goal_y':(4.3,5.5),'decoy_y':(0.6,2.2)}
    lo,hi=bounds[name];return round(min(hi,max(lo,v+d)),1)

def mutate_geometry(base:Geometry,rng:random.Random,count:int=24):
    fields=('vertical_x','vertical_y','vertical_w','vertical_h','divider_x','divider_y','divider_w','divider_h','goal_y','decoy_y');children=[]
    for _ in range(count):
        d=asdict(base)
        for f in fields:
            if rng.random()<.70:d[f]=_clamp_round(f,float(d[f]),rng.choices((-0.2,-0.1,0,0.1,0.2),weights=(1,4,3,4,1),k=1)[0])
        d['goal_y']=min(5.5,round(max(float(d['goal_y']),float(d['divider_y'])+float(d['divider_h'])+0.5),1))
        d['decoy_y']=max(0.6,round(min(float(d['decoy_y']),float(d['vertical_y'])-0.2),1))
        g=Geometry(**d)
        if geometry_static_ok(g):children.append(g)
    return children

def _rank_key(t:Trial):
    # Structural heuristic only; no MATLAB-IG claim.
    return (-(t.metrics.get('clean_policy_decoys') or 0),-(t.metrics.get('selected_target_relevance_proxy') or 0),-(t.metrics.get('proxy_ratio') or 0))

def _evaluate_nominal(gs,cfg,workers):
    out=[]
    with ProcessPoolExecutor(max_workers=max(1,workers)) as ex:
        fs=[ex.submit(_trial,(g,cfg)) for g in gs]
        for f in as_completed(fs):
            t=f.result()
            if t.pass_contract:out.append(t)
    out.sort(key=_rank_key);return out

def _evaluate_robust(ts,cfg,workers):
    reps=[];wins=[]
    if not ts:return wins,reps
    with ProcessPoolExecutor(max_workers=max(1,workers)) as ex:
        fs=[ex.submit(_robust,(t,cfg)) for t in ts]
        for f in as_completed(fs):
            t,ok,p,total,minclean,fail=f.result();r={'trial':t,'pass':ok,'passed':p,'total':total,'min_clean_decoys':minclean,'failures':fail};reps.append(r)
            t.metrics['robust_cases_passed']=p;t.metrics['robust_cases_total']=total;t.metrics['min_clean_decoys_robust']=minclean
            if ok:wins.append(t)
    wins.sort(key=lambda t:(-(t.metrics.get('min_clean_decoys_robust') or 0),)+_rank_key(t));reps.sort(key=lambda r:(-r['passed'],-r['min_clean_decoys']))
    return wins,reps

def search_parallel(n=200,seed=7,cfg=Config(),workers=4,robustify_top=50,adaptive_generations=3,adaptive_children=24,adaptive_parents=12,target_robust_winners=20):
    rng=random.Random(seed);gs=[]
    while len(gs)<n:
        g=sample_geometry(rng)
        if geometry_static_ok(g):gs.append(g)
    nominal=_evaluate_nominal(gs,cfg,workers);wins,reps=_evaluate_robust(nominal,cfg,workers)
    print(f'ROBUST STRUCTURAL SCREEN: {len(wins)} full-pass / {len(nominal)} nominal clean')
    if wins:return nominal,wins
    seen={tuple(asdict(Geometry(**t.geometry)).items()) for t in nominal};allnom=list(nominal);parents=reps
    for gen in range(1,max(0,adaptive_generations)+1):
        pgs=[Geometry(**r['trial'].geometry) for r in parents[:max(1,adaptive_parents)]];children=[]
        for p in pgs:children.extend(mutate_geometry(p,rng,adaptive_children))
        children=[g for g in _dedupe_geometries(children) if tuple(asdict(g).items()) not in seen]
        for g in children:seen.add(tuple(asdict(g).items()))
        clean=_evaluate_nominal(children,cfg,workers);allnom.extend(clean);w,rr=_evaluate_robust(clean,cfg,workers);wins.extend(w)
        print(f'ADAPTIVE GEN {gen}: {len(children)} children, {len(clean)} clean, {len(w)} robust')
        if len(wins)>=target_robust_winners:break
        parents=(parents+rr);parents.sort(key=lambda r:(-r['passed'],-r['min_clean_decoys']));parents=parents[:max(adaptive_parents*3,24)]
    wins.sort(key=lambda t:(-(t.metrics.get('min_clean_decoys_robust') or 0),)+_rank_key(t));allnom.sort(key=_rank_key)
    return allnom,wins

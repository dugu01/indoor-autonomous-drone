from __future__ import annotations
from dataclasses import dataclass,asdict
import json, math, random
from .core import Config,build_truth,scan_truth,extract_frontiers,evaluate_candidates,select_candidate

@dataclass(frozen=True)
class Geometry:
    vertical_x: float=2.45
    vertical_y: float=2.10
    vertical_w: float=0.40
    vertical_h: float=2.05
    divider_x: float=2.85
    divider_y: float=3.40
    divider_w: float=3.00
    divider_h: float=0.40
    goal_x: float=5.05
    goal_y: float=5.0
    decoy_x: float=5.05
    decoy_y: float=1.15
    start_x: float=1.0
    start_y: float=3.0
    def rects(self):
        return [[self.vertical_x,self.vertical_y,self.vertical_w,self.vertical_h],
                [self.divider_x,self.divider_y,self.divider_w,self.divider_h]]

@dataclass
class Trial:
    geometry: dict
    pass_contract: bool
    selected: dict|None
    decoy: dict|None
    metrics: dict
    reasons: list[str]


def run_trial(g:Geometry,cfg:Config=Config(),scan_jitter=(0.0,0.0)) -> Trial:
    """Physical structural pre-screen, not MATLAB entropy parity.

    Required: target-relevant Tier-1 selection plus at least one distinct clean
    Tier-3 decoy that is safety-feasible and rejected only by policy. The proxy
    visibility ordering is recorded but is deliberately NOT a release condition.
    """
    xs,ys,occ=build_truth(cfg,g.rects())
    start=(g.start_x+scan_jitter[0],g.start_y+scan_jitter[1]); goal=(g.goal_x,g.goal_y)
    grid=scan_truth(cfg,xs,ys,occ,[start])
    fronts=extract_frontiers(grid,cfg)
    cand,_=evaluate_candidates(grid,fronts,start,goal,cfg)
    sel=select_candidate(cand)
    target_ids={q.frontier_track_id for q in cand if q.target_relevance_proxy>0}
    distinct=[q for q in cand if q.visible_unknown_proxy>0 and q.target_relevance_proxy<=0 and q.frontier_track_id not in target_ids]
    clean=[q for q in distinct if q.safety_feasible and q.tier==3 and q.policy_reasons==['IRRELEVANT_EXPLORATION']]
    dec=max(clean,key=lambda q:(q.visible_unknown_proxy,q.utility_proxy),default=None)
    reasons=[]
    if sel is None: reasons.append('NO_SELECTED_TARGET')
    elif not(sel.tier==1 and sel.target_relevance_proxy>0): reasons.append('SELECTED_NOT_TIER1')
    if dec is None: reasons.append('NO_CLEAN_SAFETY_FEASIBLE_DISTINCT_DECOY')
    if dec is not None and dec.safety_reasons: reasons.append('DECOY_HAS_SAFETY_REJECTION')
    if dec is not None and dec.policy_reasons!=['IRRELEVANT_EXPLORATION']: reasons.append('DECOY_NOT_POLICY_ONLY_IRRELEVANT')
    selected_proxy=None if sel is None else sel.visible_unknown_proxy
    decoy_proxy=None if dec is None else dec.visible_unknown_proxy
    ratio=None if dec is None or sel is None or selected_proxy<=0 else decoy_proxy/selected_proxy
    metrics={
        'frontiers':len(fronts),'candidates':len(cand),'accepted':sum(q.accepted for q in cand),
        'safety_feasible':sum(q.safety_feasible for q in cand),'target_frontier_ids':sorted(target_ids),
        'distinct_irrelevant_frontiers':len(set(q.frontier_track_id for q in distinct)),
        'clean_policy_decoys':len(clean),
        'selected_visible_unknown_proxy':selected_proxy,'decoy_visible_unknown_proxy':decoy_proxy,
        'proxy_ratio':ratio,
        'selected_target_relevance_proxy':None if sel is None else sel.target_relevance_proxy,
        'NOTE':'visible_unknown_proxy is structural only; it is not MATLAB entropy information gain',
    }
    return Trial(asdict(g),not reasons,None if sel is None else sel.to_dict(),None if dec is None else dec.to_dict(),metrics,reasons)


def sample_geometry(rng:random.Random) -> Geometry:
    q=lambda a,b: round(rng.randint(round(a*10),round(b*10))/10,1)
    vy=q(1.6,2.5); vh=q(1.6,2.4); dy=q(max(3.0,vy+0.7),4.0)
    return Geometry(vertical_x=q(2.2,2.8),vertical_y=vy,vertical_w=q(0.3,0.5),vertical_h=vh,
        divider_x=q(2.7,3.2),divider_y=dy,divider_w=q(2.4,3.1),divider_h=q(0.3,0.5),
        goal_y=q(max(4.5,dy+0.8),5.3),decoy_y=q(1.0,min(2.3,vy-0.3)))


def export_geometry(trial:Trial,path):
    g=trial.geometry; vy,vh=float(g['vertical_y']),float(g['vertical_h']);dy,dh=float(g['divider_y']),float(g['divider_h'])
    vx,vw=float(g['vertical_x']),float(g['vertical_w']);dx=float(g['divider_x'])
    geom={'schema':'S2_4_E_LITERAL_COMPETING_CORRIDORS_GEOMETRY_V1','room_xy_m':[6.0,6.0],
      'grid_resolution_m':0.1,'design_inflation_m':0.602,'start_xy_m':[g['start_x'],g['start_y']],
      'home_xy_m':[g['start_x'],g['start_y']],'goal_xy_m':[g['goal_x'],g['goal_y']],
      'decoy_probe_xy_m':[g['decoy_x'],g['decoy_y']],'junction_xy_m':[round(vx-0.4,3),g['start_y']],
      'obstacle_height_m':2.3,'obstacles_xywh_m':[[vx,vy,vw,vh],[dx,dy,g['divider_w'],dh]],
      'obstacle_names':['fork_occluder','east_branch_divider'],'target_branch_y_m':[round(dy+dh,3),6.0],
      'decoy_branch_y_m':[0.0,dy],'branch_analysis_x_min_m':round(max(vx+vw,dx)+0.7,3),
      'alternate_landing_zones_xy_m':[[1.0,1.0],[1.0,5.0]],'python_structural_backtest_contract':trial.metrics,
      'design_intent':'Python structural search only: target Tier-1 plus a distinct clean safety-feasible Tier-3 decoy. Physical MATLAB entropy ordering is observed, not forced.'}
    with open(path,'w') as f: json.dump(geom,f,indent=2)

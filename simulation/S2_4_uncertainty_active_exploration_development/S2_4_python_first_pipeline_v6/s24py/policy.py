from __future__ import annotations
from dataclasses import dataclass, asdict

@dataclass
class PolicyCandidate:
    candidate_id:int
    frontier_id:int
    tier:int
    target_relevance:float
    information_gain:float
    utility:float
    safety_reasons:list[str]
    policy_reasons:list[str]

    @property
    def safety_feasible(self)->bool:return not self.safety_reasons
    @property
    def accepted(self)->bool:return self.safety_feasible and not self.policy_reasons


def select_policy(candidates:list[PolicyCandidate])->PolicyCandidate|None:
    eligible=[q for q in candidates if q.accepted]
    if not eligible:return None
    return sorted(eligible,key=lambda q:(q.tier,-q.utility,-q.target_relevance,-q.information_gain,q.frontier_id,q.candidate_id))[0]


def adversarial_policy_contract()->dict:
    """Controlled policy contract, intentionally independent of physical-map entropy.

    The decoy is safety-feasible and has both higher raw information gain and
    higher raw utility, but is Tier-3 unrelated. The Tier-1 target must win.
    """
    target=PolicyCandidate(11,101,1,4.0,5.0,0.32,[],[])
    decoy=PolicyCandidate(12,202,3,0.0,12.0,0.47,[],['IRRELEVANT_EXPLORATION'])
    selected=select_policy([target,decoy])
    checks={
        'target_safety_feasible':target.safety_feasible,
        'decoy_safety_feasible':decoy.safety_feasible,
        'decoy_policy_only':decoy.policy_reasons==['IRRELEVANT_EXPLORATION'],
        'decoy_more_informative':decoy.information_gain>target.information_gain,
        'decoy_raw_utility_higher':decoy.utility>target.utility,
        'target_tier1':target.tier==1 and target.target_relevance>0,
        'decoy_tier3':decoy.tier==3 and decoy.target_relevance==0,
        'target_selected':selected is not None and selected.candidate_id==target.candidate_id,
    }
    return {'schema':'S2_4_ADVERSARIAL_POLICY_CONTRACT_V1','target':asdict(target),'decoy':asdict(decoy),
            'selected_candidate_id':None if selected is None else selected.candidate_id,
            'checks':checks,'pass':all(checks.values())}


def physical_layered_gate(g:dict, require_benchmark_activation:bool=True)->dict:
    """Pure regression helper mirroring the v6 physical MATLAB gate.

    The fixed benchmark seed must instantiate a clean Tier-3 decoy. Robustness
    seeds must all complete safely and select the target, but are not failed
    merely because stochastic perception did not instantiate the clean decoy.
    Whenever a clean decoy is present, the same target/no-irrelevant-selection
    conditions still apply. Physical IG ordering remains observation-only.
    """
    common=(
        'geometryPass','missionPass','goalReached','rtlAndLanding','zeroCollision','zeroGeofence',
        'zeroUnknownCommitment','zeroUnsafeExecution','truthIsolation','targetSelected','noIrrelevantSelection'
    )
    checks={k:bool(g.get(k,False)) for k in common}
    if require_benchmark_activation:
        checks['cleanDecoyFound']=bool(g.get('cleanDecoyFound',False))
    return {'checks':checks,'require_benchmark_activation':require_benchmark_activation,
            'physical_decoy_more_informative_observed':bool(g.get('decoyMoreInformative',False)),
            'pass':all(checks.values())}


def multiseed_layered_gate(records:list[dict], benchmark_seed:int=0)->dict:
    """Regression model for the v6 benchmark-vs-robustness split."""
    if not records:
        return {'pass':False,'reason':'NO_RECORDS'}
    by_seed={int(r['seed']):r for r in records}
    if benchmark_seed not in by_seed:
        return {'pass':False,'reason':'BENCHMARK_SEED_MISSING'}
    bench=physical_layered_gate(by_seed[benchmark_seed],True)
    per_seed={int(r['seed']):physical_layered_gate(r,False) for r in records}
    activated=[int(r['seed']) for r in records if bool(r.get('cleanDecoyFound',False))]
    return {
        'schema':'S2_4_LAYERED_MULTI_SEED_V6',
        'benchmark_seed':benchmark_seed,
        'benchmark_pass':bench['pass'],
        'per_seed_pass':{k:v['pass'] for k,v in per_seed.items()},
        'clean_decoy_activation_seeds':activated,
        'clean_decoy_activation_count':len(activated),
        'pass':bench['pass'] and all(v['pass'] for v in per_seed.values()),
    }

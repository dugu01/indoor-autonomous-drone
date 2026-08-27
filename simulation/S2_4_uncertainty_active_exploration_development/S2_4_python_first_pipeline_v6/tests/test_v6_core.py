from dataclasses import replace
import numpy as np
from s24py.core import Config,Grid,build_truth,scan_truth,extract_frontiers,evaluate_candidates,select_candidate,visible_unknown_from
from s24py.scenario2 import Geometry,run_trial
from s24py.policy import adversarial_policy_contract,physical_layered_gate


def test_controlled_adversarial_policy_contract():
    r=adversarial_policy_contract();assert r['pass'],r
    assert r['decoy']['information_gain']>r['target']['information_gain']
    assert r['decoy']['utility']>r['target']['utility']
    assert r['decoy']['safety_reasons']==[]
    assert r['decoy']['policy_reasons']==['IRRELEVANT_EXPLORATION']
    assert r['selected_candidate_id']==r['target']['candidate_id']


def test_decision_is_invariant_to_truth_when_estimated_map_fixed():
    cfg=Config();g=Geometry();xs,ys,truth=build_truth(cfg,g.rects());grid=scan_truth(cfg,xs,ys,truth,[(g.start_x,g.start_y)])
    fronts=extract_frontiers(grid,cfg)
    c1,_=evaluate_candidates(grid,fronts,(g.start_x,g.start_y),(g.goal_x,g.goal_y),cfg)
    # Corrupt truth completely while keeping autonomy-visible map identical.
    grid2=Grid(grid.xs,grid.ys,~grid.truth_occ.copy(),grid.known_free.copy(),grid.known_occ.copy(),grid.unknown.copy(),grid.executable.copy(),grid.resolution,grid.inflation)
    c2,_=evaluate_candidates(grid2,fronts,(g.start_x,g.start_y),(g.goal_x,g.goal_y),cfg)
    sig=lambda cs:[(q.candidate_id,q.frontier_track_id,q.rc,q.visible_unknown_proxy,q.target_relevance_proxy,q.tier,q.utility_proxy,tuple(q.safety_reasons),tuple(q.policy_reasons),q.accepted) for q in cs]
    assert sig(c1)==sig(c2)
    s1=select_candidate(c1);s2=select_candidate(c2)
    assert (None if s1 is None else s1.candidate_id)==(None if s2 is None else s2.candidate_id)


def test_unknown_occludes_structural_visibility():
    n=21;xs=np.arange(n)*0.1;ys=np.arange(n)*0.1
    known_free=np.zeros((n,n),bool);known_occ=np.zeros((n,n),bool);unknown=np.ones((n,n),bool)
    r=10
    # Known-free ray from x=0.2 through x=0.9; x>=1.0 remains unknown.
    known_free[r,2:10]=True;unknown[r,2:10]=False
    executable=known_free.copy();truth=np.zeros((n,n),bool)
    grid=Grid(xs,ys,truth,known_free,known_occ,unknown,executable,0.1,0.0)
    cfg=Config(view_range=1.5,view_fov_deg=0.0,view_rays=1)
    cells=visible_unknown_from(grid,(0.2,1.0),0.0,cfg)
    assert cells==[(r,10)],cells


def test_v4_matlab_fixture_passes_v6_benchmark_layer():
    # Exact logical fields from user's v4 candidate-1 MATLAB output. Physical
    # IG ratio was 0.872<1, yet every integration/safety requirement passed.
    fixture={'geometryPass':True,'missionPass':True,'goalReached':True,'rtlAndLanding':True,
        'zeroCollision':True,'zeroGeofence':True,'zeroUnknownCommitment':True,'zeroUnsafeExecution':True,
        'truthIsolation':True,'cleanDecoyFound':True,'targetSelected':True,'noIrrelevantSelection':True,'decoyMoreInformative':False}
    r=physical_layered_gate(fixture);assert r['pass'],r
    assert r['physical_decoy_more_informative_observed'] is False


def test_trial_uses_proxy_names_not_information_gain_claim():
    tr=run_trial(Geometry(),Config())
    assert 'selected_visible_unknown_proxy' in tr.metrics
    assert 'decoy_visible_unknown_proxy' in tr.metrics
    assert 'selected_ig' not in tr.metrics and 'decoy_ig' not in tr.metrics


def test_v6_multiseed_benchmark_activation_is_not_required_every_seed():
    from s24py.policy import multiseed_layered_gate
    common={'geometryPass':True,'missionPass':True,'goalReached':True,'rtlAndLanding':True,
        'zeroCollision':True,'zeroGeofence':True,'zeroUnknownCommitment':True,'zeroUnsafeExecution':True,
        'truthIsolation':True,'targetSelected':True,'noIrrelevantSelection':True,'decoyMoreInformative':False}
    seed0=dict(common,seed=0,cleanDecoyFound=True)
    # Exact logical pattern of the user's v5 seed-1 failure: mission/safety and
    # target selection passed; only clean-decoy activation was absent.
    seed1=dict(common,seed=1,cleanDecoyFound=False)
    r=multiseed_layered_gate([seed0,seed1],benchmark_seed=0)
    assert r['pass'],r
    assert r['benchmark_pass']
    assert r['clean_decoy_activation_seeds']==[0]


def test_v6_benchmark_seed_still_requires_clean_decoy():
    from s24py.policy import multiseed_layered_gate
    common={'geometryPass':True,'missionPass':True,'goalReached':True,'rtlAndLanding':True,
        'zeroCollision':True,'zeroGeofence':True,'zeroUnknownCommitment':True,'zeroUnsafeExecution':True,
        'truthIsolation':True,'targetSelected':True,'noIrrelevantSelection':True,'decoyMoreInformative':False}
    r=multiseed_layered_gate([dict(common,seed=0,cleanDecoyFound=False),dict(common,seed=1,cleanDecoyFound=True)],0)
    assert not r['pass'] and not r['benchmark_pass']


def test_v6_any_robustness_seed_mission_or_policy_failure_still_fails():
    from s24py.policy import multiseed_layered_gate
    common={'geometryPass':True,'missionPass':True,'goalReached':True,'rtlAndLanding':True,
        'zeroCollision':True,'zeroGeofence':True,'zeroUnknownCommitment':True,'zeroUnsafeExecution':True,
        'truthIsolation':True,'targetSelected':True,'noIrrelevantSelection':True,'decoyMoreInformative':False}
    s0=dict(common,seed=0,cleanDecoyFound=True)
    s1=dict(common,seed=1,cleanDecoyFound=False,zeroCollision=False)
    assert not multiseed_layered_gate([s0,s1],0)['pass']
    s1=dict(common,seed=1,cleanDecoyFound=False,noIrrelevantSelection=False)
    assert not multiseed_layered_gate([s0,s1],0)['pass']

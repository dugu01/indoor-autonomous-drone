from pathlib import Path
import json,subprocess,sys,tempfile,shutil
from s24py.core import Config
from s24py.scenario2 import Geometry,run_trial
from s24py.matlab_bridge import geometry_payload
from s24py.search_parallel import geometry_static_ok,search_parallel

ROOT=Path(__file__).resolve().parents[1]
RES=ROOT/'resources'/'project_validation_compat'

def test_matlab_export_exact_v1_schema():
    # Trial need not be a release winner for schema testing.
    t=run_trial(Geometry(),Config());d=geometry_payload(t)
    assert d['schema']=='S2_4_E_LITERAL_COMPETING_CORRIDORS_GEOMETRY_V1'
    for k in ('junction_xy_m','obstacle_height_m','target_branch_y_m','decoy_branch_y_m','branch_analysis_x_min_m','alternate_landing_zones_xy_m'):
        assert k in d
    assert 'python_structural_backtest_contract' in d
    assert 'python_backtest_contract' not in d


def test_geometry_audit_no_route_is_normal_fail_not_traceback():
    with tempfile.TemporaryDirectory() as td:
        root=Path(td);v=root/'coupled'/'validation';s=root/'coupled'/'scenarios';v.mkdir(parents=True);s.mkdir(parents=True)
        shutil.copy2(RES/'audit_S2_4_E_literal_corridor_geometry.py',v/'audit_S2_4_E_literal_corridor_geometry.py')
        # Deliberately block both routes while retaining schema. Audit must return
        # FAIL with N/A, not TypeError formatting None.
        geom={'schema':'S2_4_E_LITERAL_COMPETING_CORRIDORS_GEOMETRY_V1','room_xy_m':[6,6],'grid_resolution_m':0.1,'design_inflation_m':0.602,
          'start_xy_m':[1,3],'home_xy_m':[1,3],'goal_xy_m':[5.05,5.0],'decoy_probe_xy_m':[5.05,1.0],
          'obstacles_xywh_m':[[2.4,0.0,0.5,6.0],[3.0,3.0,2.4,0.5]],'target_branch_y_m':[3.5,6.0],'decoy_branch_y_m':[0,3.0],
          'branch_analysis_x_min_m':3.7}
        (s/'literal_competing_corridors_geometry.json').write_text(json.dumps(geom))
        cp=subprocess.run([sys.executable,str(v/'audit_S2_4_E_literal_corridor_geometry.py')],cwd=root,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        assert cp.returncode==1,cp.stdout
        assert 'N/A' in cp.stdout,cp.stdout
        assert 'Traceback' not in cp.stdout and 'TypeError' not in cp.stdout,cp.stdout
        assert 'RESULT: FAIL' in cp.stdout


def test_matlab_v6_sources_split_benchmark_activation_from_multiseed_robustness():
    m2=(RES/'validate_S2_4_E_milestone_2.m').read_text();multi=(RES/'validate_S2_4_E_competing_corridors_multiseed.m').read_text();test=(RES/'test_S2_4_E_competing_decision_contract.m').read_text()
    assert 'S2_4_E_LAYERED_V6' in m2 and 'cleanFeasibleDecoyPresent' in m2
    assert 'physicalDecoyMoreInformativeObserved' in m2
    # Explicit required list must not include the physical IG observation.
    req=m2.split('required={',1)[1].split('};',1)[0]
    assert 'physicalDecoyMoreInformativeObserved' not in req and 'decoyStrictlyMoreInformative' not in req
    assert 'adversarialPolicyContract' in req
    assert 'S2_4_E_LAYERED_V6' in multi
    assert 'benchmarkActivationPass' in multi and 'cleanDecoyActivationCount' in multi
    assert 'all(CleanDecoyFound==1)' not in multi
    assert 'all(DecoyMoreInformative==1)' not in multi
    assert 'decoyHasMoreInformation' in test and 'decoyHasHigherRawUtility' in test and 'decoyPolicyOnly' in test


def test_known_physical_geometry_is_robust_under_v5_structural_suite():
    from s24py.search_parallel import robust_trial
    g=Geometry()
    assert geometry_static_ok(g)
    t=run_trial(g,Config())
    assert t.pass_contract,t.reasons
    rr=robust_trial(t,Config())
    assert rr['pass'],rr['failures']
    assert rr['min_clean_decoys']>=1
    assert t.decoy is not None
    assert t.decoy['safety_reasons']==[]
    assert t.decoy['policy_reasons']==['IRRELEVANT_EXPLORATION']


def test_v6_matlab_bridge_marks_benchmark_and_robustness_modes(monkeypatch,tmp_path):
    from types import SimpleNamespace
    from s24py import matlab_bridge as mb
    project=tmp_path/'project';project.mkdir()
    package=tmp_path/'package';(package/'matlab').mkdir(parents=True)
    (package/'matlab'/'python_first_gate.m').write_text('% fixture')
    seen=[]
    def fake_run(cmd,**kwargs):
        seen.append(cmd)
        return SimpleNamespace(returncode=0,stdout='ok')
    monkeypatch.setattr(mb.subprocess,'run',fake_run)
    for mode in (True,False):
        out=tmp_path/f'{mode}.json';out.write_text('{"pass":true}')
        r=mb.run_matlab_parity(project,package,1,out,'fake-matlab',require_benchmark_activation=mode)
        assert r['gate']['pass'] is True
    assert ',true);' in seen[0][2]
    assert ',false);' in seen[1][2]


def test_v6_native_release_gate_passes_explicit_benchmark_seed(monkeypatch,tmp_path):
    from types import SimpleNamespace
    from s24py import matlab_bridge as mb
    project=tmp_path/'project';project.mkdir();seen=[]
    class FakePopen:
        def __init__(self,cmd,**kwargs):
            seen.append(cmd);self.stdout=iter(['ok\n'])
        def wait(self,timeout=None):return 0
    monkeypatch.setattr(mb.subprocess,'Popen',FakePopen)
    rc=mb.run_native_release_gate(project,'fake-matlab','0:9',tmp_path/'native.log',benchmark_seed=0)
    assert rc==0
    assert 'validate_S2_4_E_competing_corridors_multiseed(0:9,0)' in seen[0][2]

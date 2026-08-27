#!/usr/bin/env python3
from __future__ import annotations
import argparse,hashlib,json,shutil,subprocess,sys,time
from dataclasses import asdict
from pathlib import Path
from s24py.core import Config
from s24py.policy import adversarial_policy_contract
from s24py.scenario2 import Geometry,run_trial,export_geometry
from s24py.search_parallel import search_parallel, robust_trial
from s24py.matlab_bridge import write_geometry,discover_matlab,run_matlab_parity,run_native_release_gate
from s24py.project_compat import ensure_project_validation_compat

def sha256(p:Path)->str:
    h=hashlib.sha256();
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()

def run_logged(cmd,log,cwd=None)->int:
    log=Path(log);log.parent.mkdir(parents=True,exist_ok=True);print('\n$',' '.join(map(str,cmd)));print('[log]',log)
    with log.open('w',encoding='utf-8') as f:
        p=subprocess.Popen(cmd,cwd=cwd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1);assert p.stdout is not None
        for line in p.stdout:print(line,end='');f.write(line)
        return p.wait()

def parse_seeds(expr):
    x=expr.strip()
    if ':' in x:a,b=[int(q) for q in x.split(':',1)];return list(range(a,b+1))
    return [int(q.strip()) for q in x.split(',') if q.strip()]

def backup(path,out,label):
    if not path.exists():return None
    d=Path(out)/'matlab_backup';d.mkdir(parents=True,exist_ok=True);p=d/f'{label}_{time.strftime("%Y%m%d_%H%M%S")}{path.suffix}';shutil.copy2(path,p);return p

def main():
    ap=argparse.ArgumentParser(description='S2.4 Python-first layered validation pipeline v6')
    ap.add_argument('--trials',type=int,default=1000);ap.add_argument('--seed',type=int,default=7);ap.add_argument('--workers',type=int,default=4)
    ap.add_argument('--force-search',action='store_true',help='Search new physical geometries even when the current v0.3.6 geometry already passes the v6 structural robustness gate')
    ap.add_argument('--robustify-top',type=int,default=50);ap.add_argument('--out',default='out');ap.add_argument('--matlab-project',default='');ap.add_argument('--matlab-bin',default='auto')
    ap.add_argument('--matlab-seeds',default='0:9');ap.add_argument('--benchmark-seed',type=int,default=0);ap.add_argument('--matlab-top',type=int,default=20);ap.add_argument('--skip-native-release-gate',action='store_true')
    ap.add_argument('--adaptive-generations',type=int,default=3);ap.add_argument('--adaptive-children',type=int,default=24);ap.add_argument('--adaptive-parents',type=int,default=12);ap.add_argument('--target-robust-winners',type=int,default=20)
    a=ap.parse_args();root=Path(__file__).resolve().parent;out=(root/a.out).resolve();out.mkdir(parents=True,exist_ok=True)
    state={'schema':'S2_4_PYTHON_FIRST_PIPELINE_REPORT_V6','python_pass':False,'matlab_pass':None}
    rc=run_logged([sys.executable,'-m','pytest','-q'],out/'pytest.log',root)
    if rc:raise SystemExit(rc)
    policy=adversarial_policy_contract();state['python_adversarial_policy_contract']=policy;print('\nCONTROLLED ADVERSARIAL POLICY CONTRACT:', 'PASS' if policy['pass'] else 'FAIL')
    print('target IG/utility:',policy['target']['information_gain'],policy['target']['utility'],'decoy IG/utility:',policy['decoy']['information_gain'],policy['decoy']['utility'])
    if not policy['pass']:raise SystemExit(3)
    project=None;matlab_exe=None
    if a.matlab_project:
        project=Path(a.matlab_project).expanduser().resolve();
        if not project.is_dir():raise SystemExit(f'MATLAB project not found: {project}')
        state['project_compat']=ensure_project_validation_compat(project,root,out)
        static_script=project/'coupled'/'validation'/'run_all_checks_S2_4_E.py';rc=run_logged([sys.executable,str(static_script)],out/'project_static_gate_before_search.log',project)
        if rc:state['failure']='PROJECT_STATIC_GATE_FAILED_BEFORE_SEARCH';(out/'pipeline_report.json').write_text(json.dumps(state,indent=2,default=str)+'\n');raise SystemExit(rc)
        matlab_exe=discover_matlab(a.matlab_bin);state['matlab_executable']=matlab_exe
    cfg=Config();baseline=run_trial(Geometry(),cfg);baseline_robust=robust_trial(baseline,cfg) if baseline.pass_contract else {'pass':False,'failures':[{'case':'nominal','reasons':baseline.reasons}]}
    print('\nCURRENT PHYSICAL STRUCTURAL BASELINE');print('pass:',baseline.pass_contract,'reasons:',baseline.reasons,'metrics:',baseline.metrics)
    print('robust structural suite:', 'PASS' if baseline_robust['pass'] else 'FAIL', f"{baseline_robust.get('passed',0)}/{baseline_robust.get('total',11)}")
    if baseline_robust['pass'] and not a.force_search:
        nominal=[baseline];winners=[baseline];baseline.metrics['robust_cases_passed']=baseline_robust['passed'];baseline.metrics['robust_cases_total']=baseline_robust['total'];baseline.metrics['min_clean_decoys_robust']=baseline_robust['min_clean_decoys']
        print('REUSE CURRENT PHYSICAL GEOMETRY: structural + robustness contract already passes; no geometry search needed.')
    else:
        t=time.time();nominal,winners=search_parallel(a.trials,a.seed,cfg,a.workers,a.robustify_top,a.adaptive_generations,a.adaptive_children,a.adaptive_parents,a.target_robust_winners)
        print(f'\nSEARCH: {len(nominal)} nominal structural clean / {a.trials}; {len(winners)} robust structural winners in {time.time()-t:.2f}s')
    if not winners:state['failure']='NO_ROBUST_STRUCTURAL_PYTHON_WINNER';(out/'pipeline_report.json').write_text(json.dumps(state,indent=2,default=str)+'\n');raise SystemExit(2)
    top=winners[:min(a.matlab_top,len(winners))];(out/'robust_winners_top20.json').write_text(json.dumps([asdict(x) for x in winners[:20]],indent=2,default=str)+'\n')
    best=top[0];export=out/'literal_competing_corridors_geometry.json';export_geometry(best,export)
    print('\nBEST PYTHON STRUCTURAL CONTRACT');print('geometry:',best.geometry);print('clean decoys:',best.metrics['clean_policy_decoys'],'selected target relevance proxy:',best.metrics['selected_target_relevance_proxy']);print('visibility proxy ratio (non-MATLAB):',best.metrics['proxy_ratio']);print('robustness: PASS')
    state.update({'python_pass':True,'trials':a.trials,'winner_count':len(winners),'best':asdict(best),'geometry_sha256':sha256(export)})
    if project is None:(out/'pipeline_report.json').write_text(json.dumps(state,indent=2,default=str)+'\n');print('\nMATLAB stage not invoked.');print('PIPELINE RESULT: PASS');return
    target=project/'coupled'/'scenarios'/'literal_competing_corridors_geometry.json';previous=backup(target,out,'literal_competing_corridors_geometry');state['previous_geometry_backup']=None if previous is None else str(previous)
    seeds=parse_seeds(a.matlab_seeds);benchmark_seed=int(a.benchmark_seed);
    if benchmark_seed not in seeds: seeds=[benchmark_seed]+seeds
    records=[];chosen=None;geom_audit=project/'coupled'/'validation'/'audit_S2_4_E_literal_corridor_geometry.py'
    try:
        for idx,tr in enumerate(top,1):
            cdir=out/'matlab_candidate_sweep'/f'candidate_{idx:03d}';cdir.mkdir(parents=True,exist_ok=True);cand_json=cdir/'geometry.json';write_geometry(tr,cand_json);shutil.copy2(cand_json,target)
            print(f'\n=== MATLAB PHYSICAL CANDIDATE {idx}/{len(top)} ===');print('Python structural clean decoys:',tr.metrics.get('clean_policy_decoys'),'proxy ratio:',tr.metrics.get('proxy_ratio'))
            rc=run_logged([sys.executable,str(geom_audit)],cdir/'geometry_audit.log',project);rec={'candidate_index':idx,'python':asdict(tr),'geometry_audit_pass':rc==0,'seeds':[]}
            if rc:records.append(rec);continue
            allpass=True
            ordered=[benchmark_seed]+[q for q in seeds if q!=benchmark_seed]
            for seed in ordered:
                require_benchmark=(seed==benchmark_seed)
                r=run_matlab_parity(project,root,seed,cdir/f'seed_{seed:03d}.json',matlab_exe,require_benchmark_activation=require_benchmark)
                rec['seeds'].append(r);gp=r.get('gate',{}).get('pass') is True and r.get('returncode')==0
                mode='BENCHMARK' if require_benchmark else 'ROBUSTNESS'
                print(f"seed {seed} [{mode}]: {'PASS' if gp else 'FAIL'}",r.get('gate',{}))
                if not gp:allpass=False;break
            rec['all_seed_pass']=allpass;records.append(rec)
            if allpass:chosen=tr;shutil.copy2(cand_json,out/'FINAL_MATLAB_GEOMETRY.json');break
        (out/'matlab_candidate_sweep_report.json').write_text(json.dumps(records,indent=2,default=str)+'\n')
        if chosen is None:
            if previous is not None:shutil.copy2(previous,target)
            state['matlab_pass']=False;state['failure']='NO_STRUCTURAL_WINNER_PASSED_MATLAB_PHYSICAL_MULTISEED';state['geometry_restored']=previous is not None;(out/'pipeline_report.json').write_text(json.dumps(state,indent=2,default=str)+'\n');raise SystemExit(6)
        state['chosen']=asdict(chosen);state['matlab_physical_multiseed_pass']=True
        if not a.skip_native_release_gate:
            rc=run_native_release_gate(project,matlab_exe,a.matlab_seeds,out/'matlab_native_release_gate.log',benchmark_seed=benchmark_seed)
            if rc:
                if previous is not None:shutil.copy2(previous,target)
                state['matlab_pass']=False;state['failure']='NATIVE_LAYERED_MATLAB_RELEASE_GATE_FAILED';state['geometry_restored']=previous is not None;(out/'pipeline_report.json').write_text(json.dumps(state,indent=2,default=str)+'\n');raise SystemExit(rc)
            state['native_matlab_layered_release_gate_pass']=True
        state['matlab_pass']=True;state['installed_geometry_sha256']=sha256(target);(out/'pipeline_report.json').write_text(json.dumps(state,indent=2,default=str)+'\n')
        print('\nPYTHON POLICY + BENCHMARK ACTIVATION + MATLAB ALL-SEED ROBUSTNESS + NATIVE LAYERED RELEASE: PASS');print('FINAL GEOMETRY:',target);print('PIPELINE RESULT: PASS')
    except BaseException:
        if chosen is None and previous is not None and target.exists():shutil.copy2(previous,target)
        (out/'pipeline_report.json').write_text(json.dumps(state,indent=2,default=str)+'\n');raise
if __name__=='__main__':main()

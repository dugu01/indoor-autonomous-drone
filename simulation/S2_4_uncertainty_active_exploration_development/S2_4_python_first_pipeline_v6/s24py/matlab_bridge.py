from __future__ import annotations
from pathlib import Path
import json, shutil, subprocess, time
from .scenario2 import Trial

def geometry_payload(trial:Trial)->dict:
    g=trial.geometry;vy,vh=float(g['vertical_y']),float(g['vertical_h']);dy,dh=float(g['divider_y']),float(g['divider_h']);vx,vw=float(g['vertical_x']),float(g['vertical_w']);dx=float(g['divider_x'])
    return {'schema':'S2_4_E_LITERAL_COMPETING_CORRIDORS_GEOMETRY_V1','room_xy_m':[6.0,6.0],'grid_resolution_m':0.1,'design_inflation_m':0.602,
        'start_xy_m':[float(g['start_x']),float(g['start_y'])],'home_xy_m':[float(g['start_x']),float(g['start_y'])],
        'goal_xy_m':[float(g['goal_x']),float(g['goal_y'])],'decoy_probe_xy_m':[float(g['decoy_x']),float(g['decoy_y'])],
        'junction_xy_m':[round(vx-0.4,3),float(g['start_y'])],'obstacle_height_m':2.3,
        'obstacles_xywh_m':[[vx,vy,vw,vh],[dx,dy,float(g['divider_w']),dh]],'obstacle_names':['fork_occluder','east_branch_divider'],
        'target_branch_y_m':[round(dy+dh,3),6.0],'decoy_branch_y_m':[0.0,dy],'branch_analysis_x_min_m':round(max(vx+vw,dx)+0.7,3),
        'alternate_landing_zones_xy_m':[[1.0,1.0],[1.0,5.0]],'python_structural_backtest_contract':trial.metrics,
        'design_intent':'v6 Python structural physical pre-screen. MATLAB physical entropy is observed; controlled adversarial IG_D>IG_T is validated separately.'}

def write_geometry(trial,path):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(geometry_payload(trial),indent=2)+'\n');return path

def matlab_quote(s:str)->str:return s.replace("'","''")
def discover_matlab(requested='auto'):
    if requested and requested!='auto':
        p=Path(requested).expanduser()
        if p.exists():return str(p.resolve())
        q=shutil.which(requested)
        if q:return q
        raise FileNotFoundError(f'MATLAB executable not found: {requested}')
    q=shutil.which('matlab')
    if q:return q
    apps=sorted(Path('/Applications').glob('MATLAB_R*.app/bin/matlab'),reverse=True)
    if apps:return str(apps[0])
    raise FileNotFoundError('MATLAB executable not found')

def run_matlab_parity(project,package_root,seed,out_json,matlab_exe,require_benchmark_activation=False,timeout_s=1800):
    out_json=Path(out_json);out_json.parent.mkdir(parents=True,exist_ok=True);mdir=Path(package_root)/'matlab'
    expr=(f"addpath('{matlab_quote(str(mdir))}'); o=python_first_gate('{matlab_quote(str(project))}',{int(seed)},'{matlab_quote(str(out_json))}',{str(bool(require_benchmark_activation)).lower()}); "
          "if ~o.pass, error('S2_4:PythonFirstPhysicalGateFailed','Python-first physical gate failed'); end")
    log=out_json.with_suffix('.matlab.log');t0=time.time();cp=subprocess.run([matlab_exe,'-batch',expr],cwd=project,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=timeout_s)
    log.write_text(cp.stdout or '',encoding='utf-8');result={'returncode':cp.returncode,'elapsed_s':time.time()-t0,'log':str(log),'json':str(out_json)}
    if out_json.exists():
        try:result['gate']=json.loads(out_json.read_text())
        except Exception as exc:result['json_error']=repr(exc)
    return result

def run_native_release_gate(project,matlab_exe,seeds_expr,log,benchmark_seed=0,timeout_s=7200):
    log=Path(log);log.parent.mkdir(parents=True,exist_ok=True);pq=matlab_quote(str(project))
    expr=(f"projectRoot='{pq}';cd(projectRoot);restoredefaultpath;setup_S2_4_E_path(projectRoot);clear functions;rehash;"
          "p=test_S2_4_E_competing_decision_contract();disp(p);assert(p.pass);"
          "g=test_S2_4_E_literal_corridor_geometry_contract();disp(g);assert(g.pass);"
          "gate2=validate_S2_4_E_milestone_2();disp(gate2);assert(gate2.pass);"
          f"multi=validate_S2_4_E_competing_corridors_multiseed({seeds_expr},{int(benchmark_seed)});disp(multi);assert(multi.pass);")
    with log.open('w',encoding='utf-8') as f:
        p=subprocess.Popen([matlab_exe,'-batch',expr],cwd=project,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        assert p.stdout is not None
        for line in p.stdout:print(line,end='');f.write(line)
        return p.wait(timeout=timeout_s)

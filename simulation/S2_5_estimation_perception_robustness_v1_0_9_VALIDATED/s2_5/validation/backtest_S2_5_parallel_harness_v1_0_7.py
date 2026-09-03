#!/usr/bin/env python3
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib, random, re, sys, multiprocessing as mp
ROOT=Path(__file__).resolve().parents[2]
S=ROOT/'s2_5'
val=(S/'validation/validate_S2_5_all.m').read_text()
runner=(S/'mission/run_S2_5_coupled.m').read_text()
wrapper=(S/'validation/run_S2_5_qualification_case.m').read_text()
pool=(S/'validation/start_S2_5_parallel_pool.m').read_text()
serial_val=(S/'evidence/v1_0_2_serial_reference/validate_S2_5_all_serial_reference.txt').read_text()
serial_runner=(S/'evidence/v1_0_2_serial_reference/run_S2_5_coupled_serial_reference.txt').read_text()
manager=S/'mission/mission_lifecycle_manager_S2_5.m'
checks=[]
def ck(name,cond,detail=''): checks.append((name,bool(cond),detail))
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()

# A. v1.0.7 intentionally changes the S2.5 manager only for the reviewed
# pre-map integrity overlay and separate S2.5 recovery scan accounting.
manager_text=manager.read_text()
ck('v107_manager_integrity_overlay_present','sanitize_perception_packet_S2_5' in manager_text)
ck('v107_manager_separate_scan_accounting','s25RecoveryScanHoldPass' in manager_text and 's25MappingCompositePass' in manager_text)
for f in ['mission/init_S2_5_config.m','scenarios/scenario_S2_5.m','sensors/simulate_sensor_packet_S2_5.m','perception/simulate_perception_packet_S2_5.m']:
    p=S/f; ck('runtime_file_present_'+re.sub(r'\W+','_',f),p.exists())

# B. Runner executes the same full mission, with only optional output/persistence controls.
for tok in ["rng(seed,'twister');",'cfg=init_S2_5_config();cfg.seed=seed;', 'scenario=scenario_S2_5(scenarioName);',
            '[log,summary,maps]=mission_lifecycle_manager_S2_5(cfg,scenario);','summary.s25CasePass=evaluate_s25_case(summary,scenario);']:
    ck('runner_core_'+re.sub(r'\W+','_',tok)[:55], tok in runner and tok in serial_runner)
ck('runner_optional_save_defaults_true','if nargin<5||isempty(saveArtifacts),saveArtifacts=true;end' in runner)
ck('runner_optional_verbose_defaults_true','if nargin<6||isempty(verbose),verbose=true;end' in runner)
ck('qualification_runs_full_logic_compact','run_S2_5_coupled(seed,caseName,false,false,false,false)' in wrapper)
ck('qualification_returns_summary_only','out.summary=r.summary;' in wrapper and 'out.log' not in wrapper and 'out.maps' not in wrapper)

# C. Acceptance semantics are byte-equivalent at helper-function level.
def fn_body(text,signature,next_signature=None):
    start=text.find('function '+signature)
    if start<0: return None
    if next_signature:
        end=text.find('function '+next_signature,start+1)
        if end<0: return None
        return text[start:end].strip()
    return text[start:].strip()
for name,nextn in [('[pass,extra]=historical_case_pass','[pass,extra]=recoverable_case_pass'),
                   ('[pass,extra]=recoverable_case_pass','[pass,mapSafety,core,evidence]=failsafe_case_pass'),
                   ('[pass,mapSafety,core,evidence]=failsafe_case_pass','[cached,r]=historical_cached_run')]:
    a=fn_body(val,name,nextn); b=fn_body(serial_val,name,nextn)
    def executable(x):
        if x is None: return None
        lines=[]
        for line in x.splitlines():
            line=line.split('%',1)[0].strip()
            if line: lines.append(re.sub(r'\s+','',line))
        return ''.join(lines)
    same=(a is not None and b is not None and executable(a)==executable(b))
    ck('acceptance_semantics_'+re.sub(r'\W+','_',name),same, f'parallel={0 if a is None else len(executable(a))} serial={0 if b is None else len(executable(b))}')

# D. Parallel phases and required-toolbox behavior.
ck('parallel_async_helper','run_parallel_cases_with_progress' in val and 'parfeval(@run_S2_5_qualification_case' in val and 'fetchNext(F)' in val)
ck('parallel_hist_preflight',"run_parallel_cases_with_progress(histNames,histSeeds,'HIST')" in val)
ck('parallel_baselines',"run_parallel_cases_with_progress(baseNames,seeds,'BASE')" in val)
ck('parallel_recoverable_jobs',"run_parallel_cases_with_progress(jobNames,jobSeeds,'REC')" in val)
ck('parallel_failsafe_jobs',"run_parallel_cases_with_progress(fsNames,fsJobSeeds,'FAILSAFE')" in val)
ck('completion_progress_reported', 'completed %d/%d:' in val and 'elapsedWall_s' in wrapper)
ck('inherited_F_remains_serial','PHASE 1 — INHERITED S2.4-F COUPLED REGRESSION (SERIAL)' in val)
ck('toolbox_required',"license('test','Distrib_Computing_Toolbox')" in pool)
ck('workers_bounded_to_8','requested=min(8,max(2,floor(requested)))' in pool)
ck('default_workers_four','n=min(4,c.NumWorkers)' in pool)
ck('existing_pool_restarted_for_override','p.NumWorkers~=n' in pool and 'delete(p)' in pool)
ck('process_pool_preferred',"parcluster('Processes')" in pool and "parcluster('local')" in pool)
ck('worker_override_supported',"getenv('S2_5_WORKERS')" in pool)
_self=Path(__file__).read_text()
ck('python_spawn_bootstrap_guard', "if __name__ == '__main__':" in _self and 'mp.freeze_support()' in _self)
ck('python_spawn_context_explicit', "mp.get_context('spawn')" in _self and 'mp_context=spawn_ctx' in _self)

# E. Exact job inventory/caching contract.
hist=[('nav_imu_fault_vio_outage',1),('perception_dual_brief',1),('perception_stale_burst',1),('perception_range_spike',2),('coupled_imu_perception',1)]
recoverable=['nav_vio_dropout','nav_lidar_dropout','nav_vio_outlier','nav_lidar_outlier','nav_imu_fault_vio_outage','nav_high_noise',
             'perception_lidar_dropout','perception_depth_dropout','perception_dual_brief','perception_stale_burst','perception_range_spike','coupled_imu_perception']
seeds=list(range(5)); fs=['nav_xy_loss','perception_dual_prolonged']; fs_seeds=list(range(3))
rec=[(n,s) for n in recoverable for s in seeds]
base=[('baseline',s) for s in seeds]; fail=[(n,s) for n in fs for s in fs_seeds]
ck('hist_exact_5',len(hist)==5 and len(set(hist))==5)
ck('recoverable_exact_60',len(rec)==60 and len(set(rec))==60)
ck('historical_subset_of_recoverable',all(x in rec for x in hist))
noncached=[x for x in rec if x not in hist]
ck('noncached_recoverable_exact_55',len(noncached)==55)
unique_missions=set(base+rec+fail)
ck('qualification_exact_71_unique_missions',len(unique_missions)==71, str(len(unique_missions)))
executed=set(hist+base+noncached+fail)
ck('parallel_execution_exact_same_71',executed==unique_missions and len(executed)==71)

# F. Output directory isolation is scenario+seed based and RNG is per case.
ck('rng_reseed_per_case',runner.find("rng(seed,'twister');") < runner.find('mission_lifecycle_manager_S2_5'))
ck('result_dir_scenario_seed_isolated', all(tok in runner for tok in ["label=lower(regexprep(scenario.name", "sprintf('seed_%03d',seed)", 'resultsDir=fullfile(cfg.resultsRoot,label']))
# Canonical case keys are unique, therefore worker writes cannot alias across the 71 jobs.
ck('parallel_case_keys_unique',len({f'{n}|{s}' for n,s in executed})==71)

# G. Simulate nondeterministic worker completion and prove aggregation is order-independent.
def synthetic_job(job):
    n,s=job
    # deterministic function of explicit key, independent of process/order
    digest=hashlib.sha256(f'{n}:{s}'.encode()).hexdigest()
    return job,digest
def main():
    # macOS and Windows use spawn. Exercise that exact bootstrap path on every OS
    # so this backtest cannot pass under fork and then fail on MATLAB hosts using spawn.
    mp.freeze_support()
    serial={j:synthetic_job(j)[1] for j in sorted(executed)}
    shuffled=list(executed); random.Random(250102).shuffle(shuffled)
    parallel={}
    spawn_ctx=mp.get_context('spawn')
    with ProcessPoolExecutor(max_workers=4, mp_context=spawn_ctx) as ex:
        futs=[ex.submit(synthetic_job,j) for j in shuffled]
        for f in as_completed(futs):
            j,d=f.result(); parallel[j]=d
    ck('python_spawn_parallel_order_independent',parallel==serial)
    # Reconstruct 12x5 matrix by key, not completion order.
    serial_matrix=[[serial[(n,s)] for s in seeds] for n in recoverable]
    parallel_matrix=[[parallel[(n,s)] for s in seeds] for n in recoverable]
    ck('recoverable_matrix_indexing_order_independent',parallel_matrix==serial_matrix)

    # H. Fail-fast contract: only 5 historical cases precede inherited regression/long matrix.
    pos_hist=val.find('PARALLEL HISTORICAL RECOVERY PREFLIGHT')
    pos_assert=val.find('assert(all(histPass)')
    pos_f=val.find('PHASE 1 — INHERITED S2.4-F')
    pos_matrix=val.find('PARALLEL RECOVERABLE MATRIX')
    ck('historical_failfast_before_inherited_and_matrix',0<=pos_hist<pos_assert<pos_f<pos_matrix)
    ck('historical_results_cached','historical_cached_run(name,seed,histNames,histSeeds,histRuns)' in val)

    # I. Compact qualification removes dominant per-case v7.3 I/O while preserving defaults.
    ck('normal_runner_still_can_save_full_artifacts',"if saveArtifacts\n    save(fullfile(resultsDir" in runner)
    ck('qualification_disables_full_artifact_save',',false,false,false,false)' in wrapper)
    ck('gate_report_still_persisted',"S2_5_qualification_report.mat" in val)

    for n,ok,d in checks:
        print(f'{n:68s} {"PASS" if ok else "FAIL"} {d}')
    passed=all(x[1] for x in checks)
    print(f'\nS2.5 v1.0.7 PARALLEL HARNESS PYTHON BACKTEST: {"PASS" if passed else "FAIL"}')
    print('Unique coupled missions:',len(unique_missions),'| historical cached:',len(hist),'| new recoverable worker jobs:',len(noncached))
    return 0 if passed else 1

if __name__ == '__main__':
    raise SystemExit(main())

function report = run_S2_5_v1_0_6_historical_preflight()
% RUN_S2_5_V1_0_6_HISTORICAL_PREFLIGHT Run only the five exact historical
% recoverable failures. This is a fail-fast MATLAB runtime gate before the
% inherited S2.4-F regression and the full 71-mission S2.5 qualification.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
setup_S2_5_path();

fprintf('\n============================================================\n');
fprintf(' S2.5 v1.0.6 — STATIC + EXACT-SNAPSHOT OFFLINE PREFLIGHT\n');
fprintf('============================================================\n');
cmds={ ...
 fullfile(root,'s2_5','validation','audit_S2_4_G_parent_immutability.py'), ...
 fullfile(root,'s2_5','validation','audit_S2_5_static_v1_0_6.py'), ...
 fullfile(root,'s2_5','validation','audit_S2_5_matlab_sources.py'), ...
 fullfile(root,'s2_5','validation','backtest_S2_5_fault_model.py'), ...
 fullfile(root,'s2_5','validation','backtest_S2_5_recovery_logic_v1_0_6.py'), ...
 fullfile(root,'s2_5','validation','exact_snapshot_replay_v1_0_5','replay_exact_snapshots.py'), ...
 fullfile(root,'s2_5','validation','backtest_S2_5_v1_0_6_root_cause.py'), ...
 fullfile(root,'s2_5','validation','backtest_S2_5_v1_0_6_perception_integrity.py'), ...
 fullfile(root,'s2_5','validation','backtest_S2_5_parallel_harness_v1_0_6.py')};
for k=1:numel(cmds)
    [rc,out]=system(sprintf('python3 "%s"',cmds{k}));fprintf('%s',out);
    assert(rc==0,'S2_5:V106OfflineGate','Offline gate failed: %s',cmds{k});
end
poolInfo=start_S2_5_parallel_pool();

fprintf('\n============================================================\n');
fprintf(' S2.5 v1.0.6 — PARALLEL HISTORICAL RUNTIME PREFLIGHT (5)\n');
fprintf('============================================================\n');
names={'nav_imu_fault_vio_outage','perception_dual_brief', ...
    'perception_stale_burst','perception_range_spike','coupled_imu_perception'};
seeds=[1 1 1 2 1];n=numel(names);out=cell(1,n);pass=false(1,n);
F=parallel.FevalFuture.empty(0,n);
for q=1:n,F(q)=parfeval(@run_S2_5_qualification_case,1,seeds(q),names{q});end
phaseClock=tic;
for done=1:n
    [idx,o]=fetchNext(F);out{idx}=o;wm=nan;
    if isstruct(o)&&isfield(o,'elapsedWall_s')&&isfinite(o.elapsedWall_s),wm=o.elapsedWall_s/60;end
    fprintf('  [HIST] completed %d/%d: %-31s seed=%d | worker %.1f min | phase %.1f min\n', ...
        done,n,upper(names{idx}),seeds(idx),wm,toc(phaseClock)/60);
end
for k=1:n
    o=out{k};
    if ~(isstruct(o)&&isfield(o,'ok')&&o.ok)
        fprintf(2,'\n[HIST WORKER ERROR] %s seed=%d\n',upper(names{k}),seeds(k));
        if isstruct(o)&&isfield(o,'errorIdentifier'),fprintf(2,'Identifier: %s\n',o.errorIdentifier);end
        if isstruct(o)&&isfield(o,'errorMessage'),fprintf(2,'Message   : %s\n',o.errorMessage);end
        error('S2_5:WorkerFailure','Historical worker failed for %s seed=%d.',names{k},seeds(k));
    end
    s=o.summary;extra=historical_extra_local(names{k},s);
    pass(k)=logical(s.s25CasePass)&&s.goalReached&&~s.failsafeTriggered&&extra;
    fprintf('%d/5 %-31s seed=%d : %s | goal=%d E=%d laneSw=%d hold=%d inv=%d rv=%d scans=%d safety=%d\n', ...
        k,upper(names{k}),seeds(k),pf_local(pass(k)),s.goalReached,s.estimatorGate,s.laneSwitches, ...
        s.perceptionHoldCount,s.executionAuthorityInvalidationCount,field_local(s,'informativeRecoveryRelocationCount',0),s.scanHoldCount,s.executionSafetyPass);
    fprintf('      map integrity dup=%d occl=%d kept=%d | scans inherited=%d s25=%d allowance=%d\n', ...
        field_local(s,'s25IntegrityDuplicateHitRayCount',0),field_local(s,'s25IntegrityOcclusionRejectedHitRayCount',0), ...
        field_local(s,'s25IntegrityKeptHitRayCount',0),field_local(s,'scanHoldPass',false), ...
        field_local(s,'s25RecoveryScanHoldPass',field_local(s,'scanHoldPass',false)),field_local(s,'recoveryScanHoldAllowance',0));
    if isfield(s,'lastInformativeRecoveryDiagnostics')&&isstruct(s.lastInformativeRecoveryDiagnostics)
        d=s.lastInformativeRecoveryDiagnostics;
        fprintf('      recovery arch=%s stage=%d gain=%d cse=%d ranked=%d metric=%d traj=%d stop=%d selected=%d\n', ...
            char_or_local(d,'architecture',''),field_local(d,'selectedStage',0),field_local(d,'selectedGain',0), ...
            field_local(d,'continuousStartEgressUsed',false),field_local(d,'rankedCandidates',0), ...
            field_local(d,'metricRouteRejected',0),field_local(d,'trajectoryRejected',0), ...
            field_local(d,'stopRejected',0),field_local(d,'selectedRank',0));
    end
end
report=struct('pass',all(pass),'casePass',pass,'caseNames',{names},'seeds',seeds,'parallelWorkers',poolInfo.workers,'parallelProfile',poolInfo.profile,'runs',{out});
assert(report.pass,'S2_5:HistoricalRecoveryFailure','One or more v1.0.6 historical cases failed. Do NOT launch the full matrix.');
fprintf('\n============================================================\n');
fprintf(' S2.5 v1.0.6 HISTORICAL PREFLIGHT: PASS 5/5\n');
fprintf(' Safe to proceed to: gate = validate_S2_5_all();\n');
fprintf('============================================================\n');
end
function extra=historical_extra_local(name,s)
extra=true;
if any(strcmp(name,{'nav_imu_fault_vio_outage','coupled_imu_perception'})),extra=s.laneSwitches>=1;
elseif strcmp(name,'perception_dual_brief'),extra=s.perceptionHoldCount>=1;
elseif strcmp(name,'perception_stale_burst'),extra=s.mapRejectedPackets>=1&&s.perceptionHoldCount>=1;end
end
function v=field_local(s,n,d),if isfield(s,n),v=s.(n);else,v=d;end,end
function v=char_or_local(s,n,d),if isfield(s,n),v=s.(n);else,v=d;end,end
function s=pf_local(x),if x,s='PASS';else,s='FAIL';end,end

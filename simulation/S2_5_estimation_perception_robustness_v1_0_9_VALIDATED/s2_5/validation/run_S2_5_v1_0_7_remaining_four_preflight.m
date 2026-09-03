function report = run_S2_5_v1_0_7_remaining_four_preflight()
% Fail-fast runtime test for the four v1.0.6 matrix failures only.
root=fileparts(fileparts(fileparts(mfilename('fullpath')))); setup_S2_5_path();
cmds={fullfile(root,'s2_5','validation','audit_S2_4_G_parent_immutability.py'), ...
 fullfile(root,'s2_5','validation','audit_S2_5_static_v1_0_7.py'), ...
 fullfile(root,'s2_5','validation','audit_S2_5_matlab_sources.py'), ...
 fullfile(root,'s2_5','validation','backtest_S2_5_v1_0_7_lifecycle_fault_phasing.py'), ...
 fullfile(root,'s2_5','validation','backtest_S2_5_recovery_logic_v1_0_7.py'), ...
 fullfile(root,'s2_5','validation','backtest_S2_5_v1_0_6_perception_integrity.py'), ...
 fullfile(root,'s2_5','validation','backtest_S2_5_parallel_harness_v1_0_7.py')};
fprintf('\n============================================================\n S2.5 v1.0.7 — OFFLINE + REMAINING FOUR PREFLIGHT\n============================================================\n');
for k=1:numel(cmds),[rc,o]=system(sprintf('python3 "%s"',cmds{k}));fprintf('%s',o);assert(rc==0,'S2_5:V107OfflineGate','Offline gate failed: %s',cmds{k});end
poolInfo=start_S2_5_parallel_pool();
names={'nav_high_noise','perception_dual_brief','perception_stale_burst','coupled_imu_perception'}; seeds=[4 2 2 2]; n=4; out=cell(1,n); pass=false(1,n);
F=parallel.FevalFuture.empty(0,n);for q=1:n,F(q)=parfeval(@run_S2_5_qualification_case,1,seeds(q),names{q});end
clk=tic;for done=1:n,[idx,o]=fetchNext(F);out{idx}=o;fprintf('  completed %d/4: %-28s seed=%d | %.1f min\n',done,upper(names{idx}),seeds(idx),toc(clk)/60);end
for k=1:n
 o=out{k};if ~(isstruct(o)&&isfield(o,'ok')&&o.ok),if isstruct(o),disp(o);end,error('S2_5:WorkerFailure','Worker failed: %s seed=%d',names{k},seeds(k));end
 s=o.summary;pass(k)=logical(s.s25CasePass);
 fprintf('%d/4 %-28s seed=%d : %s | goal=%d fail=%d timeout=%d hold=%d laneSw=%d safety=%d brake=%d retry=%d\n',k,upper(names{k}),seeds(k),pf(pass(k)),s.goalReached,s.failsafeTriggered,s.stateTimeoutTriggered,s.perceptionHoldCount,s.laneSwitches,s.executionSafetyPass,s.replanBrakeCount,s.replanRetryCount);
end
report=struct('pass',all(pass),'casePass',pass,'caseNames',{names},'seeds',seeds,'parallelWorkers',poolInfo.workers,'parallelProfile',poolInfo.profile,'runs',{out});
assert(report.pass,'S2_5:V107RemainingFourFailure','One or more v1.0.7 remaining-four cases failed. Do NOT run full matrix.');
fprintf('\nS2.5 v1.0.7 REMAINING FOUR: PASS 4/4\nSafe to proceed to: gate = validate_S2_5_all();\n');
end
function s=pf(x),if x,s='PASS';else,s='FAIL';end,end

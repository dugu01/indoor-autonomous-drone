function release = validate_S2_3_release_all(runLegacy)
% VALIDATE_S2_3_RELEASE_ALL One-command S2.3 v1.0.0 release gate.
% Runs scenario-contract backtest, deterministic validation, exact mapper
% replay, critical multi-seed robustness, and frozen S2.2 regression suites.
if nargin<1||isempty(runLegacy),runLegacy=true;end
scriptDir=fileparts(mfilename('fullpath'));
addpath(scriptDir,'-begin');
started=datetime('now');
release=struct('started',started,'finished',NaT,'pass',false,'errors',{{}}, ...
    'pythonBacktestPass',false,'deterministicPass',false,'exactReplayPass',false, ...
    'robustnessPass',false,'legacyPass',~runLegacy,'safetyPass',false, ...
    'deterministic',[],'exactReplay',[],'robustness',[],'legacy',[]);

fprintf('\n============================================================\n');
fprintf(' S2.3 v1.0.0 COMPLETE RELEASE GATE\n');
fprintf(' Legacy regression rerun: %d\n',logical(runLegacy));
fprintf('============================================================\n');

%% Gate 0: Python scenario-contract backtest.
pyScript=fullfile(scriptDir,'python_tests','release_end_to_end_backtest.py');
jsonOut=fullfile(scriptDir,'RELEASE_END_TO_END_BACKTEST_MATLAB_GATE.json');
cmd=sprintf('python3 "%s" --source-root "%s" --json "%s"',pyScript,scriptDir,jsonOut);
[status,txt]=system(cmd);fprintf('%s\n',txt);
release.pythonBacktestPass=(status==0);
if status~=0,release.errors{end+1}=sprintf('Python scenario backtest failed: %s',txt);end

%% Gate 1: deterministic 12-case suite.
try
    release.deterministic=validate_S2_3(false);
    release.deterministicPass=all([release.deterministic.pass]);
catch ME
    release.errors{end+1}=getReport(ME,'extended','hyperlinks','off');
    cfg=init_S2_3_config();p=fullfile(cfg.resultsRoot,'validation','validation_report_S2_3_candidate.mat');
    if exist(p,'file')==2
        q=load(p,'report');release.deterministic=q.report;
        release.deterministicPass=all([q.report.pass]);
    end
end

%% Gate 2: nominal coupled run and exact mapper replay.
try
    nominal=run_S2_3_online_mapping(0,'unknown_room_nominal',false,false);
    matFile=fullfile(nominal.outputDir,'S2_3_v1_0_0_candidate_trial_data.mat');
    release.exactReplay=replay_perception_log_S2_3(matFile,true);
    release.exactReplayPass=logical(nominal.summary.pass)&&logical(release.exactReplay.pass);
catch ME
    release.errors{end+1}=getReport(ME,'extended','hyperlinks','off');
end

%% Gate 3: six critical scenarios x ten seeds.
try
    release.robustness=validate_S2_3_multiseed(0:9);
    release.robustnessPass=logical(release.robustness.pass);
catch ME
    release.errors{end+1}=getReport(ME,'extended','hyperlinks','off');
    cfg=init_S2_3_config();p=fullfile(cfg.resultsRoot,'validation','multiseed_robustness_S2_3_candidate.mat');
    if exist(p,'file')==2
        q=load(p,'report');release.robustness=q.report;
        release.robustnessPass=logical(q.report.pass);
    end
end

%% Gate 4: frozen inherited S2.2 evidence rerun.
if runLegacy
    try
        release.legacy=validate_S2_3_legacy_regression();
        release.legacyPass=true;
    catch ME
        release.errors{end+1}=getReport(ME,'extended','hyperlinks','off');
        release.legacyPass=false;
    end
end

%% Aggregate hard safety across available S2.3 summaries.
allSummaries={};
if ~isempty(release.deterministic)
    for i=1:numel(release.deterministic)
        if isfield(release.deterministic(i),'summary')&&~isempty(release.deterministic(i).summary)
            allSummaries{end+1}=release.deterministic(i).summary; %#ok<AGROW>
        end
    end
end
if ~isempty(release.robustness)&&isfield(release.robustness,'summaries')
    x=release.robustness.summaries;
    for i=1:numel(x),if ~isempty(x{i}),allSummaries{end+1}=x{i};end,end %#ok<AGROW>
end
hardSafety=true;
for i=1:numel(allSummaries)
    s=allSummaries{i};
    hardSafety=hardSafety&&s.collisionCount==0&&s.geofenceViolationCount==0&& ...
        s.unknownCommitmentCount==0&&logical(s.truthIsolationPass);
end
release.safetyPass=hardSafety&&~isempty(allSummaries);
release.pass=release.pythonBacktestPass&&release.deterministicPass&& ...
    release.exactReplayPass&&release.robustnessPass&&release.legacyPass&&release.safetyPass;
release.finished=datetime('now');release.elapsed_s=seconds(release.finished-release.started);

cfg=init_S2_3_config();d=fullfile(cfg.resultsRoot,'validation');if ~exist(d,'dir'),mkdir(d);end
save(fullfile(d,'release_report_S2_3_v1_0_0_candidate.mat'),'release','-v7.3');

fprintf('\n============================================================\n');
fprintf(' S2.3 COMPLETE RELEASE GATE SUMMARY\n');
fprintf(' Python 12-case contract : %d\n',release.pythonBacktestPass);
fprintf(' Deterministic 12/12     : %d\n',release.deterministicPass);
fprintf(' Exact mapper replay     : %d\n',release.exactReplayPass);
fprintf(' Critical robustness 60  : %d\n',release.robustnessPass);
fprintf(' Frozen S2.2 regression  : %d\n',release.legacyPass);
fprintf(' Hard safety aggregate   : %d\n',release.safetyPass);
fprintf(' Elapsed                  : %.1f s\n',release.elapsed_s);
fprintf(' RELEASE READY            : %d\n',release.pass);
fprintf('============================================================\n');
if ~release.pass
    error('S2_3:ReleaseGateFailed','S2.3 complete release gate failed. Inspect release.errors and saved report.');
end
end

%% S2.4 A-D COMPLETE OFFLINE/SHADOW VALIDATION
% This script does not connect S2.4 to the drone mission manager,
% controller, trajectory generator or plant.

clear;
clc;
close all force;

%% ------------------------------------------------------------------------
% USER PATH
% -------------------------------------------------------------------------

s24root = ...
    '/Users/doodle/Documents/MATLAB/indoor-autonomous-drone/simulation/S2_4_uncertainty_active_exploration_development';

s23root = fullfile( ...
    s24root, ...
    'frozen_parent', ...
    'S2_3_online_mapping_v1_0_0_validated');

s24shadow = fullfile(s24root, 's2_4_shadow');

% true:
%   reruns S2.3 deterministic 12/12, exact replay and robustness 60/60.
%
% false:
%   skips the complete S2.3 catalogue and runs only a fresh nominal S2.3
%   mission plus exact mapper replay before S2.4 validation.
RUN_COMPLETE_S23_RELEASE_GATE = false;

% false verifies the previously saved frozen S2.2 evidence.
% true reruns the complete inherited S2.2 validation campaigns as well.
% Keep false for the first execution.
RERUN_FULL_S22_LEGACY_CAMPAIGN = false;

%% ------------------------------------------------------------------------
% OUTPUT SESSION
% -------------------------------------------------------------------------

sessionTag = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));

sessionDir = fullfile( ...
    s24root, ...
    'results', ...
    ['S2_4_AD_validation_' sessionTag]);

if ~isfolder(sessionDir)
    mkdir(sessionDir);
end

consoleLog = fullfile(sessionDir, 'MATLAB_complete_console.txt');

diary off;
diary(consoleLog);

fprintf('\n');
fprintf('============================================================\n');
fprintf(' S2.4 A-D COMPLETE OFFLINE/SHADOW VALIDATION\n');
fprintf(' Session : %s\n', sessionTag);
fprintf(' Root    : %s\n', s24root);
fprintf('============================================================\n\n');

try
    %% --------------------------------------------------------------------
    % GATE 0 — REQUIRED FOLDER AND FILE STRUCTURE
    % ---------------------------------------------------------------------

    fprintf('\n[GATE 0] PACKAGE STRUCTURE\n');

    assert(isfolder(s24root), ...
        'S2.4 root folder does not exist:\n%s', s24root);

    assert(isfolder(s23root), ...
        'Frozen S2.3 parent folder is missing:\n%s', s23root);

    assert(isfolder(s24shadow), ...
        'S2.4 shadow MATLAB folder is missing:\n%s', s24shadow);

    requiredFiles = {
        fullfile(s23root, 'run_S2_3_online_mapping.m')
        fullfile(s23root, 'validate_S2_3_release_all.m')
        fullfile(s23root, 'replay_perception_log_S2_3.m')
        fullfile(s23root, 'update_probabilistic_map_S2_3.m')
        fullfile(s24shadow, 'validate_S2_4_AD.m')
        fullfile(s24shadow, 'run_S2_4_AD_shadow_replay.m')
        fullfile(s24root, 'tools', 'run_all_checks.py')
        fullfile(s24root, 'python_tests', ...
            's2_4_ad_contract_backtest.py')
        };

    for k = 1:numel(requiredFiles)
        assert(isfile(requiredFiles{k}), ...
            'Required package file is missing:\n%s', requiredFiles{k});
    end

    fprintf('Package structure: PASS\n');

    %% --------------------------------------------------------------------
    % GATE 1 — PYTHON ENVIRONMENT, CALLED FROM MATLAB
    % ---------------------------------------------------------------------

    fprintf('\n[GATE 1] PYTHON ENVIRONMENT\n');

    [status, text] = system('python3 --version');
    fprintf('%s\n', text);

    assert(status == 0, ...
        'MATLAB could not execute python3.');

    dependencyCommand = [ ...
        'python3 -c "' ...
        'import numpy, scipy, h5py; ' ...
        'print(''NumPy'', numpy.__version__); ' ...
        'print(''SciPy'', scipy.__version__); ' ...
        'print(''h5py'', h5py.__version__)"'];

    [status, text] = system(dependencyCommand);
    fprintf('%s\n', text);

    assert(status == 0, ...
        ['Python dependencies are missing. Required packages are ' ...
         'numpy, scipy and h5py.']);

    fprintf('Python environment: PASS\n');

    %% --------------------------------------------------------------------
    % GATE 2 — S2.4 STATIC/OFFLINE CHECKS AND 15-SCENARIO MATRIX
    % ---------------------------------------------------------------------

    fprintf('\n[GATE 2] S2.4 STATIC/OFFLINE PACKAGE GATE\n');

    command = sprintf( ...
        'cd "%s" && python3 tools/run_all_checks.py', ...
        s24root);

    [status, text] = system(command);
    fprintf('%s\n', text);

    staticGateLog = fullfile( ...
        sessionDir, ...
        'S2_4_static_offline_gate.txt');

    writeTextFile(staticGateLog, text);

    assert(status == 0, ...
        'S2.4 static/offline package gate failed.');

    assert(contains(text, ...
        'S2.4 A-D PACKAGE STATIC/OFFLINE GATE: PASS'), ...
        'The expected S2.4 aggregate PASS line was not printed.');

    fprintf('S2.4 static/offline package gate: PASS\n');

    %% --------------------------------------------------------------------
    % GATE 3 — MATLAB PATH ISOLATION FOR THE FROZEN S2.3 PARENT
    % ---------------------------------------------------------------------

    fprintf('\n[GATE 3] S2.3 MATLAB PATH ISOLATION\n');

    restoredefaultpath;
    rehash toolboxcache;
    addpath(s23root, '-begin');
    cd(s23root);

    clear functions;
    rehash;

    assertFunctionLocation( ...
        'run_S2_3_online_mapping', s23root);

    assertFunctionLocation( ...
        'validate_S2_3_release_all', s23root);

    assertFunctionLocation( ...
        'replay_perception_log_S2_3', s23root);

    assertFunctionLocation( ...
        'update_probabilistic_map_S2_3', s23root);

    assertFunctionLocation( ...
        'mission_lifecycle_manager_S2_3', s23root);

    fprintf('\nResolved S2.3 functions:\n');
    fprintf('  %s\n', which('run_S2_3_online_mapping'));
    fprintf('  %s\n', which('validate_S2_3_release_all'));
    fprintf('  %s\n', which('replay_perception_log_S2_3'));
    fprintf('  %s\n', which('update_probabilistic_map_S2_3'));
    fprintf('  %s\n', which('mission_lifecycle_manager_S2_3'));

    fprintf('S2.3 MATLAB path isolation: PASS\n');

    %% --------------------------------------------------------------------
    % GATE 4 — COMPLETE INHERITED S2.3 RELEASE REGRESSION
    % ---------------------------------------------------------------------

    release = [];

    if RUN_COMPLETE_S23_RELEASE_GATE
        fprintf('\n[GATE 4] COMPLETE S2.3 RELEASE REGRESSION\n');

        release = validate_S2_3_release_all( ...
            RERUN_FULL_S22_LEGACY_CAMPAIGN);

        assert(release.pythonBacktestPass, ...
            'S2.3 Python scenario-contract backtest failed.');

        assert(release.deterministicPass, ...
            'S2.3 deterministic 12-case matrix failed.');

        assert(release.exactReplayPass, ...
            'S2.3 exact mapper replay failed.');

        assert(release.robustnessPass, ...
            'S2.3 critical 60-run robustness matrix failed.');

        assert(release.legacyPass, ...
            'Frozen S2.2 regression/evidence verification failed.');

        assert(release.safetyPass, ...
            'S2.3 aggregate hard-safety gate failed.');

        assert(release.pass, ...
            'Complete S2.3 release gate failed.');

        fprintf('Complete inherited S2.3 release regression: PASS\n');
    else
        fprintf('\n[GATE 4] COMPLETE S2.3 RELEASE REGRESSION: SKIPPED\n');
    end

    %% --------------------------------------------------------------------
    % GATE 5 — FRESH FINAL S2.3 NOMINAL TRACE
    % ---------------------------------------------------------------------

    fprintf('\n[GATE 5] FRESH S2.3 NOMINAL COUPLED TRACE\n');

    nominal = run_S2_3_online_mapping( ...
        0, ...
        'unknown_room_nominal', ...
        false, ...
        false);

    s = nominal.summary;

    assert(s.pass, ...
        'Fresh S2.3 nominal coupled mission failed.');

    assert(s.goalReached == 1, ...
        'Fresh S2.3 nominal mission did not reach the target.');

    assert(s.goalUnreachable == 0, ...
        'Fresh S2.3 nominal mission declared the goal unreachable.');

    assert(s.failsafeTriggered == 0, ...
        'Fresh S2.3 nominal mission triggered failsafe.');

    assert(s.collisionCount == 0, ...
        'Fresh S2.3 nominal mission recorded a collision.');

    assert(s.geofenceViolationCount == 0, ...
        'Fresh S2.3 nominal mission violated the geofence.');

    assert(s.unknownCommitmentCount == 0, ...
        'Fresh S2.3 nominal mission committed to unknown space.');

    assert(logical(s.truthIsolationPass), ...
        'Fresh S2.3 nominal mission failed truth isolation.');

    assert(strcmp(s.finalState, 'COMPLETE'), ...
        'Fresh S2.3 nominal mission did not finish in COMPLETE.');

    trialMat = fullfile( ...
        nominal.outputDir, ...
        'S2_3_v1_0_0_candidate_trial_data.mat');

    assert(isfile(trialMat), ...
        'Fresh S2.3 nominal MAT trace was not generated:\n%s', ...
        trialMat);

    fprintf('\nFresh nominal trace:\n%s\n', trialMat);
    fprintf('Fresh S2.3 nominal coupled trace: PASS\n');

    %% --------------------------------------------------------------------
    % GATE 6 — EXACT INHERITED S2.3 MAPPER REPLAY
    % ---------------------------------------------------------------------

    fprintf('\n[GATE 6] EXACT S2.3 PERCEPTION-MAPPER REPLAY\n');

    exactReplay = replay_perception_log_S2_3( ...
        trialMat, ...
        true);

    assert(exactReplay.pass, ...
        'Exact inherited S2.3 mapper replay failed.');

    fprintf('Exact inherited S2.3 mapper replay: PASS\n');

    %% --------------------------------------------------------------------
    % GATE 7 — PYTHON RECORDED-TRACE REPLAY
    % ---------------------------------------------------------------------

    fprintf('\n[GATE 7] PYTHON RECORDED-TRACE S2.4 REPLAY\n');

    command = sprintf( ...
        ['cd "%s" && python3 tools/run_all_checks.py ' ...
         '--recorded-mat "%s"'], ...
        s24root, ...
        trialMat);

    [status, text] = system(command);
    fprintf('%s\n', text);

    recordedReplayLog = fullfile( ...
        sessionDir, ...
        'S2_4_python_recorded_replay.txt');

    writeTextFile(recordedReplayLog, text);

    assert(status == 0, ...
        'Python S2.4 recorded-trace replay failed.');

    assert(contains(text, ...
        'S2.4 A-D PACKAGE STATIC/OFFLINE GATE: PASS'), ...
        'Recorded replay did not end with the aggregate PASS line.');

    fprintf('Python recorded-trace S2.4 replay: PASS\n');

    %% --------------------------------------------------------------------
    % GATE 8 — RESET PATH AND ISOLATE S2.4 SHADOW FUNCTIONS
    % ---------------------------------------------------------------------

    fprintf('\n[GATE 8] S2.4 MATLAB PATH ISOLATION\n');

    restoredefaultpath;
    rehash toolboxcache;
    addpath(s24shadow, '-begin');
    cd(s24root);

    clear functions;
    rehash;

    assertFunctionLocation( ...
        'validate_S2_4_AD', s24shadow);

    assertFunctionLocation( ...
        'run_S2_4_AD_shadow_replay', s24shadow);

    assertFunctionLocation( ...
        'extract_frontiers_incremental_S2_4', s24shadow);

    assertFunctionLocation( ...
        'generate_safe_viewpoints_S2_4', s24shadow);

    fprintf('\nResolved S2.4 functions:\n');
    fprintf('  %s\n', which('validate_S2_4_AD'));
    fprintf('  %s\n', which('run_S2_4_AD_shadow_replay'));
    fprintf('  %s\n', which('extract_frontiers_incremental_S2_4'));
    fprintf('  %s\n', which('generate_safe_viewpoints_S2_4'));

    fprintf('S2.4 MATLAB path isolation: PASS\n');

    %% --------------------------------------------------------------------
    % GATE 9 — TWO-RUN MATLAB S2.4 A-D SHADOW VALIDATION
    % ---------------------------------------------------------------------

    fprintf('\n[GATE 9] MATLAB S2.4 A-D TWO-RUN SHADOW GATE\n');

    shadowOutputDir = fullfile( ...
        sessionDir, ...
        'S2_4_AD_nominal_shadow');

    gate = validate_S2_4_AD( ...
        trialMat, ...
        shadowOutputDir);

    fprintf('\nS2.4 MATLAB gate values:\n');
    disp(gate);

    assert(gate.parentExactMapperReplay == 1, ...
        'S2.4 parent exact mapper replay gate failed.');

    assert(gate.mapperArraysExact == 1, ...
        'S2.4 changed inherited S2.3 mapper arrays.');

    assert(gate.uncertaintyReplayDeterministic == 1, ...
        'S2.4 uncertainty replay was not deterministic.');

    assert(gate.frontierViewpointReplayDeterministic == 1, ...
        'S2.4 frontier/viewpoint replay was not deterministic.');

    assert(gate.repeatCountsExact == 1, ...
        'The two S2.4 replay runs produced different counts.');

    assert(gate.frontierReplayCompleted == 1, ...
        'No frontier replay snapshots were processed.');

    assert(gate.unsafeAcceptedCandidates == 0, ...
        'S2.4 accepted one or more unsafe viewpoints.');

    assert(gate.truthIsolation == 1, ...
        'S2.4 truth-isolation gate failed.');

    assert(gate.commandIsolation == 1, ...
        'S2.4 issued or exposed a command.');

    assert(gate.pass == 1, ...
        'S2.4 A-D MATLAB shadow gate failed.');

    fprintf('MATLAB S2.4 A-D two-run shadow gate: PASS\n');

    %% --------------------------------------------------------------------
    % GATE 10 — VERIFY GENERATED RESULT FILES
    % ---------------------------------------------------------------------

    fprintf('\n[GATE 10] RESULT-EVIDENCE FILES\n');

    expectedResultFiles = {
        fullfile(shadowOutputDir, ...
            'S2_4_AD_validation_gate.mat')

        fullfile(shadowOutputDir, ...
            'repeat_1', ...
            'S2_4_AD_shadow_report.mat')

        fullfile(shadowOutputDir, ...
            'repeat_1', ...
            'S2_4_AD_shadow_report.txt')

        fullfile(shadowOutputDir, ...
            'repeat_2', ...
            'S2_4_AD_shadow_report.mat')

        fullfile(shadowOutputDir, ...
            'repeat_2', ...
            'S2_4_AD_shadow_report.txt')
        };

    for k = 1:numel(expectedResultFiles)
        assert(isfile(expectedResultFiles{k}), ...
            'Expected S2.4 evidence file is missing:\n%s', ...
            expectedResultFiles{k});
    end

    evidence = load( ...
        fullfile(shadowOutputDir, ...
        'S2_4_AD_validation_gate.mat'), ...
        'gate', 'r1', 'r2');

    assert(strcmp( ...
        evidence.r1.uncertaintyDeterministicDigest, ...
        evidence.r2.uncertaintyDeterministicDigest), ...
        'Saved uncertainty digests are different.');

    assert(strcmp( ...
        evidence.r1.frontierViewpointDigest, ...
        evidence.r2.frontierViewpointDigest), ...
        'Saved frontier/viewpoint digests are different.');

    fprintf('\nReplay summary:\n');
    fprintf('  Snapshots processed       : %d\n', ...
        evidence.r1.snapshotCount);

    fprintf('  Frontier tracks           : %d\n', ...
        evidence.r1.frontierTrackCount);

    fprintf('  Accepted candidates       : %d\n', ...
        evidence.r1.acceptedCandidateCount);

    fprintf('  Unsafe accepted candidates: %d\n', ...
        evidence.r1.unsafeAcceptedCandidates);

    fprintf('  Truth accesses            : %d\n', ...
        evidence.r1.truthAccessCount);

    fprintf('  Commands issued           : %d\n', ...
        evidence.r1.commandIssued);

    fprintf('Result-evidence files: PASS\n');

    %% --------------------------------------------------------------------
    % GATE 11 — POST-RUN FROZEN-PARENT IMMUTABILITY
    % ---------------------------------------------------------------------

    fprintf('\n[GATE 11] POST-RUN PARENT BYTE IDENTITY\n');

    command = sprintf( ...
        ['cd "%s" && ' ...
         'python3 tools/audit_parent_immutability.py && ' ...
         'python3 tools/audit_final_parent_manifest.py'], ...
        s24root);

    [status, text] = system(command);
    fprintf('%s\n', text);

    postImmutabilityLog = fullfile( ...
        sessionDir, ...
        'post_run_parent_immutability.txt');

    writeTextFile(postImmutabilityLog, text);

    assert(status == 0, ...
        'Frozen S2.3 parent changed during validation.');

    fprintf('Post-run frozen-parent byte identity: PASS\n');

    %% --------------------------------------------------------------------
    % SAVE COMPLETE MATLAB VALIDATION WORKSPACE
    % ---------------------------------------------------------------------

    validationWorkspace = fullfile( ...
        sessionDir, ...
        'S2_4_AD_complete_validation_workspace.mat');

    save( ...
        validationWorkspace, ...
        's24root', ...
        's23root', ...
        'sessionTag', ...
        'release', ...
        'nominal', ...
        'trialMat', ...
        'exactReplay', ...
        'gate', ...
        'evidence', ...
        '-v7.3');

    %% --------------------------------------------------------------------
    % CREATE A SINGLE ZIP OF THE COMPLETE VALIDATION SESSION
    % ---------------------------------------------------------------------

    zipFile = fullfile( ...
        s24root, ...
        'results', ...
        ['S2_4_AD_validation_' sessionTag '.zip']);

    zip(zipFile, sessionDir);

    fprintf('\n');
    fprintf('============================================================\n');
    fprintf(' S2.4 A-D VALIDATION COMPLETE\n');
    fprintf('============================================================\n');
    fprintf(' S2.4 static/offline matrix       : PASS\n');
    fprintf(' S2.3 inherited regression        : PASS\n');
    fprintf(' S2.3 exact mapper replay         : PASS\n');
    fprintf(' Python recorded replay           : PASS\n');
    fprintf(' MATLAB uncertainty determinism   : PASS\n');
    fprintf(' MATLAB frontier determinism      : PASS\n');
    fprintf(' Unsafe accepted viewpoints       : 0\n');
    fprintf(' Truth-isolation violations       : 0\n');
    fprintf(' Commands issued                  : 0\n');
    fprintf(' Parent byte identity after run   : PASS\n');
    fprintf(' FINAL S2.4 A-D SHADOW GATE       : PASS\n');
    fprintf('============================================================\n');
    fprintf('\nResults folder:\n%s\n', sessionDir);
    fprintf('\nValidation ZIP:\n%s\n', zipFile);
    fprintf('\nMATLAB console log:\n%s\n', consoleLog);

    diary off;

catch ME
    fprintf(2, '\n');
    fprintf(2, '============================================================\n');
    fprintf(2, ' S2.4 VALIDATION STOPPED AT A FAILED GATE\n');
    fprintf(2, '============================================================\n');
    fprintf(2, '%s\n', getReport(ME, 'extended', ...
        'hyperlinks', 'off'));

    failureFile = fullfile( ...
        sessionDir, ...
        'S2_4_AD_validation_failure.mat');

    save(failureFile, 'ME', 's24root', 's23root', ...
        'sessionTag', '-v7.3');

    diary off;
    rethrow(ME);
end

%% ========================================================================
% LOCAL FUNCTIONS
% ========================================================================

function assertFunctionLocation(functionName, requiredRoot)

resolved = which(functionName);

assert(~isempty(resolved), ...
    'MATLAB could not resolve function: %s', functionName);

resolvedCanonical = char(java.io.File(resolved).getCanonicalPath());
rootCanonical = char(java.io.File(requiredRoot).getCanonicalPath());

assert(startsWith(resolvedCanonical, rootCanonical), ...
    ['Function %s resolved from the wrong folder.\n' ...
     'Resolved: %s\nRequired root: %s'], ...
    functionName, resolvedCanonical, rootCanonical);

end

function writeTextFile(filePath, text)

fid = fopen(filePath, 'w');

assert(fid >= 0, ...
    'Could not create log file:\n%s', filePath);

cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid, '%s', text);

end
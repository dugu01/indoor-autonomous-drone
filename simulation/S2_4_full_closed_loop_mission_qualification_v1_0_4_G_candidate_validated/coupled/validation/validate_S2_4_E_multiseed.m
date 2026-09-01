function report = validate_S2_4_E_multiseed(seeds,scenarioName)
% VALIDATE_S2_4_E_MULTISEED Repeated coupled runs in one MATLAB session.
%
%   report = validate_S2_4_E_multiseed(0:9,'active_goal_requires_scan');
%
% The function intentionally performs no path cleanup. Configure the project
% once with setup_S2_4_E_path before calling it.

if nargin < 1 || isempty(seeds)
    seeds = 0:9;
end
if nargin < 2 || isempty(scenarioName)
    scenarioName = 'active_goal_requires_scan';
end

seeds = seeds(:);
n = numel(seeds);
runs = cell(n,1);

Seed = seeds;
GoalReached = zeros(n,1);
Requests = zeros(n,1);
Selected = zeros(n,1);
Executed = zeros(n,1);
UnsafeExecution = zeros(n,1);
UnknownCommitments = zeros(n,1);
Collisions = zeros(n,1);
GeofenceViolations = zeros(n,1);
MissionPass = zeros(n,1);

for k = 1:n
    seed = seeds(k);
    fprintf('\n============================================\n');
    fprintf(' S2.4-E MULTISEED RUN %d/%d | SEED %d\n',k,n,seed);
    fprintf('============================================\n');

    runs{k} = run_S2_4_coupled(seed,scenarioName,false,false);
    s = runs{k}.summary;

    GoalReached(k) = double(s.goalReached);
    Requests(k) = double(s.explorationRequestCount);
    Selected(k) = double(s.explorationSelectedCount);
    Executed(k) = double(s.explorationExecutedCount);
    UnsafeExecution(k) = double(s.unsafeViewpointExecutionCount);
    UnknownCommitments(k) = double(s.unknownCommitmentCount);
    Collisions(k) = double(s.collisionCount);
    GeofenceViolations(k) = double(s.geofenceViolationCount);
    MissionPass(k) = double(s.pass);

    % Confirm that the next iteration can still resolve the runner.
    assert(~isempty(which('run_S2_4_coupled')), ...
        'S2_4:PathContextLost', ...
        'run_S2_4_coupled disappeared from the MATLAB path after seed %d.',seed);
end

summaryTable = table(Seed,GoalReached,Requests,Selected,Executed, ...
    UnsafeExecution,UnknownCommitments,Collisions, ...
    GeofenceViolations,MissionPass);

disp(summaryTable);

hardSafetyPass = all(UnsafeExecution == 0) && ...
    all(UnknownCommitments == 0) && ...
    all(Collisions == 0) && ...
    all(GeofenceViolations == 0);
missionCompletionPass = all(MissionPass == 1) && ...
    all(GoalReached == 1) && all(Executed >= 1);

report = struct();
report.scenario = scenarioName;
report.seeds = seeds;
report.runs = runs;
report.summaryTable = summaryTable;
report.hardSafetyPass = hardSafetyPass;
report.missionCompletionPass = missionCompletionPass;
report.pass = hardSafetyPass && missionCompletionPass;

fprintf('\nS2.4-E %s MULTISEED GATE: %d/%d PASS | %s\n', ...
    upper(scenarioName),sum(MissionPass),n, ...
    localTernary(report.pass,'PASS','FAIL'));

assert(report.pass,'S2_4:MultiseedGateFailed', ...
    'One or more multiseed mission or safety requirements failed.');
end

function out = localTernary(condition,a,b)
if condition
    out = a;
else
    out = b;
end
end

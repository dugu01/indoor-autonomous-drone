function report = validate_S2_2(makePlots)
% VALIDATE_S2_2  Run S2.2 v0.1 scenario matrix.
if nargin < 1, makePlots = false; end
scenarios = {'static_known_obstacles','unknown_obstacle_appears','goal_blocked_failsafe','narrow_passage_rejected'};
report = struct('scenario',{},'pass',{},'summary',{});
for i = 1:numel(scenarios)
    fprintf('\n[VALIDATE S2.2] %s\n',scenarios{i});
    r = run_S2_2_mission_replanning(0,scenarios{i},makePlots,false);
    report(i).scenario = scenarios{i}; %#ok<AGROW>
    report(i).pass = r.summary.pass;
    report(i).summary = r.summary;
    assert(r.summary.pass,'S2.2 scenario failed: %s',scenarios{i});
end
fprintf('\n[VALIDATE S2.2] All %d scenarios passed.\n',numel(scenarios));
end

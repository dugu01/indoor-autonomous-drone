function scenario = scenario_S2_2(name)
% SCENARIO_S2_2  Define Stage S2.2 v0.1 mission scenarios.
if nargin < 1 || isempty(name), name = 'unknown_obstacle_appears'; end
key = lower(strtrim(name));
known = [1.0 1.0 0.5 0.5; 4.0 3.5 0.5 0.5];

scenario = struct();
scenario.name = upper(key);
scenario.start = [3.0 0.8];
scenario.goal = [5.30 5.30];
scenario.knownObstacles = known;
scenario.unknownObstacles = zeros(0,4);
scenario.unknownAppearTime_s = inf;
scenario.expectedGoalReached = true;
scenario.expectedReplan = false;
scenario.expectedFailsafe = false;

switch key
    case {'static_known_obstacles','static'}
        scenario.name = 'STATIC_KNOWN_OBSTACLES';
    case {'unknown_obstacle_appears','unknown'}
        scenario.name = 'UNKNOWN_OBSTACLE_APPEARS';
        scenario.unknownObstacles = [3.0 2.2 1.2 0.4];
        scenario.unknownAppearTime_s = 2.0;
        scenario.expectedReplan = true;
    case {'goal_blocked_failsafe','goal_blocked'}
        scenario.name = 'GOAL_BLOCKED_FAILSAFE';
        scenario.unknownObstacles = [4.55 4.55 0.80 0.80];
        scenario.unknownAppearTime_s = 0.0;
        scenario.expectedGoalReached = false;
        scenario.expectedFailsafe = true;
    case {'narrow_passage_rejected','narrow'}
        scenario.name = 'NARROW_PASSAGE_REJECTED';
        scenario.start = [0.80 3.0];
        scenario.goal = [5.30 3.0];
        scenario.knownObstacles = [2.6 0.50 0.35 2.00; 2.6 3.55 0.35 1.95];
        scenario.expectedGoalReached = false;
        scenario.expectedFailsafe = true;
    otherwise
        error('S2_2:UnknownScenario','Unknown S2.2 scenario: %s',name);
end
end

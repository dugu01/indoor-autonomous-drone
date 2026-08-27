function scenario = scenario_S2_4(name)
% SCENARIO_S2_4 Coupled active-exploration scenarios.
if nargin<1||isempty(name),name='active_goal_requires_scan';end
key=lower(strtrim(name));
switch key
    case {'active_goal_requires_scan','goal_requires_scan','milestone_1'}
        scenario=scenario_S2_3('goal_requires_scan');
        scenario.name='ACTIVE_GOAL_REQUIRES_SCAN';
        scenario.expectedMinExplorationRequests=1;
        scenario.expectedMinExplorationExecutions=1;
        scenario.expectedMaxUnsafeViewpointExecutions=0;

    case {'active_competing_corridors','competing_corridors','milestone_2'}
        % Start from the validated S2.3 goal_requires_scan schema, then replace
        % only its truth-obstacle overlay with a literal two-branch corridor
        % fork. The frozen mapper/planner/controller and S2.4 selector are not
        % modified. Corridor labels live only in validationGeometry and are
        % forbidden inputs to the autonomy decision layer.
        scenario=scenario_S2_3('goal_requires_scan');
        scenario=make_literal_competing_corridors_S2_4(scenario);
        scenario.name='ACTIVE_COMPETING_CORRIDORS';
        % Allow the inherited initial hover/scan to establish both visible
        % branch-frontier families before the first outbound decision.
        scenario.goalCommandDelay_s=max(scenario.goalCommandDelay_s,2.20);
        scenario.expectedMinExplorationRequests=1;
        scenario.expectedMinExplorationExecutions=1;
        scenario.expectedMaxUnsafeViewpointExecutions=0;
        scenario.expectedTargetRelevantSelection=true;
        scenario.expectedMinCompetingFrontiers=2;
        scenario.expectedMinIrrelevantCandidates=1;
        scenario.expectedMinDistinctIrrelevantFrontiers=1;
        scenario.expectedMaxIrrelevantSelections=0;

    case {'direct_route_no_exploration','direct_route'}
        % A direct-known-free regression: exploration should remain dormant.
        scenario=scenario_S2_3('unknown_narrow_passage');
        scenario.name='DIRECT_ROUTE_NO_EXPLORATION';
        scenario.expectedMinExplorationRequests=0;
        scenario.expectedMinExplorationExecutions=0;
        scenario.expectedMaxUnsafeViewpointExecutions=0;

    otherwise
        % Permit inherited scenarios for development, but do not impose a
        % coupled-exploration execution requirement unless explicitly named.
        scenario=scenario_S2_3(name);
        scenario.expectedMinExplorationRequests=0;
        scenario.expectedMinExplorationExecutions=0;
        scenario.expectedMaxUnsafeViewpointExecutions=0;
end
scenario=addDecisionExpectationDefaults(scenario);
end

function s=addDecisionExpectationDefaults(s)
if ~isfield(s,'expectedTargetRelevantSelection')
    s.expectedTargetRelevantSelection=false;
end
if ~isfield(s,'expectedMinCompetingFrontiers')
    s.expectedMinCompetingFrontiers=0;
end
if ~isfield(s,'expectedMinIrrelevantCandidates')
    s.expectedMinIrrelevantCandidates=0;
end
if ~isfield(s,'expectedMinDistinctIrrelevantFrontiers')
    s.expectedMinDistinctIrrelevantFrontiers=0;
end
if ~isfield(s,'expectedMaxIrrelevantSelections')
    s.expectedMaxIrrelevantSelections=inf;
end
end

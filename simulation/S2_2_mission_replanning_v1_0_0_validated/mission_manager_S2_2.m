function [log,summary,maps] = mission_manager_S2_2(cfg,scenario)
% MISSION_MANAGER_S2_2 Stage S2.2 v0.5.3.3 mission dispatcher.
%
% Lifecycle scenarios use the autonomous lifecycle manager. Legacy v0.4
% scenarios use the v0.5.3.3 robust derivative, while the exact validated v0.4
% core remains in the package as an immutable regression reference.

if isfield(scenario,'lifecycleEnabled') && scenario.lifecycleEnabled
    [log,summary,maps]=mission_lifecycle_manager_S2_2(cfg,scenario);
else
    [log,summary,maps]=mission_manager_v0_5_3_core_S2_2(cfg,scenario);
end
end

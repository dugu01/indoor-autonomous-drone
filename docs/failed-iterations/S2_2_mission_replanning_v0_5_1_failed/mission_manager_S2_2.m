function [log,summary,maps] = mission_manager_S2_2(cfg,scenario)
% MISSION_MANAGER_S2_2 Stage S2.2 v0.5 mission dispatcher.
%
% Lifecycle scenarios use the new autonomous mission manager. All v0.4
% scenarios are routed through the exact MATLAB-validated v0.4 core so that
% the estimator, planner, dynamic-obstacle and 6-DOF regression baseline is
% preserved without reconstruction.

if isfield(scenario,'lifecycleEnabled') && scenario.lifecycleEnabled
    [log,summary,maps]=mission_lifecycle_manager_S2_2(cfg,scenario);
else
    [log,summary,maps]=mission_manager_v0_4_core_S2_2(cfg,scenario);
end
end

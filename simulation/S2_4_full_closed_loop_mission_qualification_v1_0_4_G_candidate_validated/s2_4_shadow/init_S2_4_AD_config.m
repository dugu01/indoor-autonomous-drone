function c = init_S2_4_AD_config(parentCfg)
% INIT_S2_4_AD_CONFIG Frozen configuration for the S2.4 A-D shadow stack.
% This configuration may rank shadow viewpoints, but it cannot change the
% authoritative S2.3 occupancy map, safety inflation, planner, or controller.
arguments
    parentCfg (1,1) struct
end
c = struct();
c.schema = 'S2_4_AD_CONFIG_V1';
c.shadowOnly = true;
c.commandOutputEnabled = false;
c.resolutionXY_m = parentCfg.mapResolutionXY_m;
c.resolutionZ_m = parentCfg.mapResolutionZ_m;
if ~isfield(parentCfg,'baseInflationRadius') || ~isfield(parentCfg,'altitudeNominal_m')
    error('S2_4:ParentConfig','Missing inherited S2.3 geometry fields.');
end
c.effectiveInflation_m = parentCfg.baseInflationRadius;
c.nominalAltitude_m = parentCfg.altitudeNominal_m;
c.mapFreeProbability = parentCfg.mapFreeProbability;
c.mapMinFreeObservations = parentCfg.mapMinFreeObservations;
% The final S2.3 trace records 0.602 m. The runtime value is verified again
% from maps.finalGrid.inflationRadius before any replay result is accepted.
c.staleFreeAge_s = 8.0;
c.minFrontierCells = 2;
c.maxFrontierExtentCells = 18;
c.targetCorridorRadiusCells = 3;
c.candidateRadii_m = [0.7 1.0 1.3];
c.candidateAngles = 16;
c.minVisibleUnknownCells = 1;
c.maxFrontierFailures = 2;
c.blacklistCooldownUpdates = 3;
c.diagnosticUnknownCost = 5.0;
c.dynamicHorizon_s = 3.0;
c.dynamicStep_s = 0.25;
c.dynamicChi2 = 7.814727903251179; % chi-square 95%, 3 DoF
c.weights = struct('information',0.25,'target',0.35,'travel',0.10, ...
    'staticRisk',0.08,'dynamicRisk',0.10,'uncertaintyRisk',0.05, ...
    'yaw',0.02,'stopRetreat',0.05);
c.source = struct('lidarQuality',1.0,'depthQuality',1.0, ...
    'staticSaturationCount',4.0,'dynamicSaturationCount',3.0, ...
    'staticAgeConstant_s',30.0);
c.rejectionCodes = { ...
    'OUTSIDE_MAP','OUTSIDE_GEOFENCE','POSITION_UNKNOWN','POSITION_OCCUPIED_OR_UNKNOWN_INFLATED', ...
    'INSUFFICIENT_STATIC_CLEARANCE','UNREACHABLE_KNOWN_FREE', ...
    'ROUTE_TOUCHES_UNKNOWN','ROUTE_TOUCHES_OCCUPIED', ...
    'STALE_ROUTE_REQUIRES_RESCAN','LIDAR_FOV_INVALID','DEPTH_FOV_INVALID', ...
    'FRONTIER_OCCLUDED','INSUFFICIENT_VISIBLE_UNKNOWN','YAW_LIMIT', ...
    'YAW_RATE_LIMIT','STOPPING_SUPPORT_INVALID','RETREAT_ROUTE_INVALID', ...
    'DYNAMIC_ROUTE_CROSSING','DYNAMIC_UNCERTAINTY_EXCESSIVE', ...
    'SENSOR_HEALTH_INSUFFICIENT','IRRELEVANT_EXPLORATION'};
end

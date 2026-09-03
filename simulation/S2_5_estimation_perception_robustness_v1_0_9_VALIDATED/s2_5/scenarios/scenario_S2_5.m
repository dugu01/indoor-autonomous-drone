function scenario = scenario_S2_5(name)
% SCENARIO_S2_5 Deterministic estimator/perception robustness cases.
% Faults are injected only into autonomy-visible synthetic sensor packets.
if nargin<1||isempty(name),name='baseline';end
key=lower(strtrim(name));
scenario=scenario_S2_4('active_goal_requires_scan');
scenario.name=upper(key);
scenario.s25NavigationFault=navFault('NONE',inf,-inf);
scenario.s25PerceptionFault=perFault('NONE',inf,-inf);
scenario.expectedS25MissionComplete=true;
scenario.expectedS25Failsafe=false;
scenario.expectedS25EmergencyLanding=false;
scenario.expectedS25LaneSwitch=false;
scenario.expectedS25PerceptionHold=false;
scenario.expectedS25MinMapRejectedPackets=0;
scenario.expectedS25MinNavFaultApplications=0;
scenario.expectedS25MinPerceptionFaultApplications=0;

switch key
    case {'baseline','s25_baseline'}
        scenario.name='S25_BASELINE';

    case {'nav_vio_dropout','n1'}
        scenario.name='S25_NAV_VIO_DROPOUT';
        scenario.s25NavigationFault=navFault('VIO_DROPOUT',18.0,24.0);
        scenario.expectedS25MinNavFaultApplications=1;

    case {'nav_lidar_dropout','n2'}
        scenario.name='S25_NAV_LIDAR_AID_DROPOUT';
        scenario.s25NavigationFault=navFault('LIDAR_AID_DROPOUT',18.0,24.0);
        scenario.expectedS25MinNavFaultApplications=1;

    case {'nav_vio_outlier','n3'}
        scenario.name='S25_NAV_VIO_OUTLIER_BURST';
        f=navFault('VIO_OUTLIER_BURST',18.0,18.60);
        f.vioPositionBias_m=[0.80;-0.60;0.25];
        f.vioVelocityBias_mps=[0.45;-0.35;0.20];
        f.vioAttitudeBias_rad=deg2rad([4.0;-3.0;15.0]);
        scenario.s25NavigationFault=f;
        scenario.expectedS25MinNavFaultApplications=1;

    case {'nav_lidar_outlier','n4'}
        scenario.name='S25_NAV_LIDAR_OUTLIER_BURST';
        f=navFault('LIDAR_OUTLIER_BURST',18.0,18.60);
        f.lidarXYBias_m=[0.90;-0.70];f.lidarYawBias_rad=deg2rad(22.0);
        scenario.s25NavigationFault=f;
        scenario.expectedS25MinNavFaultApplications=1;

    case {'nav_imu_fault_vio_outage','n5'}
        scenario.name='S25_NAV_PRIMARY_IMU_FAULT_VIO_OUTAGE';
        scenario.primaryImuBiasStepTime=18.0;
        scenario.primaryAccelBiasStep=[0.35 -0.25 0.18];
        scenario.primaryGyroBiasStep=deg2rad([1.5 -1.0 1.2]);
        scenario.s25NavigationFault=navFault('VIO_DROPOUT',18.0,28.0);
        scenario.expectedS25MinNavFaultApplications=1;
        scenario.expectedS25LaneSwitch=true;

    case {'nav_high_noise','n6'}
        scenario.name='S25_NAV_HIGH_MEASUREMENT_NOISE';
        scenario.measurementNoiseScale=1.5;

    case {'nav_xy_loss','n7'}
        % Persistent loss of both horizontal navigation aids during flight.
        % Keep the S2.4 unknown-room mission/map stack, but require the
        % inherited controlled emergency-landing response instead of goal
        % completion.
        scenario.name='S25_NAV_XY_AID_LOSS_FAILSAFE';
        scenario.vioOutageWindows=[18.0 inf];
        scenario.lidarOutageWindows=[18.0 inf];
        scenario.expectedGoalReached=false;
        scenario.expectedFailsafe=true;
        scenario.expectedRTLRequest=true;
        scenario.expectedRTLExecuted=false;
        scenario.expectedEmergencyLanding=true;
        scenario.expectedLanding=true;
        scenario.expectedDisarm=true;
        scenario.expectedMinExplorationRequests=0;
        scenario.expectedMinExplorationExecutions=0;
        scenario.expectedS25MissionComplete=true;
        scenario.expectedS25Failsafe=true;
        scenario.expectedS25EmergencyLanding=true;

    case {'perception_lidar_dropout','p1'}
        scenario.name='S25_PERCEPTION_LIDAR_DROPOUT';
        scenario.s25PerceptionFault=perFault('LIDAR_SCAN_DROPOUT',18.0,26.0);
        scenario.expectedS25MinPerceptionFaultApplications=1;

    case {'perception_depth_dropout','p2'}
        scenario.name='S25_PERCEPTION_DEPTH_DROPOUT';
        scenario.s25PerceptionFault=perFault('DEPTH_DROPOUT',18.0,26.0);
        scenario.expectedS25MinPerceptionFaultApplications=1;

    case {'perception_dual_brief','p3'}
        scenario.name='S25_PERCEPTION_BRIEF_DUAL_DROPOUT';
        f=perFault('DUAL_DROPOUT',18.0,19.20);f=stateTriggeredFault(f,1.20);
        scenario.s25PerceptionFault=f;
        scenario.expectedS25MinPerceptionFaultApplications=1;
        scenario.expectedS25PerceptionHold=true;

    case {'perception_stale_burst','p4'}
        scenario.name='S25_PERCEPTION_STALE_PACKET_BURST';
        f=perFault('STALE_PACKET_BURST',18.0,19.20);f.timestampLag_s=0.45;f=stateTriggeredFault(f,1.20);
        scenario.s25PerceptionFault=f;
        scenario.expectedS25MinPerceptionFaultApplications=1;
        scenario.expectedS25PerceptionHold=true;
        scenario.expectedS25MinMapRejectedPackets=1;

    case {'perception_range_spike','p5'}
        scenario.name='S25_PERCEPTION_RANGE_SPIKE';
        f=perFault('RANGE_SPIKE',18.00,18.04);f.rangeBias_m=0.80;
        scenario.s25PerceptionFault=f;
        scenario.expectedS25MinPerceptionFaultApplications=1;

    case {'perception_dual_prolonged','p6'}
        scenario.name='S25_PERCEPTION_PROLONGED_DUAL_DROPOUT';
        scenario.s25PerceptionFault=perFault('DUAL_DROPOUT',18.0,25.0);
        scenario.expectedS25MinPerceptionFaultApplications=1;
        scenario.expectedS25PerceptionHold=true;
        scenario.expectedS25Failsafe=true;
        scenario.expectedS25EmergencyLanding=true;
        scenario.expectedS25MissionComplete=true;
        scenario.expectedGoalReached=false;
        scenario.expectedFailsafe=true;
        scenario.expectedRTLRequest=true;
        scenario.expectedRTLExecuted=false;
        scenario.expectedEmergencyLanding=true;
        scenario.expectedLanding=true;
        scenario.expectedDisarm=true;
        scenario.expectedMinExplorationRequests=0;
        scenario.expectedMinExplorationExecutions=0;

    case {'coupled_imu_perception','c1'}
        scenario.name='S25_COUPLED_IMU_FAULT_PERCEPTION_DROPOUT';
        scenario.primaryImuBiasStepTime=18.0;
        scenario.primaryAccelBiasStep=[0.35 -0.25 0.18];
        scenario.primaryGyroBiasStep=deg2rad([1.5 -1.0 1.2]);
        f=perFault('DUAL_DROPOUT',18.0,19.20);f=stateTriggeredFault(f,1.20);
        scenario.s25PerceptionFault=f;
        scenario.expectedS25MinPerceptionFaultApplications=1;
        scenario.expectedS25LaneSwitch=true;
        scenario.expectedS25PerceptionHold=true;

    otherwise
        error('S2_5:UnknownScenario','Unknown S2.5 robustness scenario: %s',name);
end
end

function f=navFault(name,t0,t1)
f=struct('name',upper(name),'start_s',t0,'end_s',t1, ...
    'vioPositionBias_m',zeros(3,1),'vioVelocityBias_mps',zeros(3,1), ...
    'vioAttitudeBias_rad',zeros(3,1),'lidarXYBias_m',zeros(2,1), ...
    'lidarYawBias_rad',0,'rangeBias_m',0,'baroBias_m',0);
end

function f=stateTriggeredFault(f,duration_s)
% Arm the qualification fault only after the mission reaches a state in which
% perception freshness is safety-relevant. Once armed, the original fault
% duration runs continuously even if the fault itself drives MAP_DEGRADED_HOLD.
f.stateTriggered=true;
f.triggerAfter_s=f.start_s;
f.triggerDuration_s=duration_s;
f.triggerEligibleStates={'TRACK_OUTBOUND','TRACK_RTL','SCAN_HOLD'};
end
function f=perFault(name,t0,t1)
f=struct('name',upper(name),'start_s',t0,'end_s',t1, ...
    'timestampLag_s',0,'rangeBias_m',0);
end

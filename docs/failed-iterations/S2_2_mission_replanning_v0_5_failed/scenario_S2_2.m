function scenario = scenario_S2_2(name)
% SCENARIO_S2_2 Stage S2.2 v0.5 regression and mission-lifecycle cases.
if nargin<1||isempty(name),name='full_mission_nominal';end
key=lower(strtrim(name));
scenario=struct();
scenario.name=upper(key);
scenario.start=[3.0 0.8];
scenario.home=scenario.start;
scenario.goal=[5.30 5.30];
scenario.knownObstacles=[1.0 1.0 0.5 0.5;4.0 3.5 0.5 0.5];
scenario.insertedObstacles=struct('time',{},'rect',{});
scenario.dynamicObstacles=struct('start',{},'velocity',{},'radius',{},'appearTime',{},'disappearTime',{},'stopTime',{});
scenario.obstacleSensorDropoutWindows=zeros(0,2);
scenario.vioOutageWindows=zeros(0,2);
scenario.lidarOutageWindows=zeros(0,2);
scenario.rangeOutageWindows=zeros(0,2);
scenario.baroOutageWindows=zeros(0,2);
scenario.primaryImuBiasStepTime=inf;
scenario.primaryAccelBiasStep=[0 0 0];
scenario.primaryGyroBiasStep=[0 0 0];
scenario.backupImuBiasStepTime=inf;
scenario.backupAccelBiasStep=[0 0 0];
scenario.backupGyroBiasStep=[0 0 0];
scenario.measurementNoiseScale=1.0;
scenario.trajectoryInitialTimeScale=1.0;

% Legacy v0.4 expectations.
scenario.expectedGoalReached=true;
scenario.expectedFailsafe=false;
scenario.expectedReplan=false;
scenario.expectedDynamicAvoidance=false;
scenario.expectedPromotion=false;
scenario.expectedObstacleNoDataHold=false;
scenario.expectedLaneSwitch=false;
scenario.expectedRTLRequest=false;

% v0.5 lifecycle controls and expectations.
scenario.lifecycleEnabled=false;
scenario.forcePreflightReject=false;
scenario.goalCommandDelay_s=0.60;
scenario.requestRTLAfterGoal=true;
scenario.rtlObstacle=struct('enabled',false,'delay_s',inf,'rect',[0 0 0 0]);
scenario.blockHomeAtRTL=false;
scenario.homeBlockRect=[2.68 0.48 0.64 0.64];
scenario.alternateLandingZones=[5.1 0.9;0.9 5.1;0.9 3.0];
scenario.expectedArmed=true;
scenario.expectedTakeoff=true;
scenario.expectedRTLExecuted=true;
scenario.expectedLanding=true;
scenario.expectedDisarm=true;
scenario.expectedPreflightReject=false;
scenario.expectedEmergencyLanding=false;
scenario.expectedAlternateLanding=false;
scenario.expectedRTLMidcourseReplan=false;
scenario.expectedPreflightReason='none';

switch key
    % ------------------------------------------------------------------
    % Frozen v0.4 regression matrix.
    case {'nominal_6dof','nominal'}
        scenario.name='NOMINAL_6DOF';

    case {'incremental_static_estimated','incremental'}
        scenario.name='INCREMENTAL_STATIC_ESTIMATED';
        scenario.insertedObstacles(1)=struct('time',5.0,'rect',[3.8 2.2 0.1 0.1]);
        scenario.expectedReplan=true;

    case {'dynamic_crossing_6dof','crossing'}
        scenario.name='DYNAMIC_CROSSING_6DOF';
        scenario.dynamicObstacles(1)=dyn([5.50 2.50],[-0.25 0],0.22,10.0,22.0,inf);
        scenario.expectedDynamicAvoidance=true;

    case {'dynamic_blocker_becomes_static_6dof','blocker'}
        scenario.name='DYNAMIC_BLOCKER_BECOMES_STATIC_6DOF';
        scenario.dynamicObstacles(1)=dyn([5.25 2.00],[-0.20 0],0.15,5.0,inf,9.0);
        scenario.expectedDynamicAvoidance=true;
        scenario.expectedPromotion=true;
        scenario.expectedReplan=true;

    case {'obstacle_sensor_dropout_recover_6dof','obstacle_dropout'}
        scenario.name='OBSTACLE_SENSOR_DROPOUT_RECOVER_6DOF';
        scenario.obstacleSensorDropoutWindows=[8.0 9.2];
        scenario.expectedObstacleNoDataHold=true;

    case {'primary_imu_fault_vio_outage','imu_fault'}
        scenario.name='PRIMARY_IMU_FAULT_VIO_OUTAGE';
        scenario.primaryImuBiasStepTime=8.0;
        scenario.primaryAccelBiasStep=[0.35 -0.25 0.18];
        scenario.primaryGyroBiasStep=deg2rad([1.5 -1.0 1.2]);
        scenario.vioOutageWindows=[8.0 20.0];
        scenario.expectedLaneSwitch=true;

    case {'xy_aid_loss_failsafe','xy_loss'}
        scenario.name='XY_AID_LOSS_FAILSAFE';
        scenario.vioOutageWindows=[7.0 inf];
        scenario.lidarOutageWindows=[7.0 inf];
        scenario.expectedGoalReached=false;
        scenario.expectedFailsafe=true;
        scenario.expectedRTLRequest=true;

    % ------------------------------------------------------------------
    % New v0.5 full mission lifecycle scenarios.
    case {'full_mission_nominal','lifecycle_nominal'}
        scenario.name='FULL_MISSION_NOMINAL';
        scenario.lifecycleEnabled=true;

    case {'rtl_obstacle_replan','lifecycle_rtl_replan'}
        scenario.name='RTL_OBSTACLE_REPLAN';
        scenario.lifecycleEnabled=true;
        scenario.rtlObstacle=struct('enabled',true,'delay_s',2.5, ...
            'rect',[4.45 2.55 0.18 0.18]);
        scenario.expectedRTLMidcourseReplan=true;

    case {'alternate_landing_zone','lifecycle_alternate_land'}
        scenario.name='ALTERNATE_LANDING_ZONE';
        scenario.lifecycleEnabled=true;
        scenario.blockHomeAtRTL=true;
        scenario.expectedAlternateLanding=true;

    case {'preflight_reject_unsafe_home','lifecycle_preflight_reject'}
        scenario.name='PREFLIGHT_REJECT_UNSAFE_HOME';
        scenario.lifecycleEnabled=true;
        scenario.knownObstacles=[scenario.knownObstacles;scenario.homeBlockRect];
        scenario.forcePreflightReject=false;
        scenario.expectedArmed=false;
        scenario.expectedTakeoff=false;
        scenario.expectedRTLExecuted=false;
        scenario.expectedLanding=false;
        scenario.expectedDisarm=true;
        scenario.expectedPreflightReject=true;
        scenario.expectedPreflightReason='home';
        scenario.expectedGoalReached=false;
        scenario.expectedFailsafe=false;

    case {'xy_loss_emergency_land','lifecycle_xy_loss'}
        scenario.name='XY_LOSS_EMERGENCY_LAND';
        scenario.lifecycleEnabled=true;
        scenario.vioOutageWindows=[16.0 inf];
        scenario.lidarOutageWindows=[16.0 inf];
        scenario.expectedGoalReached=false;
        scenario.expectedFailsafe=true;
        scenario.expectedRTLRequest=true;
        scenario.expectedRTLExecuted=false;
        scenario.expectedEmergencyLanding=true;

    otherwise
        error('S2_2:UnknownScenario','Unknown S2.2 v0.5 scenario: %s',name);
end
end

function s=dyn(startXY,velocityXY,radius,appearTime,disappearTime,stopTime)
s=struct('start',startXY,'velocity',velocityXY,'radius',radius, ...
    'appearTime',appearTime,'disappearTime',disappearTime,'stopTime',stopTime);
end

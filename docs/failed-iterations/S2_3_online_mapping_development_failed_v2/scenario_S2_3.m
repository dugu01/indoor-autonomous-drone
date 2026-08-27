function scenario = scenario_S2_3(name)
% SCENARIO_S2_3 Unknown-environment S2.3 deterministic scenarios.
if nargin<1||isempty(name),name='unknown_room_nominal';end
key=lower(strtrim(name));

scenario=scenario_S2_2('full_mission_nominal');
scenario.lifecycleEnabled=true;
scenario.name=upper(key);
scenario.knownObstacles=zeros(0,4); % autonomy receives no prior obstacles
scenario.truthStaticObstacles=[ ... % x y width depth topZ
    1.00 1.00 0.50 0.50 1.80; ...
    4.00 3.50 0.50 0.50 1.80];
scenario.truthInsertedObstacles=struct('time',{},'rect5',{});
scenario.truthDynamicObstacles=scenario.dynamicObstacles;
scenario.dynamicObstacles=struct('start',{},'velocity',{},'radius',{},'appearTime',{},'disappearTime',{},'stopTime',{});
scenario.depthDropoutWindows=zeros(0,2);
scenario.perceptionLidarDropoutWindows=zeros(0,2);
scenario.expectedMapExtensions=false;
scenario.expectedMapReplan=false;
scenario.expectedGoalUnreachable=false;
scenario.expectedPerceptionHold=false;
scenario.expectedDynamicPromotion=false;
scenario.expectedTruthIsolation=true;
scenario.expectedGoalReached=true;
scenario.expectedFailsafe=false;
scenario.expectedRTLRequest=false;
scenario.requestRTLAfterGoal=true;
scenario.blockHomeAtRTL=false;
scenario.rtlObstacle=struct('enabled',false,'delay_s',inf,'rect',[0 0 0 0]);
scenario.truthHomeBlockAtRTL=false;
scenario.truthHomeBlockRect5=[2.68 0.48 0.64 0.64 1.80];

switch key
    case {'unknown_room_nominal','nominal'}
        scenario.name='UNKNOWN_ROOM_NOMINAL';

    case {'hidden_obstacle_replan','hidden'}
        scenario.name='HIDDEN_OBSTACLE_REPLAN';
        scenario.truthStaticObstacles=[scenario.truthStaticObstacles;3.25 2.10 0.35 1.15 1.90];
        scenario.expectedMapExtensions=true;
        scenario.expectedMapReplan=true;

    case {'occluded_obstacle','occlusion'}
        scenario.name='OCCLUDED_OBSTACLE';
        scenario.truthStaticObstacles=[1.80 1.65 0.60 0.55 1.90;2.55 2.30 0.45 0.80 1.90;4.20 4.25 0.55 0.45 1.90];
        scenario.expectedMapExtensions=true;

    case {'unknown_narrow_passage','narrow'}
        scenario.name='UNKNOWN_NARROW_PASSAGE';
        scenario.truthStaticObstacles=[0.95 2.45 1.75 0.45 1.90;3.35 2.45 1.70 0.45 1.90];
        scenario.goal=[3.0 5.25];
        scenario.expectedMapExtensions=true;

    case {'dead_end_recovery','dead_end'}
        scenario.name='DEAD_END_RECOVERY';
        scenario.truthStaticObstacles=[1.65 1.65 0.35 2.45 1.90;1.65 4.10 2.10 0.35 1.90;3.40 2.45 0.35 2.00 1.90];
        scenario.goal=[5.10 5.10];
        scenario.expectedMapExtensions=true;
        scenario.expectedMapReplan=true;

    case {'goal_requires_scan','scan'}
        scenario.name='GOAL_REQUIRES_SCAN';
        scenario.start=[0.85 0.85];scenario.home=scenario.start;scenario.goal=[5.15 5.15];
        scenario.truthStaticObstacles=[2.45 0.80 0.40 3.55 1.90;3.35 2.10 0.40 3.05 1.90];
        scenario.expectedMapExtensions=true;

    case {'unreachable_goal','unreachable'}
        scenario.name='UNREACHABLE_GOAL';
        scenario.goal=[4.90 4.90];
        scenario.truthStaticObstacles=[4.20 4.20 1.40 0.30 2.20;4.20 5.30 1.40 0.30 2.20;4.20 4.20 0.30 1.40 2.20;5.30 4.20 0.30 1.40 2.20];
        scenario.expectedGoalReached=false;
        scenario.expectedGoalUnreachable=true;
        scenario.expectedMapExtensions=true;
        scenario.expectedRTLExecuted=true;
        scenario.expectedLanding=true;

    case {'depth_dropout_lidar','depth_dropout'}
        scenario.name='DEPTH_DROPOUT_LIDAR_AVAILABLE';
        scenario.depthDropoutWindows=[8.0 24.0];

    case {'lidar_dropout_depth','lidar_dropout'}
        scenario.name='LIDAR_DROPOUT_DEPTH_AVAILABLE';
        scenario.perceptionLidarDropoutWindows=[8.0 24.0];
        scenario.expectedMapExtensions=true;

    case {'perception_dropout_recover','perception_dropout'}
        scenario.name='PERCEPTION_DROPOUT_RECOVER';
        scenario.depthDropoutWindows=[11.0 12.2];
        scenario.perceptionLidarDropoutWindows=[11.0 12.2];
        scenario.expectedPerceptionHold=true;

    case {'primary_imu_fault_mapping','imu_mapping'}
        scenario.name='PRIMARY_IMU_FAULT_DURING_MAPPING';
        scenario.primaryImuBiasStepTime=10.0;
        scenario.primaryAccelBiasStep=[0.35 -0.25 0.18];
        scenario.primaryGyroBiasStep=deg2rad([1.5 -1.0 1.2]);
        scenario.vioOutageWindows=[10.0 22.0];
        scenario.expectedLaneSwitch=true;
        scenario.expectedMapExtensions=true;

    case {'dynamic_to_static_mapping','dynamic_static'}
        scenario.name='DYNAMIC_TO_STATIC_MAPPING';
        scenario.truthDynamicObstacles(1)=dyn([5.20 2.15],[-0.18 0],0.17,7.0,inf,11.0);
        scenario.expectedDynamicPromotion=true;
        scenario.expectedMapReplan=true;

    case {'rtl_obstacle_online','rtl_online'}
        scenario.name='RTL_OBSTACLE_ONLINE';
        scenario.rtlObstacle=struct('enabled',true,'delay_s',2.2,'rect',[4.35 2.50 0.35 0.35]);
        scenario.expectedRTLMidcourseReplan=true;
        scenario.expectedMapReplan=true;

    case {'alternate_landing_online','alternate'}
        scenario.name='ALTERNATE_LANDING_ONLINE';
        scenario.truthHomeBlockAtRTL=true;
        scenario.expectedAlternateLanding=true;
        scenario.expectedMapReplan=true;

    otherwise
        error('S2_3:UnknownScenario','Unknown S2.3 scenario: %s',name);
end
end

function s=dyn(startXY,velocityXY,radius,appearTime,disappearTime,stopTime)
s=struct('start',startXY,'velocity',velocityXY,'radius',radius, ...
    'appearTime',appearTime,'disappearTime',disappearTime,'stopTime',stopTime);
end

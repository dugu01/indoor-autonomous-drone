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
scenario.expectedMinMapExtensions=0;
scenario.expectedMinMapSafetyReplans=0;
scenario.expectedMinScanHolds=0;
scenario.expectedMaxScanHolds=inf;
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

    case {'hidden_obstacle_replan','hidden','late_corridor_blockage'}
        scenario.name='LATE_CORRIDOR_BLOCKAGE_REPLAN';
        % A previously observed corridor is blocked well ahead of the UAV,
        % representing a closing door or furniture moved into the route.
        % The autonomy stack receives no obstacle coordinates: the new
        % blockage is available only through raw LiDAR/depth measurements.
        % Insertion at 12 s occurs during TRACK_OUTBOUND with more than 9 s
        % conservative time-to-route-intersection and over 3 m sensor range.
        % Its inflated footprint intersects the committed future route while
        % a safe alternate truth-map route remains available.
        scenario.truthInsertedObstacles(1)=struct( ...
            'time',12.0,'rect5',[5.19 3.31 0.30 0.30 1.90]);
        scenario.expectedMapExtensions=false;
        scenario.expectedMapReplan=true;
        scenario.expectedMinMapSafetyReplans=1;
        scenario.expectedMaxScanHolds=8;

    case {'occluded_obstacle','occlusion'}
        scenario.name='OCCLUDED_OBSTACLE';
        % Front and rear obstacles are approximately collinear from launch,
        % so the rear obstacle is initially occluded. The truth-map route
        % remains feasible at the frozen 0.602 m inflation radius.
        scenario.truthStaticObstacles=[3.15 1.65 0.50 0.65 1.90; ...
            3.40 2.75 0.55 0.75 1.90;4.40 4.10 0.40 0.50 1.90];
        scenario.expectedMapExtensions=true;
        scenario.expectedMaxScanHolds=8;

    case {'unknown_narrow_passage','narrow'}
        scenario.name='UNKNOWN_NARROW_PASSAGE';
        % Physical opening is 1.50 m. After applying the unchanged 0.602 m
        % radius on both sides, a narrow but non-zero centre corridor remains.
        scenario.truthStaticObstacles=[0.80 2.45 1.45 0.45 1.90; ...
            3.75 2.45 1.45 0.45 1.90];
        scenario.goal=[3.0 5.25];
        % The 360-degree 6.5 m LiDAR can observe this corridor from launch;
        % reaching the goal without a map-extension manoeuvre is valid.
        scenario.expectedMapExtensions=false;
        scenario.expectedMaxScanHolds=2;

    case {'dead_end_recovery','dead_end'}
        scenario.name='DEAD_END_RECOVERY';
        scenario.truthStaticObstacles=[1.65 1.65 0.35 2.45 1.90;1.65 4.10 2.10 0.35 1.90;3.40 2.45 0.35 2.00 1.90];
        scenario.goal=[5.10 5.10];
        scenario.expectedMapExtensions=true;
        % With 360-degree sensing, some noise seeds reveal the dead end before
        % route commitment. In that case initial avoidance is the correct
        % outcome; a repair is required only if a blocked route was committed.
        scenario.expectedMapReplan=false;
        scenario.expectedMaxScanHolds=8;

    case {'goal_requires_scan','scan'}
        scenario.name='GOAL_REQUIRES_SCAN';
        scenario.start=[0.85 0.85];scenario.home=scenario.start;scenario.goal=[5.15 5.15];
        % A tall screen blocks the direct route and hides the free passage
        % around its upper end. The passage is truth-map feasible but cannot
        % be committed until the vehicle advances and scans.
        scenario.truthStaticObstacles=[2.50 1.20 0.45 3.00 1.90];
        scenario.expectedMapExtensions=true;
        scenario.expectedMinScanHolds=1;
        scenario.expectedMaxScanHolds=8;

    case {'unreachable_goal','unreachable'}
        scenario.name='UNREACHABLE_GOAL';
        scenario.goal=[4.90 4.90];
        scenario.truthStaticObstacles=[4.20 4.20 1.40 0.30 2.20;4.20 5.30 1.40 0.30 2.20;4.20 4.20 0.30 1.40 2.20;5.30 4.20 0.30 1.40 2.20];
        scenario.expectedGoalReached=false;
        scenario.expectedGoalUnreachable=true;
        % Safe refusal is the required result. A forward extension is not
        % mandatory when the goal is correctly proven unreachable.
        scenario.expectedMapExtensions=false;
        scenario.expectedMinScanHolds=1;
        scenario.expectedMaxScanHolds=24;
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
        % This case validates temporary-to-persistent map promotion. Route
        % repair is conditional on whether the promoted footprint intersects
        % the seed-specific committed corridor; late_corridor_blockage is the
        % dedicated mandatory route-repair test.
        scenario.expectedMapReplan=false;
        scenario.expectedMaxScanHolds=8;

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

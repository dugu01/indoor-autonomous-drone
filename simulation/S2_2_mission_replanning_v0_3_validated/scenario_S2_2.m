function scenario = scenario_S2_2(name)
% SCENARIO_S2_2  Stage S2.2 v0.3 validation scenarios.
if nargin < 1 || isempty(name), name = 'dynamic_crossing_yield'; end
key = lower(strtrim(name));
scenario = struct();scenario.name=upper(key);scenario.start=[3.0 0.8];scenario.goal=[5.30 5.30];
scenario.knownObstacles=[1.0 1.0 0.5 0.5;4.0 3.5 0.5 0.5];
scenario.insertedObstacles=struct('time',{},'rect',{});
scenario.dynamicObstacles=struct('start',{},'velocity',{},'radius',{},'appearTime',{},'disappearTime',{},'stopTime',{});
scenario.sensorDropoutWindows=zeros(0,2);scenario.trajectoryInitialTimeScale=1.0;
scenario.expectedGoalReached=true;scenario.expectedFailsafe=false;scenario.expectedIncrementalReplan=false;
scenario.expectedDynamicAvoidance=false;scenario.expectedNoDataHold=false;scenario.expectedPromotion=false;
scenario.expectedTimeRescale=false;scenario.expectedTrajectoryContinuity=false;
switch key
    case {'incremental_static_insert','incremental'}
        scenario.name='INCREMENTAL_STATIC_INSERT';
        scenario.insertedObstacles(1)=struct('time',2.0,'rect',[3.8 2.2 0.1 0.1]);
        scenario.expectedIncrementalReplan=true;scenario.expectedTrajectoryContinuity=true;
    case {'dynamic_crossing_yield','crossing'}
        scenario.name='DYNAMIC_CROSSING_YIELD';
        scenario.dynamicObstacles(1)=dyn([5.6 2.50],[-0.45 0],0.22,4.0,12.0,inf);
        scenario.expectedDynamicAvoidance=true;
    case {'dynamic_blocker_becomes_static','blocker'}
        scenario.name='DYNAMIC_BLOCKER_BECOMES_STATIC';
        scenario.dynamicObstacles(1)=dyn([5.25 2.00],[-0.20 0],0.15,3.0,inf,7.0);
        scenario.expectedDynamicAvoidance=true;scenario.expectedIncrementalReplan=true;
        scenario.expectedPromotion=true;scenario.expectedTrajectoryContinuity=true;
    case {'sensor_dropout_recover','dropout_recover'}
        scenario.name='SENSOR_DROPOUT_RECOVER';scenario.sensorDropoutWindows=[5.0 6.2];scenario.expectedNoDataHold=true;
    case {'sensor_dropout_failsafe','dropout_failsafe'}
        scenario.name='SENSOR_DROPOUT_FAILSAFE';scenario.sensorDropoutWindows=[5.0 12.0];
        scenario.expectedGoalReached=false;scenario.expectedFailsafe=true;scenario.expectedNoDataHold=true;
    case {'two_dynamic_crossings','two_dynamic'}
        scenario.name='TWO_DYNAMIC_CROSSINGS';
        scenario.dynamicObstacles(1)=dyn([5.6 2.50],[-0.45 0],0.20,4.0,12.0,inf);
        scenario.dynamicObstacles(2)=dyn([5.45 5.0],[-0.10 -0.35],0.20,12.0,20.0,inf);
        scenario.expectedDynamicAvoidance=true;
    case {'trajectory_time_rescale','time_rescale'}
        scenario.name='TRAJECTORY_TIME_RESCALE';scenario.trajectoryInitialTimeScale=0.28;
        scenario.expectedTimeRescale=true;
    otherwise
        error('S2_2:UnknownScenario','Unknown S2.2 v0.3 scenario: %s',name);
end
end
function s=dyn(startXY,velocityXY,radius,appearTime,disappearTime,stopTime)
s=struct('start',startXY,'velocity',velocityXY,'radius',radius,'appearTime',appearTime, ...
    'disappearTime',disappearTime,'stopTime',stopTime);
end

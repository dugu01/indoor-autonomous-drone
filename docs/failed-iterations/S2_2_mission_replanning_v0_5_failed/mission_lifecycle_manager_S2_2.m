function [log,summary,maps] = mission_lifecycle_manager_S2_2(cfg,scenario)
% MISSION_LIFECYCLE_MANAGER_S2_2 Stage S2.2 v0.5 autonomous lifecycle.
%
% Flow:
% PREFLIGHT -> ARM -> TAKEOFF -> INITIAL_HOVER -> WAIT_FOR_GOAL
% -> PLAN_OUTBOUND -> TRACK_OUTBOUND -> GOAL_HOVER -> PLAN_RTL
% -> TRACK_RTL -> LAND_HOVER -> LAND_DESCENT -> DISARM -> COMPLETE.
%
% Loss of observable horizontal navigation while airborne triggers a short
% hold followed by a controlled local emergency landing. Planning and
% control use the selected local ESKF state. Truth is used only for
% simulation safety/performance validation.

cfg.initialPosition=[scenario.start(:);cfg.groundHeight_m];
activeObstacles=scenario.knownObstacles;
truth=init_quadrotor_state_S2_2(cfg,scenario.start,cfg.groundHeight_m);
sensorModel=init_sensor_model_S2_2();
[packet,sensorModel]=simulate_sensor_packet_S2_2(cfg,scenario,truth,sensorModel,0,1);
[nav,est]=multi_lane_eskf_lifecycle_S2_2('init',cfg,packet,0);

estAcc=zeros(3,1);previousTruthA=truth.a;previousCmdAccel=zeros(3,1);
currentInflation=uncertainty_inflation_S2_2(cfg,est.xySigma_m);
grid=build_occupancy_grid_S2_2(grid_cfg(cfg,currentInflation),activeObstacles);
preflight=preflight_check_S2_2(cfg,scenario,grid,packet,est);

state='PREFLIGHT';stateEntryTime=0;armed=false;armedEver=false;disarmed=true;
preflightRejected=false;takeoffCompleted=false;goalReached=false;rtlExecuted=false;
landed=false;missionComplete=false;failsafe=false;rtlRequested=false;
emergencyLanding=false;alternateLandingUsed=false;selectedLandingXY=scenario.home;
selectedLandingIndex=1;outboundTarget=scenario.goal;currentTarget=outboundTarget;
holdPosition=[scenario.start(:);cfg.groundHeight_m];
emergencyVerticalOnly=false;
verticalStartZ=cfg.groundHeight_m;verticalStartTime=0;
planner=[];path=zeros(0,2);traj=invalid_traj();trajClock=0;
pathHistory={};trajectoryHistory={};activeObstacleHistory={activeObstacles};activeObstacleHistoryTime=0;inflationHistory=currentInflation;
insertedDone=false(1,numel(scenario.insertedObstacles));homeBlockInserted=false;rtlObstacleInserted=false;rtlObstacleReplanRecorded=false;
rtlTrackStart=nan;rtlMidcourseReplanCount=0;replanCount=0;inflationReplanCount=0;
dstarRepairExpanded=0;astarScratchExpanded=0;astarRecoveryCount=0;
trajectoryGenerationCount=0;trajectoryFallbackCount=0;maxTrajectoryTimeScale=0;
maxReferenceContinuity=zeros(1,4);maxReplanStateJump=zeros(1,3);
maxReferenceSpeed=0;maxReferenceAccel=0;maxReferenceJerk=0;
maxVerticalReferenceSpeed=0;maxVerticalReferenceAccel=0;maxVerticalReferenceJerk=0;
pendingReplan=false;pendingResumeState='';pendingReplanStart=nan;lastReplanAttempt=-inf;
replanBrakeCount=0;replanRetryCount=0;

pathLength=0;minWall=inf;minObs=inf;collisionCount=0;geofenceViolationCount=0;
maxExecutedSpeed=0;maxExecutedAccel=0;maxExecutedJerk=0;maxExecutedVerticalSpeed=0;
maxExecutedVerticalAccel=0;maxExecutedVerticalJerk=0;
maxTilt=0;maxAltitudeError=0;maxEstimatorPositionError=0;maxEstimatorAttitudeError=0;
maxTrackingError=0;maxSafetyOverrideDeviation=0;maxInflation=currentInflation;
timeToGoal=nan;timeToLand=nan;timeToComplete=nan;
stateTransitionCount=0;stateTimeoutTriggered=false;
takeoffConfirmTimer=0;arrivalConfirmTimer=0;landContactTimer=0;landDetected=false;
truthGoalReached=false;truthTakeoffReached=false;truthLanded=false;

maxSteps=ceil(cfg.maxLifecycleTime_s/cfg.dt)+1;
T=nan(maxSteps,1);Ptrue=nan(maxSteps,3);Vtrue=nan(maxSteps,3);Atrue=nan(maxSteps,3);Jtrue=nan(maxSteps,3);
Qtrue=nan(maxSteps,4);RpyTrue=nan(maxSteps,3);Pest=nan(maxSteps,3);Vest=nan(maxSteps,3);Qest=nan(maxSteps,4);RpyEst=nan(maxSteps,3);
Pref=nan(maxSteps,3);Vref=nan(maxSteps,3);Aref=nan(maxSteps,3);trackingLog=nan(maxSteps,1);estErrorLog=nan(maxSteps,1);
attErrorLog=nan(maxSteps,1);stateId=nan(maxSteps,1);laneId=nan(maxSteps,1);laneScores=nan(maxSteps,4);laneEligible=false(maxSteps,4);
inflationLog=nan(maxSteps,1);xySigmaLog=nan(maxSteps,1);thrustLog=nan(maxSteps,1);momentLog=nan(maxSteps,3);
sensorAidLog=false(maxSteps,4);predMargin=nan(maxSteps,1);actualDyn=nan(maxSteps,1,2);estimatedDyn=nan(maxSteps,1,2);
armedLog=false(maxSteps,1);landingTargetLog=nan(maxSteps,2);

k=1;ref3=ground_ref(scenario.start,cfg);cmd=zero_cmd();stepMargin=inf;currentTruthJerk=zeros(3,1);
store_sample();

for k=2:maxSteps
    t=(k-1)*cfg.dt;
    previousState=state;mapChanged=false;changedMask=false(grid.ny,grid.nx);

    % Absolute-time static insertions retained for scenario flexibility.
    for i=1:numel(scenario.insertedObstacles)
        if ~insertedDone(i)&&t>=scenario.insertedObstacles(i).time
            oldOcc=grid.occ;activeObstacles=[activeObstacles;scenario.insertedObstacles(i).rect]; %#ok<AGROW>
            grid=build_occupancy_grid_S2_2(grid_cfg(cfg,currentInflation),activeObstacles);
            changedMask=changedMask|xor(oldOcc,grid.occ);insertedDone(i)=true;mapChanged=true;
            activeObstacleHistory{end+1}=activeObstacles;activeObstacleHistoryTime(end+1)=t; %#ok<AGROW>
        end
    end

    % Scenario-specific obstacle introduced after RTL has started.
    if scenario.rtlObstacle.enabled&&strcmp(state,'TRACK_RTL')&& ...
            ~rtlObstacleInserted&&isfinite(rtlTrackStart)&& ...
            t-rtlTrackStart>=scenario.rtlObstacle.delay_s
        oldOcc=grid.occ;activeObstacles=[activeObstacles;scenario.rtlObstacle.rect]; %#ok<AGROW>
        grid=build_occupancy_grid_S2_2(grid_cfg(cfg,currentInflation),activeObstacles);
        changedMask=changedMask|xor(oldOcc,grid.occ);rtlObstacleInserted=true;mapChanged=true;
        activeObstacleHistory{end+1}=activeObstacles;activeObstacleHistoryTime(end+1)=t; %#ok<AGROW>
    end

    proposedInflation=uncertainty_inflation_S2_2(cfg,est.xySigma_m);
    if proposedInflation>currentInflation+cfg.inflationReplanThreshold_m
        oldOcc=grid.occ;currentInflation=proposedInflation;
        grid=build_occupancy_grid_S2_2(grid_cfg(cfg,currentInflation),activeObstacles);
        changedMask=changedMask|xor(oldOcc,grid.occ);mapChanged=true;
        inflationReplanCount=inflationReplanCount+1;inflationHistory(end+1)=currentInflation; %#ok<AGROW>
    end
    maxInflation=max(maxInflation,currentInflation);

    % A map update during horizontal flight repairs the current segment.
    if mapChanged&&any(strcmp(state,{'TRACK_OUTBOUND','TRACK_RTL'}))
        resumeState=state;
        [planner,candidatePath,candidateTraj,st,jump,routeExists]= ...
            repair_segment(cfg,planner,grid,changedMask,est,estAcc,currentTarget);
        dstarRepairExpanded=dstarRepairExpanded+st.dstarExpanded;
        astarScratchExpanded=astarScratchExpanded+st.astarExpanded;
        astarRecoveryCount=astarRecoveryCount+st.astarRecovery;
        replanCount=replanCount+1;
        if strcmp(resumeState,'TRACK_RTL')&&rtlObstacleInserted&&~rtlObstacleReplanRecorded
            rtlMidcourseReplanCount=rtlMidcourseReplanCount+1;
            rtlObstacleReplanRecorded=true;
        end
        pathHistory{end+1}=candidatePath;trajectoryHistory{end+1}=candidateTraj; %#ok<AGROW>
        if ~isempty(candidatePath)&&candidateTraj.valid
            path=candidatePath;traj=candidateTraj;trajClock=0;
            [trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump,maxReferenceSpeed,maxReferenceAccel,maxReferenceJerk]= ...
                update_traj_metrics(traj,jump,trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump,maxReferenceSpeed,maxReferenceAccel,maxReferenceJerk);
        elseif routeExists
            pendingReplan=true;pendingResumeState=resumeState;pendingReplanStart=t;lastReplanAttempt=t;
            replanBrakeCount=replanBrakeCount+1;state='LIFECYCLE_REPLAN_BRAKE';holdPosition=est.p;
        else
            emergencyVerticalOnly=false;
            [state,stateEntryTime,emergencyLanding,failsafe,holdPosition,verticalStartZ,verticalStartTime]= ...
                begin_emergency_landing(t,est,'EMERGENCY_HOLD');
        end
    end

    if pendingReplan&&~failsafe&&norm(est.v(1:2))<=cfg.replanBrakeSpeed_mps&& ...
            norm(estAcc(1:2))<=cfg.replanBrakeAccel_mps2&& ...
            t-lastReplanAttempt>=cfg.replanRetryPeriod_s
        lastReplanAttempt=t;replanRetryCount=replanRetryCount+1;
        [planner,candidatePath,candidateTraj,st,jump,routeExists]= ...
            plan_segment(cfg,grid,est,estAcc,currentTarget,1.0);
        dstarRepairExpanded=dstarRepairExpanded+st.dstarExpanded;
        astarScratchExpanded=astarScratchExpanded+st.astarExpanded;
        astarRecoveryCount=astarRecoveryCount+st.astarRecovery;
        pathHistory{end+1}=candidatePath;trajectoryHistory{end+1}=candidateTraj; %#ok<AGROW>
        if ~isempty(candidatePath)&&candidateTraj.valid
            path=candidatePath;traj=candidateTraj;trajClock=0;pendingReplan=false;
            state=pendingResumeState;stateEntryTime=t;
            [trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump,maxReferenceSpeed,maxReferenceAccel,maxReferenceJerk]= ...
                update_traj_metrics(traj,jump,trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump,maxReferenceSpeed,maxReferenceAccel,maxReferenceJerk);
        elseif ~routeExists
            emergencyVerticalOnly=false;
            [state,stateEntryTime,emergencyLanding,failsafe,holdPosition,verticalStartZ,verticalStartTime]= ...
                begin_emergency_landing(t,est,'EMERGENCY_HOLD');
            pendingReplan=false;
        end
    end
    if pendingReplan&&t-pendingReplanStart>=cfg.rtlReplanTimeout_s
        emergencyVerticalOnly=false;
        [state,stateEntryTime,emergencyLanding,failsafe,holdPosition,verticalStartZ,verticalStartTime]= ...
            begin_emergency_landing(t,est,'EMERGENCY_HOLD');
        pendingReplan=false;
    end

    % Observable XY loss cannot support obstacle-aware return. Trigger the
    % emergency sequence once, then leave its state-entry timer untouched.
    % Horizontal position control is disabled during the descent so the
    % vehicle never commands a return to a stale XY estimate.
    if est.rtlRequested&&armed&&~any(strcmp(state,{ ...
            'EMERGENCY_HOLD','EMERGENCY_LAND','LAND_DESCENT', ...
            'DISARM','COMPLETE','PREFLIGHT_REJECT','FAILSAFE'}))
        rtlRequested=true;
        emergencyVerticalOnly=true;
        [state,stateEntryTime,emergencyLanding,failsafe,holdPosition,verticalStartZ,verticalStartTime]= ...
            begin_emergency_landing(t,est,'EMERGENCY_HOLD');
        pendingReplan=false;
    end

    ref3=ground_ref(scenario.start,cfg);motorsActive=false;trackingError=nan;

    switch state
        case 'PREFLIGHT'
            if t-stateEntryTime>=cfg.preflightCheckTime_s
                preflight=preflight_check_S2_2(cfg,scenario,grid,packet,est);
                if preflight.pass
                    state='ARM';stateEntryTime=t;disarmed=false;
                else
                    state='PREFLIGHT_REJECT';stateEntryTime=t;preflightRejected=true;
                end
            end

        case 'ARM'
            if t-stateEntryTime>=cfg.armDelay_s
                armed=true;armedEver=true;disarmed=false;
                state='TAKEOFF';stateEntryTime=t;verticalStartTime=t;verticalStartZ=est.p(3);takeoffConfirmTimer=0;landContactTimer=0;landDetected=false;
            end

        case 'TAKEOFF'
            motorsActive=true;
            vz=vertical_profile_S2_2(verticalStartZ,cfg.altitudeNominal_m,cfg.takeoffDuration_s,t-verticalStartTime);
            ref3=struct('p',[scenario.home(:);vz.z],'v',[0;0;vz.vz],'a',[0;0;vz.az],'yaw',0);
            maxVerticalReferenceSpeed=max(maxVerticalReferenceSpeed,abs(vz.vz));
            maxVerticalReferenceAccel=max(maxVerticalReferenceAccel,abs(vz.az));
            maxVerticalReferenceJerk=max(maxVerticalReferenceJerk,abs(vz.jz));
            takeoffReady=vz.complete&& ...
                abs(est.p(3)-cfg.altitudeNominal_m)<=cfg.takeoffAltitudeTolerance_m&& ...
                abs(est.v(3))<=cfg.arrivalSpeedTolerance_mps&& ...
                ~(isfield(packet,'groundContact')&&packet.groundContact);
            if takeoffReady
                takeoffConfirmTimer=takeoffConfirmTimer+cfg.dt;
            else
                takeoffConfirmTimer=0;
            end
            if takeoffConfirmTimer>=cfg.takeoffConfirmTime_s
                takeoffCompleted=true;state='INITIAL_HOVER';stateEntryTime=t;
                holdPosition=[scenario.home(:);cfg.altitudeNominal_m];
            end

        case 'INITIAL_HOVER'
            motorsActive=true;ref3=hover_ref(scenario.home,cfg.altitudeNominal_m);
            if t-stateEntryTime>=cfg.initialHoverTime_s,state='WAIT_FOR_GOAL';stateEntryTime=t;end

        case 'WAIT_FOR_GOAL'
            motorsActive=true;ref3=hover_ref(scenario.home,cfg.altitudeNominal_m);
            if t-stateEntryTime>=scenario.goalCommandDelay_s,state='PLAN_OUTBOUND';stateEntryTime=t;end

        case 'PLAN_OUTBOUND'
            motorsActive=true;currentTarget=outboundTarget;
            [planner,path,traj,st,jump,routeExists]=plan_segment(cfg,grid,est,estAcc,currentTarget,scenario.trajectoryInitialTimeScale);
            dstarRepairExpanded=dstarRepairExpanded+st.dstarExpanded;astarScratchExpanded=astarScratchExpanded+st.astarExpanded;astarRecoveryCount=astarRecoveryCount+st.astarRecovery;
            pathHistory{end+1}=path;trajectoryHistory{end+1}=traj; %#ok<AGROW>
            if routeExists&&traj.valid
                trajClock=0;state='TRACK_OUTBOUND';stateEntryTime=t;arrivalConfirmTimer=0;
                [trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump,maxReferenceSpeed,maxReferenceAccel,maxReferenceJerk]= ...
                    update_traj_metrics(traj,jump,trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump,maxReferenceSpeed,maxReferenceAccel,maxReferenceJerk);
            else
                emergencyVerticalOnly=false;
                [state,stateEntryTime,emergencyLanding,failsafe,holdPosition,verticalStartZ,verticalStartTime]= ...
                    begin_emergency_landing(t,est,'EMERGENCY_HOLD');
            end

        case {'TRACK_OUTBOUND','TRACK_RTL'}
            motorsActive=true;
            [desiredVel,trackingError,refXY]=track_smooth_trajectory_S2_2(cfg,est.p(1:2),est.v(1:2),traj,trajClock); %#ok<ASGLU>
            ref3=struct('p',[refXY.p(:);cfg.altitudeNominal_m],'v',[refXY.v(:);0], ...
                'a',[refXY.a(:);0],'yaw',0);
            if traj.valid&&isfinite(trackingError)&&trackingError<=cfg.trajectoryClockMaxError_m
                trajClock=min(traj.duration_s,trajClock+cfg.dt);
            end
            arrivedEstimate=norm(est.p(1:2).'-currentTarget)<=cfg.goalTolerance_m&& ...
                abs(est.p(3)-cfg.altitudeNominal_m)<=cfg.altitudeTolerance_m&& ...
                norm(est.v(1:2))<=cfg.arrivalSpeedTolerance_mps;
            if arrivedEstimate
                arrivalConfirmTimer=arrivalConfirmTimer+cfg.dt;
            else
                arrivalConfirmTimer=0;
            end
            if strcmp(state,'TRACK_OUTBOUND')
                requiredArrivalTime=cfg.goalArrivalConfirmTime_s;
            else
                requiredArrivalTime=cfg.rtlArrivalConfirmTime_s;
            end
            if arrivalConfirmTimer>=requiredArrivalTime
                arrivalConfirmTimer=0;
                if strcmp(state,'TRACK_OUTBOUND')
                    goalReached=true;timeToGoal=t;state='GOAL_HOVER';stateEntryTime=t;
                    holdPosition=[currentTarget(:);cfg.altitudeNominal_m];
                else
                    state='LAND_HOVER';stateEntryTime=t;
                    holdPosition=[selectedLandingXY(:);cfg.altitudeNominal_m];
                end
            end

        case 'GOAL_HOVER'
            motorsActive=true;ref3=hover_ref(scenario.goal,cfg.altitudeNominal_m);
            if t-stateEntryTime>=cfg.goalHoverTime_s&&scenario.requestRTLAfterGoal
                state='PLAN_RTL';stateEntryTime=t;
            end

        case 'PLAN_RTL'
            motorsActive=true;
            if scenario.blockHomeAtRTL&&~homeBlockInserted
                activeObstacles=[activeObstacles;scenario.homeBlockRect]; %#ok<AGROW>
                grid=build_occupancy_grid_S2_2(grid_cfg(cfg,currentInflation),activeObstacles);
                homeBlockInserted=true;activeObstacleHistory{end+1}=activeObstacles;activeObstacleHistoryTime(end+1)=t; %#ok<AGROW>
            end
            candidates=[scenario.home;scenario.alternateLandingZones];
            landingChoice=select_safe_landing_zone_S2_2(cfg,grid,est.p(1:2).',candidates);
            astarScratchExpanded=astarScratchExpanded+landingChoice.astarExpanded;
            if ~landingChoice.valid
                emergencyVerticalOnly=false;
                [state,stateEntryTime,emergencyLanding,failsafe,holdPosition,verticalStartZ,verticalStartTime]= ...
                    begin_emergency_landing(t,est,'EMERGENCY_HOLD');
            else
                selectedLandingXY=landingChoice.xy;selectedLandingIndex=landingChoice.candidateIndex;
                alternateLandingUsed=selectedLandingIndex>1;currentTarget=selectedLandingXY;
                [planner,path,traj,st,jump,routeExists]=plan_segment(cfg,grid,est,estAcc,currentTarget,1.0);
                dstarRepairExpanded=dstarRepairExpanded+st.dstarExpanded;astarScratchExpanded=astarScratchExpanded+st.astarExpanded;astarRecoveryCount=astarRecoveryCount+st.astarRecovery;
                pathHistory{end+1}=path;trajectoryHistory{end+1}=traj; %#ok<AGROW>
                if routeExists&&traj.valid
                    trajClock=0;rtlExecuted=true;rtlTrackStart=t;state='TRACK_RTL';stateEntryTime=t;arrivalConfirmTimer=0;
                    [trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump,maxReferenceSpeed,maxReferenceAccel,maxReferenceJerk]= ...
                        update_traj_metrics(traj,jump,trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump,maxReferenceSpeed,maxReferenceAccel,maxReferenceJerk);
                else
                    emergencyVerticalOnly=false;
                    [state,stateEntryTime,emergencyLanding,failsafe,holdPosition,verticalStartZ,verticalStartTime]= ...
                        begin_emergency_landing(t,est,'EMERGENCY_HOLD');
                end
            end

        case 'LAND_HOVER'
            motorsActive=true;ref3=hover_ref(selectedLandingXY,cfg.altitudeNominal_m);
            if t-stateEntryTime>=cfg.rtlArrivalHoverTime_s
                state='LAND_DESCENT';stateEntryTime=t;verticalStartTime=t;verticalStartZ=est.p(3);landContactTimer=0;landDetected=false;
            end

        case 'LAND_DESCENT'
            motorsActive=true;
            vz=vertical_profile_S2_2(verticalStartZ,cfg.groundHeight_m,cfg.landingDuration_s,t-verticalStartTime);
            ref3=struct('p',[selectedLandingXY(:);vz.z],'v',[0;0;vz.vz],'a',[0;0;vz.az],'yaw',0);
            maxVerticalReferenceSpeed=max(maxVerticalReferenceSpeed,abs(vz.vz));
            maxVerticalReferenceAccel=max(maxVerticalReferenceAccel,abs(vz.az));
            maxVerticalReferenceJerk=max(maxVerticalReferenceJerk,abs(vz.jz));
            if vz.complete&&landDetected
                landed=true;timeToLand=t;state='DISARM';stateEntryTime=t;
            end

        case 'EMERGENCY_HOLD'
            motorsActive=true;
            if emergencyVerticalOnly
                ref3=struct('p',[est.p(1:2);holdPosition(3)], ...
                    'v',zeros(3,1),'a',zeros(3,1),'yaw',0, ...
                    'horizontalControlEnabled',false);
            else
                ref3=struct('p',holdPosition(:),'v',zeros(3,1), ...
                    'a',zeros(3,1),'yaw',0,'horizontalControlEnabled',true);
            end
            if t-stateEntryTime>=cfg.emergencyHoldTime_s
                selectedLandingXY=est.p(1:2).';state='EMERGENCY_LAND';stateEntryTime=t;
                verticalStartTime=t;verticalStartZ=est.p(3);landContactTimer=0;landDetected=false;
            end

        case 'EMERGENCY_LAND'
            motorsActive=true;
            vz=vertical_profile_S2_2(verticalStartZ,cfg.groundHeight_m,cfg.landingDuration_s,t-verticalStartTime);
            if emergencyVerticalOnly
                ref3=struct('p',[est.p(1:2);vz.z],'v',[0;0;vz.vz], ...
                    'a',[0;0;vz.az],'yaw',0,'horizontalControlEnabled',false);
            else
                ref3=struct('p',[selectedLandingXY(:);vz.z],'v',[0;0;vz.vz], ...
                    'a',[0;0;vz.az],'yaw',0,'horizontalControlEnabled',true);
            end
            maxVerticalReferenceSpeed=max(maxVerticalReferenceSpeed,abs(vz.vz));
            maxVerticalReferenceAccel=max(maxVerticalReferenceAccel,abs(vz.az));
            maxVerticalReferenceJerk=max(maxVerticalReferenceJerk,abs(vz.jz));
            if vz.complete&&landDetected
                landed=true;timeToLand=t;state='DISARM';stateEntryTime=t;
            end

        case 'LIFECYCLE_REPLAN_BRAKE'
            motorsActive=true;
            ref3=struct('p',[est.p(1:2);cfg.altitudeNominal_m], ...
                'v',zeros(3,1),'a',zeros(3,1),'yaw',0);

        case 'DISARM'
            if t-stateEntryTime>=cfg.disarmDelay_s
                armed=false;disarmed=true;state='COMPLETE';stateEntryTime=t;
                missionComplete=true;timeToComplete=t;
            end

        case {'PREFLIGHT_REJECT','COMPLETE','FAILSAFE'}
            % Terminal states use zero thrust below.

        otherwise
            state='FAILSAFE';stateEntryTime=t;failsafe=true;
    end

    if ~strcmp(previousState,state),stateTransitionCount=stateTransitionCount+1;end
    if t-stateEntryTime>cfg.missionStateTimeout_s&& ...
            ~any(strcmp(state,{'TRACK_OUTBOUND','TRACK_RTL','PREFLIGHT_REJECT','COMPLETE'}))
        stateTimeoutTriggered=true;
        emergencyVerticalOnly=logical(est.degraded||est.rtlRequested);
        [state,stateEntryTime,emergencyLanding,failsafe,holdPosition,verticalStartZ,verticalStartTime]= ...
            begin_emergency_landing(t,est,'EMERGENCY_HOLD');
    end

    % Terminal/disarm states cut motor command immediately, including the
    % same integration step in which landing is confirmed.
    motorsActive=motorsActive&&~any(strcmp(state,{ ...
        'DISARM','COMPLETE','PREFLIGHT_REJECT','FAILSAFE'}));
    if motorsActive&&armed
        cmd=geometric_controller_S2_2(cfg,est,ref3,previousCmdAccel);
        previousCmdAccel=cmd.aCmd;
    else
        cmd=zero_cmd();previousCmdAccel=zeros(3,1);
    end

    oldTruthP=truth.p;truth=quadrotor_dynamics_S2_2(cfg,truth,cmd);
    pathLength=pathLength+norm(truth.p-oldTruthP);
    [packet,sensorModel]=simulate_sensor_packet_S2_2(cfg,scenario,truth,sensorModel,t,k);
    oldEstV=est.v;[nav,est]=multi_lane_eskf_lifecycle_S2_2('step',nav,cfg,packet,t,cfg.dt);
    rawEstAcc=(est.v-oldEstV)/cfg.dt;
    estAcc=(1-cfg.estAccelerationFilterAlpha)*estAcc+cfg.estAccelerationFilterAlpha*rawEstAcc;
    hAcc=norm(estAcc(1:2));if hAcc>cfg.maxReplanStartAccel_mps2,estAcc(1:2)=estAcc(1:2)*(cfg.maxReplanStartAccel_mps2/hAcc);end
    [landDetected,landContactTimer]=land_detector_S2_2( ...
        cfg,packet,est,landContactTimer);

    currentTruthJerk=(truth.a-previousTruthA)/cfg.dt;previousTruthA=truth.a;
    truthTakeoffReached=truthTakeoffReached|| ...
        (~truth.onGround&&truth.p(3)>=cfg.altitudeNominal_m-cfg.takeoffAltitudeTolerance_m);
    truthGoalReached=truthGoalReached|| ...
        (norm(truth.p(1:2).'-scenario.goal)<=1.5*cfg.goalTolerance_m&& ...
        abs(truth.p(3)-cfg.altitudeNominal_m)<=cfg.altitudeTolerance_m);
    truthLanded=truthLanded||(armedEver&&takeoffCompleted&&truth.onGround);
    truthTrackErr=norm(truth.p-ref3.p);
    if any(strcmp(state,{'TRACK_OUTBOUND','TRACK_RTL','TAKEOFF','LAND_DESCENT'}))
        maxTrackingError=max(maxTrackingError,truthTrackErr);
    else
        maxSafetyOverrideDeviation=max(maxSafetyOverrideDeviation,truthTrackErr);
    end
    maxExecutedSpeed=max(maxExecutedSpeed,norm(truth.v(1:2)));
    maxExecutedVerticalSpeed=max(maxExecutedVerticalSpeed,abs(truth.v(3)));
    maxExecutedAccel=max(maxExecutedAccel,norm(truth.a(1:2)));
    maxExecutedVerticalAccel=max(maxExecutedVerticalAccel,abs(truth.a(3)));
    maxExecutedJerk=max(maxExecutedJerk,norm(currentTruthJerk(1:2)));
    maxExecutedVerticalJerk=max(maxExecutedVerticalJerk,abs(currentTruthJerk(3)));
    rpyTruth=q2rpy_S2_2(truth.q);maxTilt=max(maxTilt,norm(rpyTruth(1:2)));
    maxAltitudeError=max(maxAltitudeError,abs(truth.p(3)-ref3.p(3)));
    posEstErr=norm(est.p-truth.p);attEstErr=norm(qlog_S2_2(qmul_S2_2(qconj_S2_2(est.q),truth.q)));
    maxEstimatorPositionError=max(maxEstimatorPositionError,posEstErr);maxEstimatorAttitudeError=max(maxEstimatorAttitudeError,attEstErr);
    wallRaw=min([truth.p(1),cfg.room(1)-truth.p(1),truth.p(2),cfg.room(2)-truth.p(2)]);
    minWall=min(minWall,wallRaw);if truth.p(3)>cfg.groundHeight_m+0.05&&wallRaw<cfg.collisionRadius,geofenceViolationCount=geofenceViolationCount+1;end
    obsClear=inf;for j=1:size(activeObstacles,1),dObs=dist_point_rect_S2_2(truth.p(1:2).',activeObstacles(j,:));obsClear=min(obsClear,dObs);if truth.p(3)>cfg.groundHeight_m+0.05&&dObs<cfg.collisionRadius,collisionCount=collisionCount+1;end,end;minObs=min(minObs,obsClear);

    store_sample();
    if any(strcmp(state,{'PREFLIGHT_REJECT','COMPLETE','FAILSAFE'})),break;end
end

valid=isfinite(T);T=T(valid);Ptrue=Ptrue(valid,:);Vtrue=Vtrue(valid,:);Atrue=Atrue(valid,:);Jtrue=Jtrue(valid,:);Qtrue=Qtrue(valid,:);RpyTrue=RpyTrue(valid,:);
Pest=Pest(valid,:);Vest=Vest(valid,:);Qest=Qest(valid,:);RpyEst=RpyEst(valid,:);Pref=Pref(valid,:);Vref=Vref(valid,:);Aref=Aref(valid,:);
trackingLog=trackingLog(valid);estErrorLog=estErrorLog(valid);attErrorLog=attErrorLog(valid);stateId=stateId(valid);laneId=laneId(valid);laneScores=laneScores(valid,:);laneEligible=laneEligible(valid,:);
inflationLog=inflationLog(valid);xySigmaLog=xySigmaLog(valid);thrustLog=thrustLog(valid);momentLog=momentLog(valid,:);sensorAidLog=sensorAidLog(valid,:);predMargin=predMargin(valid);
actualDyn=actualDyn(valid,:,:);estimatedDyn=estimatedDyn(valid,:,:);armedLog=armedLog(valid);landingTargetLog=landingTargetLog(valid,:);

staticPass=(isempty(activeObstacles)||minObs>=cfg.baseInflationRadius-0.03)&&minWall>=cfg.baseInflationRadius-0.03;
referenceKinematicPass=maxReferenceSpeed<=cfg.maxSpeedXY_mps+1e-6&&maxReferenceAccel<=cfg.maxAccelXY_mps2+1e-6&&maxReferenceJerk<=cfg.maxJerkXY_mps3+1e-6&& ...
    maxVerticalReferenceSpeed<=cfg.maxVerticalReferenceSpeed_mps+1e-6&&maxVerticalReferenceAccel<=cfg.maxVerticalReferenceAccel_mps2+1e-6&&maxVerticalReferenceJerk<=cfg.maxVerticalReferenceJerk_mps3+1e-6;
executedKinematicPass=maxExecutedSpeed<=cfg.maxExecutedSpeed_mps&& ...
    maxExecutedAccel<=cfg.maxExecutedAccel_mps2&&maxExecutedJerk<=cfg.maxExecutedJerk_mps3&& ...
    maxExecutedVerticalSpeed<=cfg.maxExecutedVerticalSpeed_mps&& ...
    maxExecutedVerticalAccel<=cfg.maxExecutedVerticalAccel_mps2&& ...
    maxExecutedVerticalJerk<=cfg.maxExecutedVerticalJerk_mps3;
controllerPass=maxTrackingError<=cfg.maxPositionTrackingError_m&&maxAltitudeError<=cfg.maxAltitudeError_m&&rad2deg(maxTilt)<=cfg.maxAttitudeError_deg;
if scenario.expectedFailsafe,estimatorPositionPass=maxEstimatorPositionError<=cfg.maxEstimatorFailsafeBound_m;else,estimatorPositionPass=maxEstimatorPositionError<=cfg.maxEstimatorPositionError_m;end
estimatorAttitudePass=rad2deg(maxEstimatorAttitudeError)<=cfg.maxEstimatorAttitudeError_deg;
continuityPass=all(maxReferenceContinuity<=cfg.maxReferenceContinuityJump)&&all(maxReplanStateJump<=cfg.maxReplanStateJump);
uncertaintyPass=maxInflation>=cfg.baseInflationRadius&&maxInflation<=cfg.maxInflationRadius+1e-9;

armedPass=(armedEver==scenario.expectedArmed);takeoffPass=(takeoffCompleted==scenario.expectedTakeoff);
rtlExecutionPass=(rtlExecuted==scenario.expectedRTLExecuted);landingPass=(landed==scenario.expectedLanding)||scenario.expectedPreflightReject;
disarmPass=(disarmed==scenario.expectedDisarm);preflightPass=(preflightRejected==scenario.expectedPreflightReject);
emergencyPass=(emergencyLanding==scenario.expectedEmergencyLanding);alternatePass=(alternateLandingUsed==scenario.expectedAlternateLanding);
rtlReplanPass=~scenario.expectedRTLMidcourseReplan||rtlMidcourseReplanCount>=1;
goalPass=(goalReached==scenario.expectedGoalReached);failsafePass=(failsafe==scenario.expectedFailsafe);
rtlRequestPass=~scenario.expectedRTLRequest||rtlRequested;
truthGoalPass=~scenario.expectedGoalReached||truthGoalReached;
truthTakeoffPass=~scenario.expectedTakeoff||truthTakeoffReached;
truthLandingPass=~scenario.expectedLanding||truthLanded;
completionPass=scenario.expectedPreflightReject||missionComplete;
preflightReasonPass=true;
if scenario.expectedPreflightReject&&isfield(scenario,'expectedPreflightReason')
    switch lower(scenario.expectedPreflightReason)
        case 'home'
            preflightReasonPass=~preflight.homeClear;
        case 'horizontal_aid'
            preflightReasonPass=~preflight.horizontalAidsOK;
        case 'vertical_aid'
            preflightReasonPass=~preflight.verticalAidOK;
        otherwise
            preflightReasonPass=~preflight.pass;
    end
end
missionLifecyclePass=armedPass&&takeoffPass&&rtlExecutionPass&&landingPass&& ...
    disarmPass&&preflightPass&&preflightReasonPass&&emergencyPass&&alternatePass&& ...
    rtlReplanPass&&goalPass&&failsafePass&&rtlRequestPass&&completionPass&& ...
    truthGoalPass&&truthTakeoffPass&&truthLandingPass;
if scenario.expectedPreflightReject
    trajectoryGate=true;controllerGate=true;estimatorGate=true;staticGate=true;
else
    trajectoryGate=trajectoryGenerationCount>=1||scenario.expectedEmergencyLanding;
    controllerGate=controllerPass&&referenceKinematicPass&&executedKinematicPass;
    estimatorGate=estimatorPositionPass&&estimatorAttitudePass;
    staticGate=staticPass&&collisionCount==0&&geofenceViolationCount==0;
end
pass=missionLifecyclePass&&trajectoryGate&&controllerGate&&estimatorGate&&continuityPass&&uncertaintyPass&&staticGate&&~stateTimeoutTriggered;

summary=struct('lifecycleEnabled',true,'goalReached',goalReached,'pass',pass,'failsafeTriggered',failsafe, ...
    'rtlRequested',rtlRequested,'rtlExecuted',rtlExecuted,'missionComplete',missionComplete, ...
    'armedEver',armedEver,'takeoffCompleted',takeoffCompleted,'landed',landed,'disarmed',disarmed, ...
    'preflightRejected',preflightRejected,'emergencyLanding',emergencyLanding, ...
    'alternateLandingUsed',alternateLandingUsed,'selectedLandingXY',selectedLandingXY, ...
    'selectedLandingIndex',selectedLandingIndex,'rtlMidcourseReplanCount',rtlMidcourseReplanCount, ...
    'stateTransitionCount',stateTransitionCount,'stateTimeoutTriggered',stateTimeoutTriggered, ...
    'truthGoalReached',truthGoalReached,'truthTakeoffReached',truthTakeoffReached,'truthLanded',truthLanded, ...
    'completionPass',completionPass,'preflightReasonPass',preflightReasonPass, ...
    'collisionCount',collisionCount,'geofenceViolationCount',geofenceViolationCount,'replanCount',replanCount, ...
    'promotionCount',0,'dynamicAvoidSteps',0,'dynamicHoldCount',0,'obstacleNoDataHoldCount',0,'rejoinCount',0, ...
    'replanBrakeCount',replanBrakeCount,'replanRetryCount',replanRetryCount,'inflationReplanCount',inflationReplanCount, ...
    'dstarInitialExpanded',0,'dstarRepairExpanded',dstarRepairExpanded,'astarScratchExpanded',astarScratchExpanded, ...
    'astarRecoveryCount',astarRecoveryCount,'timeToGoal_s',timeToGoal,'timeToLand_s',timeToLand,'timeToComplete_s',timeToComplete, ...
    'pathLength_m',pathLength,'minObstacleClearance_m',minObs,'minWallClearance_m',minWall,'minDynamicClearance_m',inf, ...
    'minPredictedMargin_m',inf,'maxExecutedSpeed_mps',maxExecutedSpeed,'maxExecutedAccel_mps2',maxExecutedAccel, ...
    'maxExecutedJerk_mps3',maxExecutedJerk,'maxExecutedVerticalSpeed_mps',maxExecutedVerticalSpeed, ...
    'maxExecutedVerticalAccel_mps2',maxExecutedVerticalAccel,'maxExecutedVerticalJerk_mps3',maxExecutedVerticalJerk, ...
    'maxReferenceSpeed_mps',maxReferenceSpeed,'maxReferenceAccel_mps2',maxReferenceAccel,'maxReferenceJerk_mps3',maxReferenceJerk, ...
    'maxVerticalReferenceSpeed_mps',maxVerticalReferenceSpeed,'maxVerticalReferenceAccel_mps2',maxVerticalReferenceAccel, ...
    'maxVerticalReferenceJerk_mps3',maxVerticalReferenceJerk,'maxTilt_deg',rad2deg(maxTilt),'maxAltitudeError_m',maxAltitudeError, ...
    'maxEstimatorPositionError_m',maxEstimatorPositionError,'maxEstimatorAttitudeError_deg',rad2deg(maxEstimatorAttitudeError), ...
    'maxTrackingError_m',maxTrackingError,'maxSafetyOverrideDeviation_m',maxSafetyOverrideDeviation,'maxInflationRadius_m',maxInflation, ...
    'finalInflationRadius_m',currentInflation,'activeLaneFinal',est.activeLane,'laneSwitches',nav.selector.switchCount, ...
    'trajectoryGenerationCount',trajectoryGenerationCount,'trajectoryFallbackCount',trajectoryFallbackCount, ...
    'maxTrajectoryTimeScale',maxTrajectoryTimeScale,'maxReferenceContinuityJump',maxReferenceContinuity,'maxReplanStateJump',maxReplanStateJump, ...
    'staticPass',staticPass,'dynamicPass',true,'referenceKinematicPass',referenceKinematicPass, ...
    'executedKinematicPass',executedKinematicPass,'controllerPass',controllerPass,'estimatorPositionPass',estimatorPositionPass, ...
    'estimatorAttitudePass',estimatorAttitudePass,'continuityPass',continuityPass,'uncertaintyPass',uncertaintyPass, ...
    'missionOutcomePass',missionLifecyclePass,'failsafeExpectationPass',failsafePass,'replanEventPass',rtlReplanPass, ...
    'dynamicEventPass',true,'promotionEventPass',true,'noDataEventPass',true,'laneSwitchEventPass',true, ...
    'rtlEventPass',rtlRequestPass,'eventPass',missionLifecyclePass,'preflightCheck',preflight, ...
    'finalTruthPosition',truth.p.','finalEstimatedPosition',est.p.');

log=struct('t',T,'truthP',Ptrue,'truthV',Vtrue,'truthA',Atrue,'truthJ',Jtrue,'truthQ',Qtrue,'truthRpy',RpyTrue, ...
    'estP',Pest,'estV',Vest,'estQ',Qest,'estRpy',RpyEst,'pRef',Pref,'vRef',Vref,'aRef',Aref, ...
    'trackingError',trackingLog,'estimatorPositionError',estErrorLog,'estimatorAttitudeError_deg',attErrorLog, ...
    'stateId',stateId,'laneId',laneId,'laneScores',laneScores,'laneEligible',laneEligible,'inflationRadius',inflationLog, ...
    'xySigma',xySigmaLog,'thrust_N',thrustLog,'moment_Nm',momentLog,'sensorAids',sensorAidLog,'predictedMargin',predMargin, ...
    'actualDynamic',actualDyn,'estimatedDynamic',estimatedDyn,'pathHistory',{pathHistory},'trajectoryHistory',{trajectoryHistory}, ...
    'activeObstacleHistory',{activeObstacleHistory},'activeObstacleHistoryTime',activeObstacleHistoryTime, ...
    'inflationHistory',inflationHistory,'switchLog',{nav.switchLog}, ...
    'armed',armedLog,'landingTarget',landingTargetLog);
maps=struct('finalGrid',grid,'activeObstacles',activeObstacles,'planner',planner, ...
    'selectedLandingXY',selectedLandingXY,'home',scenario.home);

    function store_sample()
        T(k)=(k-1)*cfg.dt;Ptrue(k,:)=truth.p.';Vtrue(k,:)=truth.v.';Atrue(k,:)=truth.a.';Jtrue(k,:)=currentTruthJerk.';
        Qtrue(k,:)=truth.q;RpyTrue(k,:)=q2rpy_S2_2(truth.q);Pest(k,:)=est.p.';Vest(k,:)=est.v.';Qest(k,:)=est.q;RpyEst(k,:)=q2rpy_S2_2(est.q);
        Pref(k,:)=ref3.p.';Vref(k,:)=ref3.v.';Aref(k,:)=ref3.a.';trackingLog(k)=norm(truth.p-ref3.p);estErrorLog(k)=norm(est.p-truth.p);
        attErrorLog(k)=rad2deg(norm(qlog_S2_2(qmul_S2_2(qconj_S2_2(est.q),truth.q))));stateId(k)=lifecycle_state_id(state);laneId(k)=est.activeLane;
        if ~isempty(est.scores),laneScores(k,:)=est.scores(:).';end;if ~isempty(est.eligible),laneEligible(k,:)=est.eligible(:).';end
        inflationLog(k)=currentInflation;xySigmaLog(k)=est.xySigma_m;thrustLog(k)=cmd.thrust_N;momentLog(k,:)=cmd.moment_Nm(:).';
        sensorAidLog(k,:)=[packet.hasVio packet.hasLidar packet.hasRange packet.hasBaro];predMargin(k)=stepMargin;armedLog(k)=armed;landingTargetLog(k,:)=selectedLandingXY;
    end
end

function [planner,path,traj,stats,jump,routeExists]=plan_segment(cfg,grid,est,estAcc,target,initialScale)
[planner,raw,dstat]=dstar_lite_S2_2('init',grid,est.p(1:2).',target);
[rawA,aInfo]=astar_grid_S2_2(grid,est.p(1:2).',target);
path=prepare_path_local(grid,est.p(1:2).',raw);
[traj,jump]=make_traj(cfg,grid,path,est,estAcc,initialScale);
astarRecovery=0;
if (isempty(path)||~traj.valid)&&~isempty(rawA)
    path=prepare_path_local(grid,est.p(1:2).',rawA);
    [traj,jump]=make_traj(cfg,grid,path,est,estAcc,initialScale);astarRecovery=1;
end
routeExists=~isempty(raw)||~isempty(rawA);
stats=struct('dstarExpanded',dstat.expanded,'astarExpanded',aInfo.expanded,'astarRecovery',astarRecovery);
end

function [planner,path,traj,stats,jump,routeExists]=repair_segment(cfg,planner,grid,changedMask,est,estAcc,target)
[planner,raw,dstat]=dstar_lite_S2_2('repair',planner,grid,est.p(1:2).',changedMask);
[rawA,aInfo]=astar_grid_S2_2(grid,est.p(1:2).',target);
path=prepare_path_local(grid,est.p(1:2).',raw);
[traj,jump]=make_traj(cfg,grid,path,est,estAcc,1.0);astarRecovery=0;
if (isempty(path)||~traj.valid)&&~isempty(rawA)
    [planner,~,~]=dstar_lite_S2_2('init',grid,est.p(1:2).',target);
    path=prepare_path_local(grid,est.p(1:2).',rawA);
    [traj,jump]=make_traj(cfg,grid,path,est,estAcc,1.0);astarRecovery=1;
end
routeExists=~isempty(raw)||~isempty(rawA);
stats=struct('dstarExpanded',dstat.expanded,'astarExpanded',aInfo.expanded,'astarRecovery',astarRecovery);
end

function [traj,jump]=make_traj(cfg,grid,path,est,estAcc,initialScale)
traj=generate_min_snap_trajectory_S2_2(cfg,grid,path,est.v(1:2),estAcc(1:2),initialScale);jump=inf(1,3);
if traj.valid,r=sample_min_snap_state_S2_2(traj,0);jump=[norm(r.p-est.p(1:2)) norm(r.v-est.v(1:2)) norm(r.a-estAcc(1:2))];end
end

function [ng,nf,ms,mc,mr,mv,ma,mj]=update_traj_metrics(traj,jump,ng,nf,ms,mc,mr,mv,ma,mj)
if ~traj.valid,return;end
ng=ng+1;nf=nf+double(traj.fallbackUsed);ms=max(ms,traj.timeScale);mc=max(mc,traj.continuity);mr=max(mr,jump);
mv=max(mv,traj.maxSpeed_mps);ma=max(ma,traj.maxAccel_mps2);mj=max(mj,traj.maxJerk_mps3);
end

function path=prepare_path_local(grid,startXY,raw)
path=zeros(0,2);if isempty(raw),return;end
startXY=double(startXY(:).');raw=double(raw);if size(raw,2)~=2||any(~isfinite(raw(:)))||any(~isfinite(startXY)),return;end
raw=[startXY;raw];raw=remove_duplicates_local(raw,grid.resolution/4);raw(1,:)=startXY;
candidate=smooth_path_S2_2(grid,raw);if ~isempty(candidate),candidate(1,:)=startXY;end
if path_valid_local(grid,candidate),path=candidate;elseif path_valid_local(grid,raw),path=raw;end
end
function tf=path_valid_local(grid,path)
tf=~isempty(path)&&size(path,1)>=2;if ~tf,return;end
for i=1:size(path,1)-1,if segment_occupied_grid_S2_2(grid,path(i,:),path(i+1,:)),tf=false;return;end,end
end
function p=remove_duplicates_local(p,tol)
if isempty(p),return;end
keep=true(size(p,1),1);last=1;for i=2:size(p,1),if norm(p(i,:)-p(last,:))<tol,keep(i)=false;else,last=i;end,end;p=p(keep,:);
end
function cfg2=grid_cfg(cfg,inflation),cfg2=cfg;cfg2.inflationRadius=inflation;end
function ref=hover_ref(xy,z),ref=struct('p',[xy(:);z],'v',zeros(3,1),'a',zeros(3,1),'yaw',0);end
function ref=ground_ref(xy,cfg),ref=hover_ref(xy,cfg.groundHeight_m);end
function cmd=zero_cmd(),cmd=struct('thrust_N',0,'moment_Nm',zeros(3,1),'aCmd',zeros(3,1));end
function tr=invalid_traj(),tr=struct('valid',false,'fallbackUsed',false,'timeScale',0,'continuity',zeros(1,4),'maxSpeed_mps',0,'maxAccel_mps2',0,'maxJerk_mps3',0);end
function [state,entry,emergency,failsafe,hold,z0,t0]=begin_emergency_landing(t,est,newState)
state=newState;entry=t;emergency=true;failsafe=true;hold=est.p(:);
hold(3)=max(est.p(3),0.03);z0=hold(3);t0=t;
end
function id=lifecycle_state_id(s)
switch s
    case 'PREFLIGHT',id=1;case 'ARM',id=2;case 'TAKEOFF',id=3;case 'INITIAL_HOVER',id=4;
    case 'WAIT_FOR_GOAL',id=5;case 'PLAN_OUTBOUND',id=6;case 'TRACK_OUTBOUND',id=7;case 'GOAL_HOVER',id=8;
    case 'PLAN_RTL',id=9;case 'TRACK_RTL',id=10;case 'LAND_HOVER',id=11;case 'LAND_DESCENT',id=12;
    case 'DISARM',id=13;case 'COMPLETE',id=14;case 'PREFLIGHT_REJECT',id=15;case 'EMERGENCY_HOLD',id=16;
    case 'EMERGENCY_LAND',id=17;case 'LIFECYCLE_REPLAN_BRAKE',id=18;case 'FAILSAFE',id=19;otherwise,id=0;
end
end

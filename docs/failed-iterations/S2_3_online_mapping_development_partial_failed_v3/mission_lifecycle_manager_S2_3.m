function [log,summary,maps] = mission_lifecycle_manager_S2_3(cfg,scenario)
% MISSION_LIFECYCLE_MANAGER_S2_3 Stage S2.3 online-mapping lifecycle candidate.
%
% Flow extends the frozen S2.2 lifecycle with perception-driven mapping,
% known-free frontier segments, scan holds and perception-degraded holds.
%
% Loss of observable horizontal navigation while airborne triggers a short
% hold followed by a controlled local emergency landing. Planning and
% control use the selected local ESKF state. Truth is used only for
% simulation safety/performance validation.

cfg.initialPosition=[scenario.start(:);cfg.groundHeight_m];
truth=init_quadrotor_state_S2_2(cfg,scenario.start,cfg.groundHeight_m);
sensorModel=init_sensor_model_S2_2();
[packet,sensorModel]=simulate_sensor_packet_S2_2(cfg,scenario,truth,sensorModel,0,1);
[nav,est]=multi_lane_eskf_lifecycle_S2_2('init',cfg,packet,0);
poseBuffer=init_pose_buffer_S2_3(est,0);
perceptionModel=init_perception_model_S2_3();
% Diagnostic replay capture. This records only autonomy-visible raw rays and
% the exact estimated pose supplied to the mapper; environment truth is not
% copied into the replay stream.
perceptionReplay={};
truthContext=struct('rtlObstacleActive',false,'homeBlockActive',false);
[perceptionPacket,perceptionModel,truthWorld]=simulate_perception_packet_S2_3( ...
    cfg,scenario,truth,perceptionModel,0,1,truthContext);
mapState=init_probabilistic_map_S2_3(cfg);
[mapState,lastMapUpdate]=update_probabilistic_map_S2_3( ...
    cfg,mapState,perceptionPacket,est,0);
if lastMapUpdate.accepted
    replayPose=struct('p',est.p,'q',est.q,'xySigma_m',est.xySigma_m);
    perceptionReplay{end+1}=struct('packet',perceptionPacket, ...
        'pose',replayPose,'callTime',0,'update',lastMapUpdate); %#ok<AGROW>
end

estAcc=zeros(3,1);previousTruthA=truth.a;previousCmdAccel=zeros(3,1);
currentInflation=uncertainty_inflation_S2_2(cfg,est.xySigma_m);
grid=project_map_to_planner_S2_3(cfg,mapState,currentInflation,0);
preflight=preflight_check_S2_3(cfg,scenario,grid,packet,est,mapState,perceptionPacket,0);
activeObstacles=truthWorld.staticRects5(:,1:4);

state='PREFLIGHT';stateEntryTime=0;armed=false;armedEver=false;disarmed=true;
preflightRejected=false;takeoffCompleted=false;goalReached=false;rtlExecuted=false;
landed=false;missionComplete=false;failsafe=false;rtlRequested=false;
emergencyLanding=false;alternateLandingUsed=false;selectedLandingXY=scenario.home;
selectedLandingIndex=1;outboundTarget=scenario.goal;currentTarget=outboundTarget;
holdPosition=[scenario.start(:);cfg.groundHeight_m];
emergencyVerticalOnly=false;
navigationResumeState='';navigationDegradedHoldCount=0;
verticalStartZ=cfg.groundHeight_m;verticalStartTime=0;
planner=[];path=zeros(0,2);traj=invalid_traj();trajClock=0;
pathHistory={};trajectoryHistory={};activeObstacleHistory={activeObstacles};activeObstacleHistoryTime=0;inflationHistory=currentInflation;
mapVersionHistory=double(mapState.version);mapVersionHistoryTime=0;mapSnapshots={};mapSnapshotTimes=[];
lastMapSnapshotTime=-inf;segmentIsFinal=true;segmentMissionGoal=outboundTarget;
scanResumeState='PLAN_OUTBOUND';scanEntryMapVersion=double(mapState.version);scanNoProgressCount=0;
mapExtensionCount=0;mapExtensionPlanCount=0;scanHoldCount=0;goalUnreachable=false;mapSafetyReplanCount=0;
segmentStartGoalDistance=norm(est.p(1:2)-outboundTarget(:));
yawCommand=estimated_yaw_S2_3(est);scanStartYaw=yawCommand;
perceptionHoldCount=0;perceptionResumeState='';perceptionLossStart=nan;
unknownCommitmentCount=0;lastPlannerMapVersion=double(mapState.version);
insertedDone=false(1,numel(scenario.truthInsertedObstacles));homeBlockInserted=false;rtlObstacleInserted=false;rtlObstacleReplanRecorded=false;
rtlTrackStart=nan;rtlMidcourseReplanCount=0;replanCount=0;inflationReplanCount=0;
dstarRepairExpanded=0;astarScratchExpanded=0;astarRecoveryCount=0;
trajectoryGenerationCount=0;trajectoryFallbackCount=0;maxTrajectoryTimeScale=0;
maxReferenceContinuity=zeros(1,4);maxReplanStateJump=zeros(1,3);
maxReferenceSpeed=0;maxReferenceAccel=0;maxReferenceJerk=0;
maxVerticalReferenceSpeed=0;maxVerticalReferenceAccel=0;maxVerticalReferenceJerk=0;
pendingReplan=false;pendingResumeState='';pendingReplanStart=nan;lastReplanAttempt=-inf;
replanBrakeCount=0;replanRetryCount=0;
gridFallbackActive=false;gridFallbackPath=zeros(0,2);gridFallbackIndex=1;
gridFallbackCount=0;fallbackVelCmd=zeros(2,1);fallbackAccelCmd=zeros(2,1);
pendingFallbackPath=zeros(0,2);
xyLossStartTime=nan;xyLossVelocityTrustEnd=nan;
lastReliableXYVelocity=est.v(1:2);xyLossBrakeAccel=zeros(2,1);xyLossBrakeEndTime=nan;
xyLossBrakeReleaseTime=nan;xyLossBrakeReleaseSpeed=nan;

pathLength=0;minWall=inf;minObs=inf;minDynamicClearance=inf;collisionCount=0;geofenceViolationCount=0;
maxExecutedSpeed=0;maxExecutedAccel=0;maxExecutedJerk=0;maxExecutedVerticalSpeed=0;
maxExecutedVerticalAccel=0;maxExecutedVerticalJerk=0;
maxTilt=0;maxAltitudeError=0;maxEstimatorPositionError=0;maxEstimatorAttitudeError=0;
maxEstimatorPositionErrorObservable=0;maxEstimatorPositionErrorPostLoss=0;
estimatorPositionErrorAtFailsafeTrigger=nan;emergencyStartTruthXY=[nan;nan];
captureFailsafeTriggerMetrics=false;maxEmergencyHorizontalDrift=0;
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
sensorAidLog=false(maxSteps,4);predMargin=nan(maxSteps,1);
nDynamicLog=max(1,numel(scenario.truthDynamicObstacles));
actualDyn=nan(maxSteps,nDynamicLog,2);estimatedDyn=nan(maxSteps,nDynamicLog,2);
armedLog=false(maxSteps,1);landingTargetLog=nan(maxSteps,2);degradedLog=false(maxSteps,1);rtlRequestLog=false(maxSteps,1);
mapVersionLog=nan(maxSteps,1);knownFreeFractionLog=nan(maxSteps,1);unknownFractionLog=nan(maxSteps,1);
perceptionFreshLog=false(maxSteps,1);segmentFinalLog=false(maxSteps,1);mapUpdateAcceptedLog=false(maxSteps,1);

k=1;ref3=ground_ref(scenario.start,cfg);cmd=zero_cmd();stepMargin=inf;currentTruthJerk=zeros(3,1);
store_sample();

for k=2:maxSteps
    t=(k-1)*cfg.dt;
    previousState=state;mapChanged=false;changedMask=false(grid.ny,grid.nx);

    % Simulation-only world activation. These flags are consumed only by
    % raw sensor generation and independent truth validation.
    for i=1:numel(scenario.truthInsertedObstacles)
        if ~insertedDone(i)&&t>=scenario.truthInsertedObstacles(i).time
            insertedDone(i)=true;
        end
    end
    if scenario.rtlObstacle.enabled&&strcmp(state,'TRACK_RTL')&& ...
            ~rtlObstacleInserted&&isfinite(rtlTrackStart)&& ...
            t-rtlTrackStart>=scenario.rtlObstacle.delay_s
        rtlObstacleInserted=true;
    end
    if scenario.truthHomeBlockAtRTL&&any(strcmp(state,{'PLAN_RTL','TRACK_RTL'}))
        homeBlockInserted=true;
    end
    truthContext=struct('rtlObstacleActive',rtlObstacleInserted, ...
        'homeBlockActive',homeBlockInserted);

    % Insert the previous timestamped raw perception packet using the current
    % selected ESKF output. The mapper has no environment-truth argument.
    oldGrid=grid;
    mapPose=interpolate_pose_buffer_S2_3(poseBuffer,perceptionPacket.timestamp);
    [mapState,lastMapUpdate]=update_probabilistic_map_S2_3( ...
        cfg,mapState,perceptionPacket,mapPose,t);
    if lastMapUpdate.accepted
        replayPose=struct('p',mapPose.p,'q',mapPose.q,'xySigma_m',mapPose.xySigma_m);
        perceptionReplay{end+1}=struct('packet',perceptionPacket, ...
            'pose',replayPose,'callTime',t,'update',lastMapUpdate); %#ok<AGROW>
    end
    proposedInflation=uncertainty_inflation_S2_2(cfg,est.xySigma_m);
    if proposedInflation>currentInflation+cfg.inflationReplanThreshold_m
        currentInflation=proposedInflation;
        inflationReplanCount=inflationReplanCount+1;
        inflationHistory(end+1)=currentInflation; %#ok<AGROW>
    end
    grid=project_map_to_planner_S2_3(cfg,mapState,currentInflation,t);
    changedMask=xor(oldGrid.occ,grid.occ);
    newlyBlockedMask=grid.occ&~oldGrid.occ;
    mapChanged=any(newlyBlockedMask(:));
    routeAffected=false;
    if mapChanged&&any(strcmp(state,{'TRACK_OUTBOUND','TRACK_RTL'}))
        routeAffected=changed_cells_affect_route_S2_3(grid,newlyBlockedMask, ...
            traj,trajClock,gridFallbackActive,gridFallbackPath,gridFallbackIndex,est.p(1:2));
    end
    if double(mapState.version)~=mapVersionHistory(end)
        mapVersionHistory(end+1)=double(mapState.version); %#ok<AGROW>
        mapVersionHistoryTime(end+1)=t; %#ok<AGROW>
    end
    if t-lastMapSnapshotTime>=cfg.mapStoreSnapshotPeriod_s
        mapSnapshots{end+1}=struct('time',t,'version',mapState.version, ...
            'knownFree',grid.knownFree,'unknown',grid.unknown, ...
            'staticOccupied',grid.staticOccupied,'dynamicOccupied',grid.dynamicOccupied); %#ok<AGROW>
        mapSnapshotTimes(end+1)=t;lastMapSnapshotTime=t; %#ok<AGROW>
    end
    maxInflation=max(maxInflation,currentInflation);

    perceptionAge=t-max(mapState.lastLidarTime,mapState.lastDepthTime);
    perceptionFresh=isfinite(perceptionAge)&&perceptionAge<=cfg.mapPerceptionHoldTimeout_s;
    if armed&&~perceptionFresh&&any(strcmp(state,{'TRACK_OUTBOUND','TRACK_RTL','SCAN_HOLD'}))
        perceptionResumeState=state;state='MAP_DEGRADED_HOLD';stateEntryTime=t;
        perceptionLossStart=t;perceptionHoldCount=perceptionHoldCount+1;holdPosition=est.p;
        gridFallbackActive=false;pendingReplan=false;
    end

    % A newly blocked cell repairs the active segment only when its already
    % inflated footprint intersects the future route. Map changes elsewhere
    % remain available to the next planning call without resetting progress.
    if routeAffected&&any(strcmp(state,{'TRACK_OUTBOUND','TRACK_RTL'}))
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
            path=candidatePath;traj=candidateTraj;trajClock=0;gridFallbackActive=false;
            [trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump,maxReferenceSpeed,maxReferenceAccel,maxReferenceJerk]= ...
                update_traj_metrics(traj,jump,trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump,maxReferenceSpeed,maxReferenceAccel,maxReferenceJerk);
        elseif routeExists
            pendingReplan=true;pendingResumeState=resumeState;pendingReplanStart=t;lastReplanAttempt=t;
            pendingFallbackPath=candidatePath;
            replanBrakeCount=replanBrakeCount+1;state='LIFECYCLE_REPLAN_BRAKE';holdPosition=est.p;
        else
            mapSafetyReplanCount=mapSafetyReplanCount+1;
            if strcmp(resumeState,'TRACK_RTL')
                state='PLAN_RTL';stateEntryTime=t;holdPosition=est.p;
            else
                scanNoProgressCount=scanNoProgressCount+1;
                if scanNoProgressCount>=cfg.mapMaxNoProgressScans|| ...
                        mapExtensionCount>=cfg.mapMaxExtensionAttempts
                    goalUnreachable=true;state='GOAL_UNREACHABLE';stateEntryTime=t;holdPosition=est.p;
                else
                    state='SCAN_HOLD';stateEntryTime=t;scanResumeState='PLAN_OUTBOUND';scanStartYaw=yawCommand;
                    scanEntryMapVersion=double(mapState.version);holdPosition=est.p;scanHoldCount=scanHoldCount+1;
                end
            end
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
        if routeExists&&~isempty(candidatePath),pendingFallbackPath=candidatePath;end
        if ~isempty(candidatePath)&&candidateTraj.valid
            path=candidatePath;traj=candidateTraj;trajClock=0;pendingReplan=false;
            gridFallbackActive=false;pendingFallbackPath=zeros(0,2);
            state=pendingResumeState;stateEntryTime=t;arrivalConfirmTimer=0;
            if strcmp(state,'TRACK_RTL')
                rtlExecuted=true;
                if ~isfinite(rtlTrackStart),rtlTrackStart=t;end
            end
            [trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump,maxReferenceSpeed,maxReferenceAccel,maxReferenceJerk]= ...
                update_traj_metrics(traj,jump,trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump,maxReferenceSpeed,maxReferenceAccel,maxReferenceJerk);
        elseif routeExists&&~isempty(pendingFallbackPath)&& ...
                replanRetryCount>=cfg.gridFallbackRetryLimit
            % Smooth trajectory generation remains invalid even after the
            % vehicle has braked. Execute the verified grid route with a
            % stop-at-corner velocity-mode fallback instead of landing at the
            % current location while a safe route still exists.
            path=pendingFallbackPath;gridFallbackPath=path;
            gridFallbackIndex=min(2,size(gridFallbackPath,1));
            gridFallbackActive=true;gridFallbackCount=gridFallbackCount+1;
            fallbackVelCmd=est.v(1:2);fallbackAccelCmd=zeros(2,1);
            pendingReplan=false;pendingFallbackPath=zeros(0,2);
            state=pendingResumeState;stateEntryTime=t;arrivalConfirmTimer=0;
            if strcmp(state,'TRACK_RTL')
                rtlExecuted=true;
                if ~isfinite(rtlTrackStart),rtlTrackStart=t;end
            end
        elseif ~routeExists
            % In an unknown environment, failure to find a route on the
            % current partial map is not proof that the mission is unsafe.
            % Stop in known free space, scan, and re-enter the appropriate
            % planner. Emergency landing remains reserved for exhausted RTL
            % recovery or loss of navigation/perception safety.
            resumeAfterFailure=pendingResumeState;
            pendingReplan=false;pendingFallbackPath=zeros(0,2);gridFallbackActive=false;
            if strcmp(resumeAfterFailure,'TRACK_OUTBOUND')
                scanNoProgressCount=scanNoProgressCount+1;
            end
            if strcmp(resumeAfterFailure,'TRACK_OUTBOUND')&& ...
                    (scanNoProgressCount>=cfg.mapMaxNoProgressScans|| ...
                    mapExtensionCount>=cfg.mapMaxExtensionAttempts)
                goalUnreachable=true;state='GOAL_UNREACHABLE';stateEntryTime=t;holdPosition=est.p;
            else
                state='SCAN_HOLD';stateEntryTime=t;holdPosition=est.p;scanStartYaw=yawCommand;
                if strcmp(resumeAfterFailure,'TRACK_RTL'),scanResumeState='PLAN_RTL';else,scanResumeState='PLAN_OUTBOUND';end
                scanEntryMapVersion=double(mapState.version);scanHoldCount=scanHoldCount+1;
            end
        end
    end
    if pendingReplan&&t-pendingReplanStart>=cfg.rtlReplanTimeout_s
        % One final geometry-only recovery is attempted before emergency
        % landing. A valid route is executed with the conservative grid
        % fallback; emergency landing is reserved for an unreachable target.
        [rawFallback,aFallback]=astar_grid_S2_2(grid,est.p(1:2).',currentTarget);
        astarScratchExpanded=astarScratchExpanded+aFallback.expanded;
        timeoutPath=prepare_path_local(grid,est.p(1:2).',rawFallback);
        if ~isempty(timeoutPath)
            path=timeoutPath;gridFallbackPath=path;
            gridFallbackIndex=min(2,size(gridFallbackPath,1));
            gridFallbackActive=true;gridFallbackCount=gridFallbackCount+1;
            fallbackVelCmd=est.v(1:2);fallbackAccelCmd=zeros(2,1);
            pendingReplan=false;pendingFallbackPath=zeros(0,2);
            state=pendingResumeState;stateEntryTime=t;arrivalConfirmTimer=0;
            if strcmp(state,'TRACK_RTL')
                rtlExecuted=true;
                if ~isfinite(rtlTrackStart),rtlTrackStart=t;end
            end
        else
            resumeAfterFailure=pendingResumeState;
            pendingReplan=false;pendingFallbackPath=zeros(0,2);gridFallbackActive=false;
            if strcmp(resumeAfterFailure,'TRACK_OUTBOUND')
                scanNoProgressCount=scanNoProgressCount+1;
            end
            if strcmp(resumeAfterFailure,'TRACK_OUTBOUND')&& ...
                    (scanNoProgressCount>=cfg.mapMaxNoProgressScans|| ...
                    mapExtensionCount>=cfg.mapMaxExtensionAttempts)
                goalUnreachable=true;state='GOAL_UNREACHABLE';stateEntryTime=t;holdPosition=est.p;
            else
                state='SCAN_HOLD';stateEntryTime=t;holdPosition=est.p;scanStartYaw=yawCommand;
                if strcmp(resumeAfterFailure,'TRACK_RTL'),scanResumeState='PLAN_RTL';else,scanResumeState='PLAN_OUTBOUND';end
                scanEntryMapVersion=double(mapState.version);scanHoldCount=scanHoldCount+1;
            end
        end
    end

    % Retain the most recent aid-bounded velocity. It is frozen at loss
    % detection and used only to construct a short open-loop braking pulse;
    % the subsequent blind descent never chases a drifting inertial velocity.
    if ~est.degraded&&est.horizontalAidAge_s<=cfg.lifecycleXYRecoveryAge_s
        lastReliableXYVelocity=est.v(1:2);
    end

    % Detect horizontal-aid loss before the full lane timeout expires. VIO
    % normally updates every 0.04 s, so a 0.35 s age is a strong causal loss
    % indication while still leaving time to brake before a wall is reached.
    xyLossDetected=armed&&(est.degraded|| ...
        est.horizontalAidAge_s>=cfg.lifecycleXYLossDetectionAge_s);
    if xyLossDetected&&~any(strcmp(state,{'NAV_DEGRADED_HOLD','EMERGENCY_HOLD', ...
            'EMERGENCY_LAND','LAND_DESCENT','DISARM','COMPLETE', ...
            'PREFLIGHT_REJECT','FAILSAFE'}))
        navigationResumeState=state;state='NAV_DEGRADED_HOLD';stateEntryTime=t;
        navigationDegradedHoldCount=navigationDegradedHoldCount+1;
        holdPosition=est.p;xyLossStartTime=t;
        xyLossVelocityTrustEnd=t+cfg.xyLossVelocityTrustTime_s;
        [xyLossBrakeAccel,xyLossBrakeEndTime]=make_xy_loss_brake( ...
            cfg,lastReliableXYVelocity,t);
        gridFallbackActive=false;pendingReplan=false;
    end

    % The ESKF request remains a valid trigger, but the lifecycle manager may
    % already have started the local emergency response earlier.
    if est.rtlRequested&&armed&&~any(strcmp(state,{ ...
            'EMERGENCY_HOLD','EMERGENCY_LAND','LAND_DESCENT', ...
            'DISARM','COMPLETE','PREFLIGHT_REJECT','FAILSAFE'}))
        rtlRequested=true;emergencyVerticalOnly=true;
        if ~isfinite(xyLossStartTime)
            xyLossStartTime=t;
            [xyLossBrakeAccel,xyLossBrakeEndTime]=make_xy_loss_brake( ...
                cfg,lastReliableXYVelocity,t);
        end
        if ~isfinite(xyLossVelocityTrustEnd),xyLossVelocityTrustEnd=t+cfg.xyLossVelocityTrustTime_s;end
        if ~isfinite(estimatorPositionErrorAtFailsafeTrigger),captureFailsafeTriggerMetrics=true;end
        [state,stateEntryTime,emergencyLanding,failsafe,holdPosition,verticalStartZ,verticalStartTime]= ...
            begin_emergency_landing(t,est,'EMERGENCY_HOLD');
        pendingReplan=false;gridFallbackActive=false;
    end

    ref3=ground_ref(scenario.start,cfg);motorsActive=false;trackingError=nan;
    % Arm touchdown detection only after the commanded vertical profile has
    % completed. A qualified contact pulse is then latched; persistence is
    % not required because the simple ground model can rebound by millimetres.
    landingDetectionArmed=false;

    switch state
        case 'NAV_DEGRADED_HOLD'
            motorsActive=true;
            [brakeAccel,dampingEnabled]=xy_loss_horizontal_command( ...
                t,xyLossBrakeEndTime,xyLossVelocityTrustEnd,xyLossBrakeAccel);
            ref3=struct('p',[est.p(1:2);holdPosition(3)], ...
                'v',zeros(3,1),'a',[brakeAccel(:);0],'yaw',yawCommand, ...
                'horizontalControlEnabled',false, ...
                'horizontalVelocityDampingEnabled',dampingEnabled, ...
                'horizontalVelocityDampingGain',cfg.emergencyVelocityDampingGain, ...
                'horizontalFeedforwardAccelEnabled',true);
            recovered=~est.degraded&& ...
                est.horizontalAidAge_s<=cfg.lifecycleXYRecoveryAge_s;
            if recovered
                state=navigationResumeState;stateEntryTime=t;arrivalConfirmTimer=0;
                navigationResumeState='';xyLossStartTime=nan;xyLossVelocityTrustEnd=nan;
                xyLossBrakeAccel=zeros(2,1);xyLossBrakeEndTime=nan;
            elseif isfinite(xyLossStartTime)&& ...
                    t-xyLossStartTime>=cfg.lifecycleXYLossEmergencyDelay_s
                rtlRequested=true;emergencyVerticalOnly=true;
                if ~isfinite(estimatorPositionErrorAtFailsafeTrigger),captureFailsafeTriggerMetrics=true;end
                [state,stateEntryTime,emergencyLanding,failsafe,holdPosition,verticalStartZ,verticalStartTime]= ...
                    begin_emergency_landing(t,est,'EMERGENCY_HOLD');
                pendingReplan=false;gridFallbackActive=false;
            end

        case 'PREFLIGHT'
            if t-stateEntryTime>=cfg.preflightCheckTime_s
                preflight=preflight_check_S2_3(cfg,scenario,grid,packet,est,mapState,perceptionPacket,t);
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
            ref3=struct('p',[scenario.home(:);vz.z],'v',[0;0;vz.vz],'a',[0;0;vz.az],'yaw',yawCommand);
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
                holdPosition=[scenario.home(:);cfg.altitudeNominal_m];scanStartYaw=yawCommand;
            end

        case 'INITIAL_HOVER'
            motorsActive=true;
            scanYaw=wrap_pi_S2_2(scanStartYaw+cfg.mapScanYawRate_radps*(t-stateEntryTime));
            yawCommand=scanYaw;ref3=hover_ref_yaw(scenario.home,cfg.altitudeNominal_m,yawCommand);
            if t-stateEntryTime>=max(cfg.initialHoverTime_s,cfg.mapInitialScanTime_s)
                state='WAIT_FOR_GOAL';stateEntryTime=t;
            end

        case 'WAIT_FOR_GOAL'
            motorsActive=true;ref3=hover_ref_yaw(scenario.home,cfg.altitudeNominal_m,yawCommand);
            if t-stateEntryTime>=scenario.goalCommandDelay_s,state='PLAN_OUTBOUND';stateEntryTime=t;end

        case 'PLAN_OUTBOUND'
            motorsActive=true;segmentMissionGoal=outboundTarget;
            ref3=hover_ref_yaw(est.p(1:2).',cfg.altitudeNominal_m,yawCommand);
            [planner,path,traj,st,jump,routeExists,planMeta]=plan_unknown_segment_S2_3( ...
                cfg,grid,est,estAcc,segmentMissionGoal,scenario.trajectoryInitialTimeScale);
            dstarRepairExpanded=dstarRepairExpanded+st.dstarExpanded;
            astarScratchExpanded=astarScratchExpanded+st.astarExpanded;
            astarRecoveryCount=astarRecoveryCount+st.astarRecovery;
            pathHistory{end+1}=path;trajectoryHistory{end+1}=traj; %#ok<AGROW>
            currentTarget=planMeta.segmentTarget;segmentIsFinal=planMeta.isFinal;
            segmentStartGoalDistance=norm(est.p(1:2)-segmentMissionGoal(:));
            if planMeta.frontierUsed,mapExtensionPlanCount=mapExtensionPlanCount+1;end
            if routeExists&&traj.valid
                trajClock=0;state='TRACK_OUTBOUND';stateEntryTime=t;arrivalConfirmTimer=0;
                lastPlannerMapVersion=double(grid.mapVersion);
                ref3=trajectory_start_ref(traj,cfg.altitudeNominal_m,yawCommand);
                [trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump,maxReferenceSpeed,maxReferenceAccel,maxReferenceJerk]= ...
                    update_traj_metrics(traj,jump,trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump,maxReferenceSpeed,maxReferenceAccel,maxReferenceJerk);
            elseif routeExists
                pendingReplan=true;pendingResumeState='TRACK_OUTBOUND';pendingReplanStart=t;lastReplanAttempt=t;
                pendingFallbackPath=path;replanBrakeCount=replanBrakeCount+1;
                state='LIFECYCLE_REPLAN_BRAKE';stateEntryTime=t;holdPosition=est.p;
            else
                scanNoProgressCount=scanNoProgressCount+1;
                if scanNoProgressCount>=cfg.mapMaxNoProgressScans||mapExtensionCount>=cfg.mapMaxExtensionAttempts
                    goalUnreachable=true;state='GOAL_UNREACHABLE';stateEntryTime=t;holdPosition=est.p;
                else
                    state='SCAN_HOLD';stateEntryTime=t;scanResumeState='PLAN_OUTBOUND';scanStartYaw=yawCommand;
                    scanEntryMapVersion=double(mapState.version);scanHoldCount=scanHoldCount+1;holdPosition=est.p;
                end
            end

        case {'TRACK_OUTBOUND','TRACK_RTL'}
            motorsActive=true;
            if gridFallbackActive
                [targetVel,gridFallbackIndex]=grid_fallback_target( ...
                    cfg,gridFallbackPath,gridFallbackIndex,est.p(1:2),est.v(1:2));
                cfgFallback=cfg;
                cfgFallback.safetyVelocityAccelLimit_mps2=cfg.gridFallbackAccelLimit_mps2;
                cfgFallback.safetyVelocityJerkLimit_mps3=cfg.gridFallbackJerkLimit_mps3;
                [fallbackVelCmd,fallbackAccelCmd]=shape_velocity_command_S2_2( ...
                    cfgFallback,fallbackVelCmd,fallbackAccelCmd,targetVel);
                ref3=struct('p',[est.p(1:2);cfg.altitudeNominal_m], ...
                    'v',[fallbackVelCmd(:);0],'a',[fallbackAccelCmd(:);0],'yaw',yawCommand);
                trackingError=norm(est.p(1:2).'-currentTarget);
            else
                [desiredVel,trackingError,refXY]=track_smooth_trajectory_S2_2( ...
                    cfg,est.p(1:2),est.v(1:2),traj,trajClock); %#ok<ASGLU>
                ref3=struct('p',[refXY.p(:);cfg.altitudeNominal_m], ...
                    'v',[refXY.v(:);0],'a',[refXY.a(:);0],'yaw',yawCommand);
                if traj.valid&&isfinite(trackingError)&& ...
                        trackingError<=cfg.trajectoryClockMaxError_m
                    trajClock=min(traj.duration_s,trajClock+cfg.dt);
                end
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
                arrivalConfirmTimer=0;gridFallbackActive=false;
                if strcmp(state,'TRACK_OUTBOUND')
                    if segmentIsFinal
                        goalReached=true;timeToGoal=t;state='GOAL_HOVER';stateEntryTime=t;
                        holdPosition=[segmentMissionGoal(:);cfg.altitudeNominal_m];scanNoProgressCount=0;
                    else
                        extensionProgress=segmentStartGoalDistance- ...
                            norm(est.p(1:2)-segmentMissionGoal(:));
                        mapExtensionCount=mapExtensionCount+1;
                        if extensionProgress>=cfg.mapMinFrontierProgress_m
                            scanNoProgressCount=0;
                        else
                            scanNoProgressCount=scanNoProgressCount+1;
                        end
                        state='SCAN_HOLD';stateEntryTime=t;scanResumeState='PLAN_OUTBOUND';scanStartYaw=yawCommand;
                        scanEntryMapVersion=double(mapState.version);scanHoldCount=scanHoldCount+1;holdPosition=est.p;
                    end
                else
                    if segmentIsFinal
                        state='LAND_HOVER';stateEntryTime=t;
                        holdPosition=[selectedLandingXY(:);cfg.altitudeNominal_m];scanNoProgressCount=0;
                    else
                        extensionProgress=segmentStartGoalDistance- ...
                            norm(est.p(1:2)-segmentMissionGoal(:));
                        mapExtensionCount=mapExtensionCount+1;
                        if extensionProgress>=cfg.mapMinFrontierProgress_m
                            scanNoProgressCount=0;
                        else
                            scanNoProgressCount=scanNoProgressCount+1;
                        end
                        state='SCAN_HOLD';stateEntryTime=t;scanResumeState='PLAN_RTL';scanStartYaw=yawCommand;
                        scanEntryMapVersion=double(mapState.version);scanHoldCount=scanHoldCount+1;holdPosition=est.p;
                    end
                end
            end

        case 'GOAL_HOVER'
            motorsActive=true;ref3=hover_ref_yaw(scenario.goal,cfg.altitudeNominal_m,yawCommand);
            if t-stateEntryTime>=cfg.goalHoverTime_s&&scenario.requestRTLAfterGoal
                state='PLAN_RTL';stateEntryTime=t;
            end

        case 'PLAN_RTL'
            motorsActive=true;ref3=hover_ref_yaw(est.p(1:2).',cfg.altitudeNominal_m,yawCommand);
            candidates=[scenario.home;scenario.alternateLandingZones];
            landingChoice=select_safe_landing_zone_S2_3(cfg,grid,est.p(1:2).',candidates,t);
            astarScratchExpanded=astarScratchExpanded+landingChoice.astarExpanded;
            if ~landingChoice.valid
                scanNoProgressCount=scanNoProgressCount+1;
                if scanNoProgressCount<cfg.mapMaxNoProgressScans
                    state='SCAN_HOLD';stateEntryTime=t;scanResumeState='PLAN_RTL';scanStartYaw=yawCommand;
                    scanEntryMapVersion=double(mapState.version);scanHoldCount=scanHoldCount+1;holdPosition=est.p;
                else
                    emergencyVerticalOnly=false;
                    [state,stateEntryTime,emergencyLanding,failsafe,holdPosition,verticalStartZ,verticalStartTime]= ...
                        begin_emergency_landing(t,est,'EMERGENCY_HOLD');
                end
            else
                selectedLandingXY=landingChoice.xy;selectedLandingIndex=landingChoice.candidateIndex;
                alternateLandingUsed=selectedLandingIndex>1;segmentMissionGoal=selectedLandingXY;
                [planner,path,traj,st,jump,routeExists,planMeta]=plan_unknown_segment_S2_3( ...
                    cfg,grid,est,estAcc,segmentMissionGoal,1.0);
                dstarRepairExpanded=dstarRepairExpanded+st.dstarExpanded;
                astarScratchExpanded=astarScratchExpanded+st.astarExpanded;
                astarRecoveryCount=astarRecoveryCount+st.astarRecovery;
                pathHistory{end+1}=path;trajectoryHistory{end+1}=traj; %#ok<AGROW>
                currentTarget=planMeta.segmentTarget;segmentIsFinal=planMeta.isFinal;
                segmentStartGoalDistance=norm(est.p(1:2)-segmentMissionGoal(:));
                if planMeta.frontierUsed,mapExtensionPlanCount=mapExtensionPlanCount+1;end
                if routeExists&&traj.valid
                    trajClock=0;gridFallbackActive=false;rtlExecuted=true;
                    if ~isfinite(rtlTrackStart),rtlTrackStart=t;end
                    state='TRACK_RTL';stateEntryTime=t;arrivalConfirmTimer=0;
                    lastPlannerMapVersion=double(grid.mapVersion);
                    ref3=trajectory_start_ref(traj,cfg.altitudeNominal_m,yawCommand);
                    [trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump,maxReferenceSpeed,maxReferenceAccel,maxReferenceJerk]= ...
                        update_traj_metrics(traj,jump,trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump,maxReferenceSpeed,maxReferenceAccel,maxReferenceJerk);
                elseif routeExists
                    pendingReplan=true;pendingResumeState='TRACK_RTL';pendingReplanStart=t;lastReplanAttempt=t;
                    pendingFallbackPath=path;replanBrakeCount=replanBrakeCount+1;
                    state='LIFECYCLE_REPLAN_BRAKE';stateEntryTime=t;holdPosition=est.p;
                else
                    state='SCAN_HOLD';stateEntryTime=t;scanResumeState='PLAN_RTL';scanStartYaw=yawCommand;
                    scanEntryMapVersion=double(mapState.version);scanHoldCount=scanHoldCount+1;holdPosition=est.p;
                end
            end

        case 'SCAN_HOLD'
            motorsActive=true;
            scanYaw=wrap_pi_S2_2(scanStartYaw+cfg.mapScanYawRate_radps*(t-stateEntryTime));
            yawCommand=scanYaw;ref3=hover_ref_yaw(holdPosition(1:2).',cfg.altitudeNominal_m,yawCommand);
            if t-stateEntryTime>=cfg.mapScanHoldTime_s
                % A changing map version alone is not physical progress.
                % Resume planning; PLAN_* and completed frontier arrivals
                % update the no-progress watchdog using route/progress evidence.
                state=scanResumeState;stateEntryTime=t;
            end

        case 'MAP_DEGRADED_HOLD'
            motorsActive=true;ref3=hover_ref_yaw(est.p(1:2).',cfg.altitudeNominal_m,yawCommand);
            perceptionAge=t-max(mapState.lastLidarTime,mapState.lastDepthTime);
            if isfinite(perceptionAge)&&perceptionAge<=cfg.mapPerceptionHoldTimeout_s
                if any(strcmp(perceptionResumeState,{'TRACK_OUTBOUND','TRACK_RTL'}))
                    if strcmp(perceptionResumeState,'TRACK_RTL'),state='PLAN_RTL';else,state='PLAN_OUTBOUND';end
                else
                    state=perceptionResumeState;
                end
                stateEntryTime=t;perceptionLossStart=nan;
            elseif isfinite(perceptionLossStart)&&t-perceptionLossStart>=cfg.mapPerceptionFailsafeTimeout_s
                % Do not initiate new translation on a stale obstacle map.
                % Persistent loss of all obstacle perception therefore uses
                % the validated controlled emergency-landing path.
                rtlRequested=true;emergencyVerticalOnly=false;
                [state,stateEntryTime,emergencyLanding,failsafe,holdPosition,verticalStartZ,verticalStartTime]= ...
                    begin_emergency_landing(t,est,'EMERGENCY_HOLD');
            end

        case 'GOAL_UNREACHABLE'
            motorsActive=true;ref3=hover_ref_yaw(est.p(1:2).',cfg.altitudeNominal_m,yawCommand);
            if t-stateEntryTime>=cfg.goalHoverTime_s
                rtlRequested=true;state='PLAN_RTL';stateEntryTime=t;
            end

        case 'LAND_HOVER'
            motorsActive=true;ref3=hover_ref_yaw(selectedLandingXY,cfg.altitudeNominal_m,yawCommand);
            if t-stateEntryTime>=cfg.rtlArrivalHoverTime_s
                state='LAND_DESCENT';stateEntryTime=t;verticalStartTime=t;verticalStartZ=est.p(3);landContactTimer=0;landDetected=false;
            end

        case 'LAND_DESCENT'
            motorsActive=true;
            vz=vertical_profile_S2_2(verticalStartZ,cfg.groundHeight_m,cfg.landingDuration_s,t-verticalStartTime);
            landingDetectionArmed=vz.complete;
            ref3=struct('p',[selectedLandingXY(:);vz.z],'v',[0;0;vz.vz],'a',[0;0;vz.az],'yaw',yawCommand);
            maxVerticalReferenceSpeed=max(maxVerticalReferenceSpeed,abs(vz.vz));
            maxVerticalReferenceAccel=max(maxVerticalReferenceAccel,abs(vz.az));
            maxVerticalReferenceJerk=max(maxVerticalReferenceJerk,abs(vz.jz));
            if vz.complete&&landDetected
                landed=true;timeToLand=t;state='DISARM';stateEntryTime=t;
            end

        case 'EMERGENCY_HOLD'
            motorsActive=true;
            if emergencyVerticalOnly
                [brakeAccel,dampingEnabled]=xy_loss_horizontal_command( ...
                    t,xyLossBrakeEndTime,xyLossVelocityTrustEnd,xyLossBrakeAccel);
                ref3=struct('p',[est.p(1:2);holdPosition(3)], ...
                    'v',zeros(3,1),'a',[brakeAccel(:);0],'yaw',yawCommand, ...
                    'horizontalControlEnabled',false, ...
                    'horizontalVelocityDampingEnabled',dampingEnabled, ...
                    'horizontalVelocityDampingGain',cfg.emergencyVelocityDampingGain, ...
                    'horizontalFeedforwardAccelEnabled',true);
                holdDuration=cfg.xyLossEmergencyHoldTime_s;

                % Do not use the post-loss ESKF velocity to decide whether
                % braking is complete: after absolute XY aiding disappears,
                % that velocity can drift and reverse the command. Apply the
                % frozen aid-bounded impulse once, then hold zero horizontal
                % acceleration long enough for the jerk-limited command to
                % return to zero before starting the vertical descent.
                brakeTimeComplete=~isfinite(xyLossBrakeEndTime)||t>=xyLossBrakeEndTime;
                brakeSettleComplete=~isfinite(xyLossBrakeEndTime)|| ...
                    t>=xyLossBrakeEndTime+cfg.xyLossBrakeSettleTime_s;
                horizontalBrakeComplete=brakeTimeComplete&&brakeSettleComplete;
                holdComplete=(t-stateEntryTime>=holdDuration)&&horizontalBrakeComplete;
            else
                ref3=struct('p',holdPosition(:),'v',zeros(3,1), ...
                    'a',zeros(3,1),'yaw',yawCommand,'horizontalControlEnabled',true);
                holdDuration=cfg.emergencyHoldTime_s;
                holdComplete=t-stateEntryTime>=holdDuration;
            end
            if holdComplete
                if emergencyVerticalOnly
                    xyLossBrakeReleaseTime=t;
                    xyLossBrakeReleaseSpeed=norm(est.v(1:2));
                end
                selectedLandingXY=est.p(1:2).';state='EMERGENCY_LAND';stateEntryTime=t;
                verticalStartTime=t;verticalStartZ=est.p(3);landContactTimer=0;landDetected=false;
            end

        case 'EMERGENCY_LAND'
            motorsActive=true;
            if emergencyVerticalOnly,descentDuration=cfg.emergencyLandingDuration_s;else,descentDuration=cfg.landingDuration_s;end
            vz=vertical_profile_S2_2(verticalStartZ,cfg.groundHeight_m,descentDuration,t-verticalStartTime);
            landingDetectionArmed=vz.complete;
            if emergencyVerticalOnly
                % Horizontal braking has already completed in EMERGENCY_HOLD.
                % During the blind descent command a level vehicle and no XY
                % position/velocity feedback, so a drifting inertial estimate
                % cannot create a new horizontal acceleration command.
                ref3=struct('p',[est.p(1:2);vz.z],'v',[0;0;vz.vz], ...
                    'a',[0;0;vz.az],'yaw',yawCommand,'horizontalControlEnabled',false, ...
                    'horizontalVelocityDampingEnabled',false, ...
                    'horizontalFeedforwardAccelEnabled',false);
            else
                ref3=struct('p',[selectedLandingXY(:);vz.z],'v',[0;0;vz.vz], ...
                    'a',[0;0;vz.az],'yaw',yawCommand,'horizontalControlEnabled',true);
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
                'v',zeros(3,1),'a',zeros(3,1),'yaw',yawCommand);

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
            ~any(strcmp(state,{'TRACK_OUTBOUND','TRACK_RTL','SCAN_HOLD','MAP_DEGRADED_HOLD','GOAL_UNREACHABLE','PREFLIGHT_REJECT','COMPLETE'}))
        stateTimeoutTriggered=true;
        emergencyVerticalOnly=logical(est.degraded||est.rtlRequested);
        [state,stateEntryTime,emergencyLanding,failsafe,holdPosition,verticalStartZ,verticalStartTime]= ...
            begin_emergency_landing(t,est,'EMERGENCY_HOLD');
    end

    % State transitions can occur inside PLAN_* or timeout logic after the
    % switch case has already selected a reference. Re-apply the safe hold
    % reference for recovery states so no one-step ground/home command leaks
    % into the 6-DOF controller.
    if strcmp(state,'NAV_DEGRADED_HOLD')
        [brakeAccel,dampingEnabled]=xy_loss_horizontal_command( ...
            t,xyLossBrakeEndTime,xyLossVelocityTrustEnd,xyLossBrakeAccel);
        ref3=struct('p',[est.p(1:2);holdPosition(3)], ...
            'v',zeros(3,1),'a',[brakeAccel(:);0],'yaw',yawCommand, ...
            'horizontalControlEnabled',false, ...
            'horizontalVelocityDampingEnabled',dampingEnabled, ...
            'horizontalVelocityDampingGain',cfg.emergencyVelocityDampingGain, ...
            'horizontalFeedforwardAccelEnabled',true);
    elseif strcmp(state,'LIFECYCLE_REPLAN_BRAKE')
        ref3=struct('p',[est.p(1:2);cfg.altitudeNominal_m], ...
            'v',zeros(3,1),'a',zeros(3,1),'yaw',yawCommand);
    elseif strcmp(state,'MAP_DEGRADED_HOLD')
        ref3=hover_ref_yaw(est.p(1:2).',cfg.altitudeNominal_m,yawCommand);
    elseif strcmp(state,'SCAN_HOLD')
        scanYaw=wrap_pi_S2_2(scanStartYaw+cfg.mapScanYawRate_radps*(t-stateEntryTime));
        yawCommand=scanYaw;ref3=hover_ref_yaw(holdPosition(1:2).',cfg.altitudeNominal_m,yawCommand);
    elseif strcmp(state,'EMERGENCY_HOLD')
        if emergencyVerticalOnly
            [brakeAccel,dampingEnabled]=xy_loss_horizontal_command( ...
                t,xyLossBrakeEndTime,xyLossVelocityTrustEnd,xyLossBrakeAccel);
            ref3=struct('p',[est.p(1:2);holdPosition(3)], ...
                'v',zeros(3,1),'a',[brakeAccel(:);0],'yaw',yawCommand, ...
                'horizontalControlEnabled',false, ...
                'horizontalVelocityDampingEnabled',dampingEnabled, ...
                'horizontalVelocityDampingGain',cfg.emergencyVelocityDampingGain, ...
                'horizontalFeedforwardAccelEnabled',true);
        else
            ref3=struct('p',holdPosition(:),'v',zeros(3,1), ...
                'a',zeros(3,1),'yaw',yawCommand,'horizontalControlEnabled',true);
        end
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
    poseBuffer=append_pose_buffer_S2_3(poseBuffer,est,t,2.0);
    truthContext=struct('rtlObstacleActive',rtlObstacleInserted, ...
        'homeBlockActive',homeBlockInserted);
    [perceptionPacket,perceptionModel,truthWorld]=simulate_perception_packet_S2_3( ...
        cfg,scenario,truth,perceptionModel,t,k,truthContext);
    activeObstacles=truthWorld.staticRects5(:,1:4);
    if isempty(activeObstacleHistory)||~isequal(activeObstacleHistory{end},activeObstacles)
        activeObstacleHistory{end+1}=activeObstacles; %#ok<AGROW>
        activeObstacleHistoryTime(end+1)=t; %#ok<AGROW>
    end
    rawEstAcc=(est.v-oldEstV)/cfg.dt;
    estAcc=(1-cfg.estAccelerationFilterAlpha)*estAcc+cfg.estAccelerationFilterAlpha*rawEstAcc;
    hAcc=norm(estAcc(1:2));if hAcc>cfg.maxReplanStartAccel_mps2,estAcc(1:2)=estAcc(1:2)*(cfg.maxReplanStartAccel_mps2/hAcc);end
    [landDetected,landContactTimer]=land_detector_S2_2( ...
        cfg,packet,est,landContactTimer,landDetected,landingDetectionArmed);

    currentTruthJerk=(truth.a-previousTruthA)/cfg.dt;previousTruthA=truth.a;
    truthTakeoffReached=truthTakeoffReached|| ...
        (~truth.onGround&&truth.p(3)>=cfg.altitudeNominal_m-cfg.takeoffAltitudeTolerance_m);
    truthGoalReached=truthGoalReached|| ...
        (norm(truth.p(1:2).'-scenario.goal)<=1.5*cfg.goalTolerance_m&& ...
        abs(truth.p(3)-cfg.altitudeNominal_m)<=cfg.altitudeTolerance_m);
    truthLanded=truthLanded||(armedEver&&takeoffCompleted&&truth.onGround);
    horizontalTrackErr=norm(truth.p(1:2)-ref3.p(1:2));
    if any(strcmp(state,{'TRACK_OUTBOUND','TRACK_RTL'}))
        maxTrackingError=max(maxTrackingError,horizontalTrackErr);
    else
        % Safety/hold deviation is reported separately and does not include
        % vertical takeoff/landing motion, which has its own altitude gate.
        maxSafetyOverrideDeviation=max(maxSafetyOverrideDeviation,horizontalTrackErr);
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
    if captureFailsafeTriggerMetrics
        estimatorPositionErrorAtFailsafeTrigger=posEstErr;
        emergencyStartTruthXY=truth.p(1:2);
        captureFailsafeTriggerMetrics=false;
    end
    maxEstimatorPositionError=max(maxEstimatorPositionError,posEstErr);maxEstimatorAttitudeError=max(maxEstimatorAttitudeError,attEstErr);
    if est.degraded
        maxEstimatorPositionErrorPostLoss=max(maxEstimatorPositionErrorPostLoss,posEstErr);
    else
        maxEstimatorPositionErrorObservable=max(maxEstimatorPositionErrorObservable,posEstErr);
    end
    if all(isfinite(emergencyStartTruthXY))
        maxEmergencyHorizontalDrift=max(maxEmergencyHorizontalDrift, ...
            norm(truth.p(1:2)-emergencyStartTruthXY));
    end
    wallRaw=min([truth.p(1),cfg.room(1)-truth.p(1),truth.p(2),cfg.room(2)-truth.p(2)]);
    minWall=min(minWall,wallRaw);if truth.p(3)>cfg.groundHeight_m+0.05&&wallRaw<cfg.collisionRadius,geofenceViolationCount=geofenceViolationCount+1;end
    obsClear=inf;
    for j=1:size(truthWorld.staticRects5,1)
        dObs=dist_point_rect_S2_2(truth.p(1:2).',truthWorld.staticRects5(j,1:4));
        obsClear=min(obsClear,dObs);
        if truth.p(3)>cfg.groundHeight_m+0.05&&truth.p(3)<=truthWorld.staticRects5(j,5)&&dObs<cfg.collisionRadius,collisionCount=collisionCount+1;end
    end
    for j=1:numel(truthWorld.dynamic)
        dObs=norm(truth.p(1:2).'-truthWorld.dynamic(j).p)-truthWorld.dynamic(j).radius;
        obsClear=min(obsClear,dObs);minDynamicClearance=min(minDynamicClearance,dObs);
        if truth.p(3)>cfg.groundHeight_m+0.05&&dObs<cfg.collisionRadius,collisionCount=collisionCount+1;end
    end
    minObs=min(minObs,obsClear);
    if any(strcmp(state,{'TRACK_OUTBOUND','TRACK_RTL'}))
        ix=round(ref3.p(1)/grid.resolution)+1;iy=round(ref3.p(2)/grid.resolution)+1;
        if ix<1||ix>grid.nx||iy<1||iy>grid.ny||~grid.knownFree(iy,ix),unknownCommitmentCount=unknownCommitmentCount+1;end
    end

    store_sample();
    if any(strcmp(state,{'PREFLIGHT_REJECT','COMPLETE','FAILSAFE'})),break;end
end

valid=isfinite(T);T=T(valid);Ptrue=Ptrue(valid,:);Vtrue=Vtrue(valid,:);Atrue=Atrue(valid,:);Jtrue=Jtrue(valid,:);Qtrue=Qtrue(valid,:);RpyTrue=RpyTrue(valid,:);
Pest=Pest(valid,:);Vest=Vest(valid,:);Qest=Qest(valid,:);RpyEst=RpyEst(valid,:);Pref=Pref(valid,:);Vref=Vref(valid,:);Aref=Aref(valid,:);
trackingLog=trackingLog(valid);estErrorLog=estErrorLog(valid);attErrorLog=attErrorLog(valid);stateId=stateId(valid);laneId=laneId(valid);laneScores=laneScores(valid,:);laneEligible=laneEligible(valid,:);
inflationLog=inflationLog(valid);xySigmaLog=xySigmaLog(valid);thrustLog=thrustLog(valid);momentLog=momentLog(valid,:);sensorAidLog=sensorAidLog(valid,:);predMargin=predMargin(valid);
actualDyn=actualDyn(valid,:,:);estimatedDyn=estimatedDyn(valid,:,:);armedLog=armedLog(valid);landingTargetLog=landingTargetLog(valid,:);
degradedLog=degradedLog(valid);rtlRequestLog=rtlRequestLog(valid);
mapVersionLog=mapVersionLog(valid);knownFreeFractionLog=knownFreeFractionLog(valid);unknownFractionLog=unknownFractionLog(valid);
perceptionFreshLog=perceptionFreshLog(valid);segmentFinalLog=segmentFinalLog(valid);mapUpdateAcceptedLog=mapUpdateAcceptedLog(valid);

staticPass=(isempty(activeObstacles)||minObs>=cfg.baseInflationRadius-0.03)&&minWall>=cfg.baseInflationRadius-0.03;
referenceKinematicPass=maxReferenceSpeed<=cfg.maxSpeedXY_mps+1e-6&&maxReferenceAccel<=cfg.maxAccelXY_mps2+1e-6&&maxReferenceJerk<=cfg.maxJerkXY_mps3+1e-6&& ...
    maxVerticalReferenceSpeed<=cfg.maxVerticalReferenceSpeed_mps+1e-6&&maxVerticalReferenceAccel<=cfg.maxVerticalReferenceAccel_mps2+1e-6&&maxVerticalReferenceJerk<=cfg.maxVerticalReferenceJerk_mps3+1e-6;
executedKinematicPass=maxExecutedSpeed<=cfg.maxExecutedSpeed_mps&& ...
    maxExecutedAccel<=cfg.maxExecutedAccel_mps2&&maxExecutedJerk<=cfg.maxExecutedJerk_mps3&& ...
    maxExecutedVerticalSpeed<=cfg.maxExecutedVerticalSpeed_mps&& ...
    maxExecutedVerticalAccel<=cfg.maxExecutedVerticalAccel_mps2&& ...
    maxExecutedVerticalJerk<=cfg.maxExecutedVerticalJerk_mps3;
controllerPass=maxTrackingError<=cfg.maxPositionTrackingError_m&&maxAltitudeError<=cfg.maxAltitudeError_m&&rad2deg(maxTilt)<=cfg.maxAttitudeError_deg;
if scenario.expectedFailsafe
    % Once all horizontal aids are lost, absolute XY position is unobservable.
    % Validate estimator accuracy while observable and at the failsafe trigger;
    % retain the complete post-loss maximum as an explicit diagnostic.
    triggerMetric=estimatorPositionErrorAtFailsafeTrigger;
    if ~isfinite(triggerMetric),triggerMetric=maxEstimatorPositionErrorObservable;end
    estimatorFailsafeMetric=max(maxEstimatorPositionErrorObservable,triggerMetric);
    estimatorPositionPass=estimatorFailsafeMetric<=cfg.maxEstimatorFailsafeBound_m;
else
    estimatorFailsafeMetric=maxEstimatorPositionError;
    estimatorPositionPass=maxEstimatorPositionError<=cfg.maxEstimatorPositionError_m;
end
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
finalWorld=truth_world_S2_3(cfg,scenario,T(end),truthContext);
mapMetrics=validate_map_against_truth_S2_3(cfg,mapState,finalWorld);
minMapExtensions=double(scenario.expectedMapExtensions);
if isfield(scenario,'expectedMinMapExtensions')
    minMapExtensions=max(minMapExtensions,double(scenario.expectedMinMapExtensions));
end
minSafetyReplans=0;
if isfield(scenario,'expectedMinMapSafetyReplans')
    minSafetyReplans=double(scenario.expectedMinMapSafetyReplans);
end
minScanHolds=0;maxScanHolds=inf;
if isfield(scenario,'expectedMinScanHolds'),minScanHolds=double(scenario.expectedMinScanHolds);end
if isfield(scenario,'expectedMaxScanHolds'),maxScanHolds=double(scenario.expectedMaxScanHolds);end
mapExtensionPass=mapExtensionCount>=minMapExtensions;
mapReplanPass=~scenario.expectedMapReplan||(mapSafetyReplanCount>=1||replanCount>=1);
mapSafetyReplanPass=mapSafetyReplanCount>=minSafetyReplans;
scanHoldPass=scanHoldCount>=minScanHolds&&scanHoldCount<=maxScanHolds;
perceptionHoldPass=~scenario.expectedPerceptionHold||perceptionHoldCount>=1;
promotionPassS23=~scenario.expectedDynamicPromotion||double(mapState.promotedCount)>=1;
unreachablePass=(goalUnreachable==scenario.expectedGoalUnreachable);
truthIsolationPass=mapState.truthAccessCount==0&&unknownCommitmentCount==0;
mappingPass=mapMetrics.pass&&mapExtensionPass&&mapReplanPass&& ...
    mapSafetyReplanPass&&scanHoldPass&&perceptionHoldPass&& ...
    promotionPassS23&&unreachablePass&&truthIsolationPass;
pass=missionLifecyclePass&&trajectoryGate&&controllerGate&&estimatorGate&&continuityPass&&uncertaintyPass&&staticGate&&mappingPass&&~stateTimeoutTriggered;

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
    'promotionCount',double(mapState.promotedCount),'dynamicAvoidSteps',0,'dynamicHoldCount',0,'obstacleNoDataHoldCount',perceptionHoldCount,'rejoinCount',0, ...
    'replanBrakeCount',replanBrakeCount,'replanRetryCount',replanRetryCount,'inflationReplanCount',inflationReplanCount, ...
    'gridFallbackCount',gridFallbackCount,'gridFallbackUsed',gridFallbackCount>0, ...
    'navigationDegradedHoldCount',navigationDegradedHoldCount, ...
    'mapVersionFinal',double(mapState.version),'mapFrameVersionFinal',double(mapState.frameVersion), ...
    'mapAcceptedPackets',double(mapState.acceptedPackets),'mapRejectedPackets',double(mapState.rejectedPackets), ...
    'mapNoDataPackets',double(mapState.noDataPackets),'perceptionReplayCount',numel(perceptionReplay), ...
    'mapExtensionCount',mapExtensionCount,'mapExtensionPlanCount',mapExtensionPlanCount, ...
    'scanHoldCount',scanHoldCount,'mapSafetyReplanCount',mapSafetyReplanCount, ...
    'perceptionHoldCount',perceptionHoldCount,'goalUnreachable',goalUnreachable, ...
    'unknownCommitmentCount',unknownCommitmentCount,'mapPromotionCount',double(mapState.promotedCount), ...
    'mapFalseFreeRate',mapMetrics.falseFreeRate,'mapOccupiedRecall',mapMetrics.occupiedRecall, ...
    'mapObservedFraction',mapMetrics.observedFraction,'mapPass',mapMetrics.pass, ...
    'mapExtensionPass',mapExtensionPass,'mapReplanPass',mapReplanPass, ...
    'mapSafetyReplanPass',mapSafetyReplanPass,'scanHoldPass',scanHoldPass, ...
    'requiredMinMapExtensions',minMapExtensions,'requiredMinMapSafetyReplans',minSafetyReplans, ...
    'requiredMinScanHolds',minScanHolds,'requiredMaxScanHolds',maxScanHolds, ...
    'perceptionHoldPass',perceptionHoldPass,'mapPromotionPass',promotionPassS23, ...
    'goalUnreachablePass',unreachablePass,'truthIsolationPass',truthIsolationPass, ...
    'xyLossDetectionTime_s',xyLossStartTime,'xyLossVelocityTrustEnd_s',xyLossVelocityTrustEnd, ...
    'xyLossBrakeEndTime_s',xyLossBrakeEndTime,'xyLossBrakeAccel_mps2',xyLossBrakeAccel.', ...
    'xyLossBrakeReleaseTime_s',xyLossBrakeReleaseTime, ...
    'xyLossBrakeReleaseSpeed_mps',xyLossBrakeReleaseSpeed, ...
    'dstarInitialExpanded',0,'dstarRepairExpanded',dstarRepairExpanded,'astarScratchExpanded',astarScratchExpanded, ...
    'astarRecoveryCount',astarRecoveryCount,'timeToGoal_s',timeToGoal,'timeToLand_s',timeToLand,'timeToComplete_s',timeToComplete, ...
    'pathLength_m',pathLength,'minObstacleClearance_m',minObs,'minDynamicClearance_m',minDynamicClearance,'minWallClearance_m',minWall, ...
    'minPredictedMargin_m',inf,'maxExecutedSpeed_mps',maxExecutedSpeed,'maxExecutedAccel_mps2',maxExecutedAccel, ...
    'maxExecutedJerk_mps3',maxExecutedJerk,'maxExecutedVerticalSpeed_mps',maxExecutedVerticalSpeed, ...
    'maxExecutedVerticalAccel_mps2',maxExecutedVerticalAccel,'maxExecutedVerticalJerk_mps3',maxExecutedVerticalJerk, ...
    'maxReferenceSpeed_mps',maxReferenceSpeed,'maxReferenceAccel_mps2',maxReferenceAccel,'maxReferenceJerk_mps3',maxReferenceJerk, ...
    'maxVerticalReferenceSpeed_mps',maxVerticalReferenceSpeed,'maxVerticalReferenceAccel_mps2',maxVerticalReferenceAccel, ...
    'maxVerticalReferenceJerk_mps3',maxVerticalReferenceJerk,'maxTilt_deg',rad2deg(maxTilt),'maxAltitudeError_m',maxAltitudeError, ...
    'maxEstimatorPositionError_m',maxEstimatorPositionError, ...
    'maxEstimatorPositionErrorObservable_m',maxEstimatorPositionErrorObservable, ...
    'maxEstimatorPositionErrorPostLoss_m',maxEstimatorPositionErrorPostLoss, ...
    'estimatorPositionErrorAtFailsafeTrigger_m',estimatorPositionErrorAtFailsafeTrigger, ...
    'estimatorFailsafeMetric_m',estimatorFailsafeMetric, ...
    'maxEmergencyHorizontalDrift_m',maxEmergencyHorizontalDrift, ...
    'maxEstimatorAttitudeError_deg',rad2deg(maxEstimatorAttitudeError), ...
    'maxTrackingError_m',maxTrackingError,'maxSafetyOverrideDeviation_m',maxSafetyOverrideDeviation,'maxInflationRadius_m',maxInflation, ...
    'finalInflationRadius_m',currentInflation,'activeLaneFinal',est.activeLane,'laneSwitches',nav.selector.switchCount, ...
    'trajectoryGenerationCount',trajectoryGenerationCount,'trajectoryFallbackCount',trajectoryFallbackCount, ...
    'maxTrajectoryTimeScale',maxTrajectoryTimeScale,'maxReferenceContinuityJump',maxReferenceContinuity,'maxReplanStateJump',maxReplanStateJump, ...
    'staticPass',staticPass,'dynamicPass',true,'referenceKinematicPass',referenceKinematicPass, ...
    'executedKinematicPass',executedKinematicPass,'controllerPass',controllerPass,'estimatorPositionPass',estimatorPositionPass, ...
    'estimatorAttitudePass',estimatorAttitudePass,'continuityPass',continuityPass,'uncertaintyPass',uncertaintyPass, ...
    'missionOutcomePass',missionLifecyclePass,'failsafeExpectationPass',failsafePass,'replanEventPass',rtlReplanPass, ...
    'dynamicEventPass',true,'promotionEventPass',promotionPassS23,'noDataEventPass',perceptionHoldPass,'laneSwitchEventPass',true, ...
    'rtlEventPass',rtlRequestPass,'eventPass',missionLifecyclePass&&mappingPass,'preflightCheck',preflight, ...
    'finalTruthPosition',truth.p.','finalEstimatedPosition',est.p.', ...
    'finalDistanceToGoal_m',norm(truth.p(1:2)-scenario.goal(:)), ...
    'finalDistanceToLanding_m',norm(truth.p(1:2)-selectedLandingXY(:)), ...
    'finalState',state,'imuFaultDetectedTime_s',est.imuFaultDetectedTime, ...
    'faultSwitchTime_s',fault_switch_time(nav.switchLog), ...
    'faultBlendDuration_s',fault_blend_duration(nav.switchLog));

log=struct('t',T,'truthP',Ptrue,'truthV',Vtrue,'truthA',Atrue,'truthJ',Jtrue,'truthQ',Qtrue,'truthRpy',RpyTrue, ...
    'estP',Pest,'estV',Vest,'estQ',Qest,'estRpy',RpyEst,'pRef',Pref,'vRef',Vref,'aRef',Aref, ...
    'trackingError',trackingLog,'estimatorPositionError',estErrorLog,'estimatorAttitudeError_deg',attErrorLog, ...
    'stateId',stateId,'laneId',laneId,'laneScores',laneScores,'laneEligible',laneEligible,'inflationRadius',inflationLog, ...
    'xySigma',xySigmaLog,'thrust_N',thrustLog,'moment_Nm',momentLog,'sensorAids',sensorAidLog,'predictedMargin',predMargin, ...
    'actualDynamic',actualDyn,'estimatedDynamic',estimatedDyn,'pathHistory',{pathHistory},'trajectoryHistory',{trajectoryHistory}, ...
    'activeObstacleHistory',{activeObstacleHistory},'activeObstacleHistoryTime',activeObstacleHistoryTime, ...
    'inflationHistory',inflationHistory,'switchLog',{nav.switchLog}, ...
    'armed',armedLog,'landingTarget',landingTargetLog, ...
     'navigationDegraded',degradedLog,'rtlRequest',rtlRequestLog, ...
    'mapVersion',mapVersionLog,'knownFreeFraction',knownFreeFractionLog,'unknownFraction',unknownFractionLog, ...
    'perceptionFresh',perceptionFreshLog,'segmentIsFinal',segmentFinalLog,'mapUpdateAccepted',mapUpdateAcceptedLog, ...
    'mapVersionHistory',mapVersionHistory,'mapVersionHistoryTime',mapVersionHistoryTime, ...
    'mapSnapshots',{mapSnapshots},'mapSnapshotTimes',mapSnapshotTimes);
maps=struct('finalGrid',grid,'activeObstacles',activeObstacles,'planner',planner, ...
    'selectedLandingXY',selectedLandingXY,'home',scenario.home, ...
    'probabilisticMap',mapState,'truthWorldFinal',finalWorld, ...
    'perceptionReplaySchema','S2_3_RAW_RAYS_POSE_V1', ...
    'perceptionReplay',{perceptionReplay});

    function store_sample()
        T(k)=(k-1)*cfg.dt;Ptrue(k,:)=truth.p.';Vtrue(k,:)=truth.v.';Atrue(k,:)=truth.a.';Jtrue(k,:)=currentTruthJerk.';
        Qtrue(k,:)=truth.q;RpyTrue(k,:)=q2rpy_S2_2(truth.q);Pest(k,:)=est.p.';Vest(k,:)=est.v.';Qest(k,:)=est.q;RpyEst(k,:)=q2rpy_S2_2(est.q);
        Pref(k,:)=ref3.p.';Vref(k,:)=ref3.v.';Aref(k,:)=ref3.a.';trackingLog(k)=norm(truth.p-ref3.p);estErrorLog(k)=norm(est.p-truth.p);
        attErrorLog(k)=rad2deg(norm(qlog_S2_2(qmul_S2_2(qconj_S2_2(est.q),truth.q))));stateId(k)=lifecycle_state_id(state);laneId(k)=est.activeLane;
        if ~isempty(est.scores),laneScores(k,:)=est.scores(:).';end;if ~isempty(est.eligible),laneEligible(k,:)=est.eligible(:).';end
        inflationLog(k)=currentInflation;xySigmaLog(k)=est.xySigma_m;thrustLog(k)=cmd.thrust_N;momentLog(k,:)=cmd.moment_Nm(:).';
        sensorAidLog(k,:)=[packet.hasVio packet.hasLidar packet.hasRange packet.hasBaro];predMargin(k)=stepMargin;armedLog(k)=armed;landingTargetLog(k,:)=selectedLandingXY;
        degradedLog(k)=est.degraded;rtlRequestLog(k)=est.rtlRequested;
        mapVersionLog(k)=double(mapState.version);knownFreeFractionLog(k)=nnz(grid.knownFree)/numel(grid.knownFree);
        unknownFractionLog(k)=nnz(grid.unknown)/numel(grid.unknown);
        perceptionFreshLog(k)=isfinite(max(mapState.lastLidarTime,mapState.lastDepthTime))&& ...
            T(k)-max(mapState.lastLidarTime,mapState.lastDepthTime)<=cfg.mapPerceptionHoldTimeout_s;
        segmentFinalLog(k)=segmentIsFinal;mapUpdateAcceptedLog(k)=lastMapUpdate.accepted;
        if isfield(truthWorld,'dynamic')
            for jd=1:numel(truthWorld.dynamic)
                slot=truthWorld.dynamic(jd).id;
                if slot>=1&&slot<=nDynamicLog
                    actualDyn(k,slot,:)=reshape(truthWorld.dynamic(jd).p,1,1,2);
                end
            end
        end
    end
end

function [aBrake,tEnd]=make_xy_loss_brake(cfg,vReliable,t)
vReliable=vReliable(:);speed=norm(vReliable);
if speed<1e-6
    aBrake=zeros(2,1);tEnd=t;
    return;
end
duration=speed/max(cfg.xyLossOpenLoopBrakeAccel_mps2,eps)+ ...
    cfg.xyLossBrakeRampAllowance_s;
duration=max(cfg.xyLossBrakeMinTime_s,min(cfg.xyLossBrakeMaxTime_s,duration));
aBrake=-vReliable/duration;
mag=norm(aBrake);
if mag>cfg.xyLossOpenLoopBrakeAccel_mps2
    aBrake=aBrake*(cfg.xyLossOpenLoopBrakeAccel_mps2/mag);
end
tEnd=t+duration;
end

function a=active_xy_loss_brake(t,tEnd,aBrake)
if isfinite(tEnd)&&t<=tEnd,a=aBrake(:);else,a=zeros(2,1);end
end

function [a,dampingEnabled]=xy_loss_horizontal_command(t,tBrakeEnd,~,aBrake)
% Position-loss arrest command. The impulse is constructed only from the
% last aid-bounded velocity. Post-loss ESKF velocity damping is deliberately
% disabled because an unobservable velocity can reverse the brake and drive
% the vehicle during the blind descent.
a=active_xy_loss_brake(t,tBrakeEnd,aBrake);
dampingEnabled=false;
end

function [vTarget,index]=grid_fallback_target(cfg,path,index,p,v)
% Conservative stop-at-corner follower for a verified grid route.
vTarget=zeros(2,1);
if isempty(path)||size(path,1)<2,return;end
index=max(2,min(index,size(path,1)));
while index<size(path,1)&&norm(p(:).'-path(index,:))<=cfg.gridFallbackWaypointTolerance_m
    index=index+1;
end
delta=path(index,:).'-p(:);distance=norm(delta);
if distance<1e-9,return;end
direction=delta/distance;
stopSpeed=sqrt(max(0,2*cfg.gridFallbackAccelLimit_mps2*distance));
speed=min([cfg.gridFallbackSpeed_mps,stopSpeed]);
if index<size(path,1)&&distance<=2*cfg.gridFallbackWaypointTolerance_m
    speed=min(speed,0.5*cfg.gridFallbackSpeed_mps);
end
vTarget=speed*direction;
% Avoid commanding deeper into a waypoint already passed by inertia.
if dot(v(:),direction)<-0.05,vTarget=zeros(2,1);end
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
traj=generate_strict_trajectory_S2_3(cfg,grid,path,est.v(1:2),estAcc(1:2),initialScale);jump=inf(1,3);
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
function ref=hover_ref_yaw(xy,z,yaw),ref=struct('p',[xy(:);z],'v',zeros(3,1),'a',zeros(3,1),'yaw',yaw);end
function ref=trajectory_start_ref(traj,z,yaw)
r=sample_min_snap_state_S2_2(traj,0);
ref=struct('p',[r.p(:);z],'v',[r.v(:);0],'a',[r.a(:);0],'yaw',yaw);
end
function ref=ground_ref(xy,cfg),ref=hover_ref(xy,cfg.groundHeight_m);end
function cmd=zero_cmd(),cmd=struct('thrust_N',0,'moment_Nm',zeros(3,1),'aCmd',zeros(3,1));end
function tr=invalid_traj(),tr=struct('valid',false,'fallbackUsed',false,'timeScale',0,'continuity',zeros(1,4),'maxSpeed_mps',0,'maxAccel_mps2',0,'maxJerk_mps3',0);end
function [state,entry,emergency,failsafe,hold,z0,t0]=begin_emergency_landing(t,est,newState)
state=newState;entry=t;emergency=true;failsafe=true;hold=est.p(:);
hold(3)=max(est.p(3),0.03);z0=hold(3);t0=t;
end
function t=fault_switch_time(switchLog)
t=nan;
for i=1:numel(switchLog)
    if isfield(switchLog(i),'faultAware')&&switchLog(i).faultAware
        t=switchLog(i).time;
        return;
    end
end
end

function d=fault_blend_duration(switchLog)
d=nan;
for i=1:numel(switchLog)
    if isfield(switchLog(i),'faultAware')&&switchLog(i).faultAware
        d=switchLog(i).blendDuration_s;
        return;
    end
end
end

function yaw=estimated_yaw_S2_3(est)
rpy=q2rpy_S2_2(est.q);yaw=rpy(3);
end

function id=lifecycle_state_id(s)
switch s
    case 'PREFLIGHT',id=1;case 'ARM',id=2;case 'TAKEOFF',id=3;case 'INITIAL_HOVER',id=4;
    case 'WAIT_FOR_GOAL',id=5;case 'PLAN_OUTBOUND',id=6;case 'TRACK_OUTBOUND',id=7;case 'GOAL_HOVER',id=8;
    case 'PLAN_RTL',id=9;case 'TRACK_RTL',id=10;case 'LAND_HOVER',id=11;case 'LAND_DESCENT',id=12;
    case 'DISARM',id=13;case 'COMPLETE',id=14;case 'PREFLIGHT_REJECT',id=15;case 'EMERGENCY_HOLD',id=16;
    case 'EMERGENCY_LAND',id=17;case 'LIFECYCLE_REPLAN_BRAKE',id=18;case 'FAILSAFE',id=19;
    case 'NAV_DEGRADED_HOLD',id=20;case 'SCAN_HOLD',id=21;case 'MAP_DEGRADED_HOLD',id=22;case 'GOAL_UNREACHABLE',id=23;otherwise,id=0;
end
end

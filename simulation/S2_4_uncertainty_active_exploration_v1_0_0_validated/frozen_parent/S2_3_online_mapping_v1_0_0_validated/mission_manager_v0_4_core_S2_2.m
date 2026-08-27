function [log,summary,maps] = mission_manager_v0_4_core_S2_2(cfg,scenario)
% MISSION_MANAGER_V0_4_CORE_S2_2 Frozen MATLAB-validated v0.4 regression core.
% Planning and control use the selected local ESKF state. Collision and
% geofence validation use the independent 6-DOF truth state.

activeObstacles=scenario.knownObstacles;
truth=init_quadrotor_state_S2_2(cfg,scenario.start);
sensorModel=init_sensor_model_S2_2();
[packet,sensorModel]=simulate_sensor_packet_S2_2(cfg,scenario,truth,sensorModel,0,1);
[nav,est]=multi_lane_eskf_S2_2('init',cfg,packet,0);
estAcc=zeros(3,1);previousEstV=est.v;previousTruthA=truth.a;
previousCmdAccel=zeros(3,1);safetyVelCmd=est.v(1:2);safetyAccelCmd=zeros(2,1);
currentInflation=uncertainty_inflation_S2_2(cfg,est.xySigma_m);
cfgGrid=grid_cfg(cfg,currentInflation);
grid=build_occupancy_grid_S2_2(cfgGrid,activeObstacles);
[planner,rawPath,dstarInit]=dstar_lite_S2_2('init',grid,est.p(1:2).',scenario.goal);
path=prepare_path(grid,est.p(1:2).',rawPath);
traj=generate_min_snap_trajectory_S2_2(cfg,grid,path,est.v(1:2),estAcc(1:2),scenario.trajectoryInitialTimeScale);
trajClock=0;
if isempty(path)||~traj.valid,state='FAILSAFE';else,state='TRACK';end
holdPosition=est.p;

insertedDone=false(1,numel(scenario.insertedObstacles));
nDyn=numel(scenario.dynamicObstacles);tracks=cell(1,nDyn);stopTimers=zeros(1,nDyn);promoted=false(1,nDyn);
replanCount=0;promotionCount=0;dynamicAvoidSteps=0;dynamicHoldCount=0;obstacleNoDataHoldCount=0;
inflationReplanCount=0;dstarRepairExpanded=0;astarScratchExpanded=0;astarRecoveryCount=0;
collisionCount=0;geofenceViolationCount=0;goalReached=false;failsafe=strcmp(state,'FAILSAFE');rtlRequested=false;
timeToGoal=nan;obstacleNoDataElapsed=0;clearTimer=0;wasAvoiding=false;rejoinCount=0;stallTimer=0;
pendingReplan=false;pendingReplanStart=nan;lastReplanAttempt=-inf;
replanBrakeCount=0;replanRetryCount=0;
pathLength=0;minWall=inf;minObs=inf;minDynamicClearance=inf;minPredictedMargin=inf;
maxExecutedSpeed=0;maxExecutedAccel=0;maxExecutedJerk=0;maxTilt=0;maxAltitudeError=0;
maxEstimatorPositionError=0;maxEstimatorAttitudeError=0;maxTrackingError=0;maxSafetyOverrideDeviation=0;
maxInflation=currentInflation;trajectoryGenerationCount=double(traj.valid);trajectoryFallbackCount=double(traj.valid&&traj.fallbackUsed);
maxTrajectoryTimeScale=finite_or_zero(traj.timeScale);maxReferenceContinuity=zeros(1,4);maxReplanStateJump=zeros(1,3);
maxReferenceSpeed=0;maxReferenceAccel=0;maxReferenceJerk=0;
if traj.valid
    maxReferenceContinuity=max(maxReferenceContinuity,traj.continuity);
    maxReferenceSpeed=max(maxReferenceSpeed,traj.maxSpeed_mps);maxReferenceAccel=max(maxReferenceAccel,traj.maxAccel_mps2);maxReferenceJerk=max(maxReferenceJerk,traj.maxJerk_mps3);
end

maxSteps=ceil(cfg.maxTime_s/cfg.dt)+1;
T=nan(maxSteps,1);Ptrue=nan(maxSteps,3);Vtrue=nan(maxSteps,3);Atrue=nan(maxSteps,3);Jtrue=nan(maxSteps,3);
Qtrue=nan(maxSteps,4);RpyTrue=nan(maxSteps,3);Pest=nan(maxSteps,3);Vest=nan(maxSteps,3);Qest=nan(maxSteps,4);RpyEst=nan(maxSteps,3);
Pref=nan(maxSteps,3);Vref=nan(maxSteps,3);Aref=nan(maxSteps,3);trackingLog=nan(maxSteps,1);estErrorLog=nan(maxSteps,1);
attErrorLog=nan(maxSteps,1);stateId=nan(maxSteps,1);laneId=nan(maxSteps,1);laneScores=nan(maxSteps,4);laneEligible=false(maxSteps,4);
inflationLog=nan(maxSteps,1);xySigmaLog=nan(maxSteps,1);thrustLog=nan(maxSteps,1);momentLog=nan(maxSteps,3);
sensorAidLog=false(maxSteps,4);predMargin=nan(maxSteps,1);actualDyn=nan(maxSteps,max(1,nDyn),2);estimatedDyn=nan(maxSteps,max(1,nDyn),2);
pathHistory={path};trajectoryHistory={traj};activeObstacleHistory={activeObstacles};inflationHistory=currentInflation;

k=1;ref3=hold_ref(est,cfg);cmd=struct('thrust_N',cfg.mass_kg*norm(cfg.gW),'moment_Nm',zeros(3,1));stepMargin=inf;currentTruthJerk=zeros(3,1);
store_sample();

for k=2:maxSteps
    t=(k-1)*cfg.dt;
    mapChanged=false;changedMask=false(grid.ny,grid.nx);

    % New static obstacles.
    for i=1:numel(scenario.insertedObstacles)
        if ~insertedDone(i)&&t>=scenario.insertedObstacles(i).time
            oldOcc=grid.occ;activeObstacles=[activeObstacles;scenario.insertedObstacles(i).rect]; %#ok<AGROW>
            grid=build_occupancy_grid_S2_2(grid_cfg(cfg,currentInflation),activeObstacles);
            changedMask=changedMask|xor(oldOcc,grid.occ);insertedDone(i)=true;mapChanged=true;
            activeObstacleHistory{end+1}=activeObstacles; %#ok<AGROW>
        end
    end

    % Dynamic obstacle sensing and static promotion.
    obstacleCoverageOK=~inside_windows(t,scenario.obstacleSensorDropoutWindows);
    sensedDyn=struct('p',{},'v',{},'radius',{},'id',{});actualStates=struct('p',{},'v',{},'radius',{},'id',{});
    for i=1:nDyn
        [pTrueDyn,vTrueDyn,active]=dynamic_obstacle_state_S2_2(scenario.dynamicObstacles(i),t);
        if ~active
            if ~promoted(i),tracks{i}=[];stopTimers(i)=0;end
            continue;
        end
        actualStates(end+1)=struct('p',pTrueDyn,'v',vTrueDyn,'radius',scenario.dynamicObstacles(i).radius,'id',i); %#ok<AGROW>
        actualDyn(k,i,:)=pTrueDyn;
        if ~obstacleCoverageOK,continue;end
        z=pTrueDyn+cfg.dynamicSensorNoiseSigma*randn(1,2);
        tracks{i}=alpha_beta_track_S2_2(cfg,tracks{i},z,t);estimatedDyn(k,i,:)=tracks{i}.p;
        if tracks{i}.updates>=3&&tracks{i}.speedFiltered<=cfg.stoppedSpeedThreshold_mps
            stopTimers(i)=stopTimers(i)+cfg.dt;
        else
            stopTimers(i)=0;
        end
        if stopTimers(i)>=cfg.stoppedPersistence_s&&~promoted(i)
            r=scenario.dynamicObstacles(i).radius;rect=[tracks{i}.p(1)-r,tracks{i}.p(2)-r,2*r,2*r];oldOcc=grid.occ;
            activeObstacles=[activeObstacles;rect];grid=build_occupancy_grid_S2_2(grid_cfg(cfg,currentInflation),activeObstacles); %#ok<AGROW>
            changedMask=changedMask|xor(oldOcc,grid.occ);promoted(i)=true;promotionCount=promotionCount+1;mapChanged=true;
            activeObstacleHistory{end+1}=activeObstacles; %#ok<AGROW>
        end
        if ~promoted(i)
            sensedDyn(end+1)=struct('p',tracks{i}.p,'v',tracks{i}.vFiltered, ...
                'radius',scenario.dynamicObstacles(i).radius,'id',i); %#ok<AGROW>
        end
    end

    % Uncertainty-aware inflation: increase only, to avoid unsafe route
    % shrinkage without a deliberate replan.
    proposedInflation=uncertainty_inflation_S2_2(cfg,est.xySigma_m);
    if proposedInflation>currentInflation+cfg.inflationReplanThreshold_m
        oldOcc=grid.occ;currentInflation=proposedInflation;
        grid=build_occupancy_grid_S2_2(grid_cfg(cfg,currentInflation),activeObstacles);
        changedMask=changedMask|xor(oldOcc,grid.occ);mapChanged=true;inflationReplanCount=inflationReplanCount+1;
        inflationHistory(end+1)=currentInflation; %#ok<AGROW>
    end
    maxInflation=max(maxInflation,currentInflation);

    if mapChanged
        [planner,rawPath,st]=dstar_lite_S2_2('repair',planner,grid,est.p(1:2).',changedMask);
        dstarRepairExpanded=dstarRepairExpanded+st.expanded;
        [rawA,aInfo]=astar_grid_S2_2(grid,est.p(1:2).',scenario.goal);
        astarScratchExpanded=astarScratchExpanded+aInfo.expanded;

        candidatePath=prepare_path(grid,est.p(1:2).',rawPath);
        [candidateTraj,candidateClock,jump]=new_trajectory( ...
            cfg,grid,candidatePath,est,estAcc,1.0);

        if (isempty(candidatePath)||~candidateTraj.valid)&&~isempty(rawA)
            [planner,~,~]=dstar_lite_S2_2( ...
                'init',grid,est.p(1:2).',scenario.goal);
            candidatePath=prepare_path(grid,est.p(1:2).',rawA);
            [candidateTraj,candidateClock,jump]=new_trajectory( ...
                cfg,grid,candidatePath,est,estAcc,1.0);
            astarRecoveryCount=astarRecoveryCount+1;
        end

        replanCount=replanCount+1;stallTimer=0;
        pathHistory{end+1}=candidatePath; %#ok<AGROW>
        trajectoryHistory{end+1}=candidateTraj; %#ok<AGROW>

        if ~isempty(candidatePath)&&candidateTraj.valid
            path=candidatePath;traj=candidateTraj;trajClock=candidateClock;
            pendingReplan=false;pendingReplanStart=nan;
            [trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump,maxReferenceSpeed,maxReferenceAccel,maxReferenceJerk]= ...
                update_traj_metrics(traj,jump,trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump,maxReferenceSpeed,maxReferenceAccel,maxReferenceJerk);
        elseif ~isempty(candidatePath)||~isempty(rawA)||~isempty(rawPath)
            % A route exists, but the current moving start state cannot be
            % joined to it without entering the inflated map. Brake in place
            % first, then regenerate from a near-hover state. Do not convert
            % a recoverable kinodynamic mismatch into an immediate failsafe.
            pendingReplan=true;pendingReplanStart=t;lastReplanAttempt=t;
            replanBrakeCount=replanBrakeCount+1;state='REPLAN_BRAKE';
            holdPosition=est.p;wasAvoiding=true;clearTimer=0;
            safetyVelCmd=est.v(1:2);safetyAccelCmd=zeros(2,1);
        else
            state='FAILSAFE';failsafe=true;holdPosition=est.p;
        end
    end

    % Complete a deferred map replan only after the vehicle has braked to a
    % near-hover state. The route is refreshed from the current continuous
    % estimate because the vehicle moves slightly during braking.
    if pendingReplan&&~failsafe&& ...
            norm(est.v(1:2))<=cfg.replanBrakeSpeed_mps&& ...
            norm(estAcc(1:2))<=cfg.replanBrakeAccel_mps2&& ...
            t-lastReplanAttempt>=cfg.replanRetryPeriod_s
        lastReplanAttempt=t;replanRetryCount=replanRetryCount+1;
        [planner,rawPath,st]=dstar_lite_S2_2( ...
            'refresh',planner,grid,est.p(1:2).');
        dstarRepairExpanded=dstarRepairExpanded+st.expanded;
        [rawA,aInfo]=astar_grid_S2_2(grid,est.p(1:2).',scenario.goal);
        astarScratchExpanded=astarScratchExpanded+aInfo.expanded;
        candidatePath=prepare_path(grid,est.p(1:2).',rawPath);
        [candidateTraj,candidateClock,jump]=new_trajectory( ...
            cfg,grid,candidatePath,est,estAcc,1.0);
        if (isempty(candidatePath)||~candidateTraj.valid)&&~isempty(rawA)
            [planner,~,~]=dstar_lite_S2_2( ...
                'init',grid,est.p(1:2).',scenario.goal);
            candidatePath=prepare_path(grid,est.p(1:2).',rawA);
            [candidateTraj,candidateClock,jump]=new_trajectory( ...
                cfg,grid,candidatePath,est,estAcc,1.0);
            astarRecoveryCount=astarRecoveryCount+1;
        end
        pathHistory{end+1}=candidatePath; %#ok<AGROW>
        trajectoryHistory{end+1}=candidateTraj; %#ok<AGROW>
        if ~isempty(candidatePath)&&candidateTraj.valid
            path=candidatePath;traj=candidateTraj;trajClock=candidateClock;
            pendingReplan=false;pendingReplanStart=nan;
            state='REJOIN';rejoinCount=rejoinCount+1;wasAvoiding=true;
            safetyVelCmd=est.v(1:2);safetyAccelCmd=zeros(2,1);
            [trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump,maxReferenceSpeed,maxReferenceAccel,maxReferenceJerk]= ...
                update_traj_metrics(traj,jump,trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump,maxReferenceSpeed,maxReferenceAccel,maxReferenceJerk);
        end
    end
    if pendingReplan&&t-pendingReplanStart>=cfg.replanBrakeTimeout_s
        state='FAILSAFE';failsafe=true;holdPosition=est.p;
    end

    desiredVel=zeros(2,1);modified=false;stepMargin=inf;trackingError=nan;refXY=[];previousState=state;
    if est.rtlRequested
        rtlRequested=true;state='FAILSAFE';failsafe=true;holdPosition=est.p;
    elseif strcmp(state,'FAILSAFE')
        failsafe=true;
    elseif norm(est.p(1:2).'-scenario.goal)<=cfg.goalTolerance_m&& ...
            norm(truth.p(1:2).'-scenario.goal)<=1.5*cfg.goalTolerance_m&& ...
            abs(truth.p(3)-cfg.altitudeNominal_m)<=cfg.altitudeTolerance_m
        goalReached=true;timeToGoal=t;holdPosition=est.p;state='COMPLETE';
    elseif pendingReplan||strcmp(state,'REPLAN_BRAKE')
        state='REPLAN_BRAKE';
        [safetyVelCmd,safetyAccelCmd]=shape_velocity_command_S2_2( ...
            cfg,safetyVelCmd,safetyAccelCmd,zeros(2,1));
        desiredVel=safetyVelCmd;
    elseif ~obstacleCoverageOK
        obstacleNoDataElapsed=obstacleNoDataElapsed+cfg.dt;
        if obstacleNoDataElapsed>=cfg.obstacleNoDataStopTimeout_s
            if ~strcmp(state,'OBSTACLE_NO_DATA_HOLD'),obstacleNoDataHoldCount=obstacleNoDataHoldCount+1;holdPosition=est.p;end
            state='OBSTACLE_NO_DATA_HOLD';
        end
        if obstacleNoDataElapsed>=cfg.obstacleNoDataFailsafeTimeout_s,state='FAILSAFE';failsafe=true;holdPosition=est.p;end
    else
        if strcmp(state,'OBSTACLE_NO_DATA_HOLD')
            % Obstacle-sensor coverage recovery does not change the static
            % map. Resume the paused, already validated trajectory through a
            % controlled rejoin instead of rebuilding a trajectory from a
            % noisy instantaneous acceleration estimate.
            state='REJOIN';rejoinCount=rejoinCount+1;wasAvoiding=true;clearTimer=0;
            safetyVelCmd=est.v(1:2);safetyAccelCmd=zeros(2,1);
        end
        obstacleNoDataElapsed=0;
        if ~strcmp(state,'FAILSAFE')&&~strcmp(state,'COMPLETE')
            [desiredVel,trackingError,refXY]=track_smooth_trajectory_S2_2(cfg,est.p(1:2),est.v(1:2),traj,trajClock);
            if norm(desiredVel)>1e-9
                direction=desiredVel/norm(desiredVel);vLimit=static_braking_speed_S2_2(cfg,est.p(1:2),direction,grid);
                if norm(desiredVel)>vLimit,desiredVel=desiredVel*(vLimit/norm(desiredVel));end
            end
            [safeVel,modified,stepMargin]=velocity_obstacle_filter_S2_2(cfg,est.p(1:2),desiredVel,sensedDyn,grid);minPredictedMargin=min(minPredictedMargin,stepMargin);
            if modified
                dynamicAvoidSteps=dynamicAvoidSteps+1;clearTimer=0;
                if ~wasAvoiding
                    safetyVelCmd=est.v(1:2);safetyAccelCmd=zeros(2,1);
                end
                wasAvoiding=true;
                [safetyVelCmd,safetyAccelCmd]=shape_velocity_command_S2_2( ...
                    cfg,safetyVelCmd,safetyAccelCmd,safeVel);
                desiredVel=safetyVelCmd;
                if norm(safeVel)<1e-6
                    if ~strcmp(state,'DYNAMIC_HOLD'),dynamicHoldCount=dynamicHoldCount+1;holdPosition=est.p;end
                    state='DYNAMIC_HOLD';
                else
                    state='DYNAMIC_AVOID';
                end
            elseif strcmp(state,'DYNAMIC_HOLD')
                clearTimer=clearTimer+cfg.dt;
                [safetyVelCmd,safetyAccelCmd]=shape_velocity_command_S2_2( ...
                    cfg,safetyVelCmd,safetyAccelCmd,zeros(2,1));
                desiredVel=safetyVelCmd;
                if clearTimer>=cfg.holdClearTime_s
                    state='REJOIN';rejoinCount=rejoinCount+1;
                end
            elseif wasAvoiding||strcmp(state,'REJOIN')
                state='REJOIN';
                [safetyVelCmd,safetyAccelCmd]=shape_velocity_command_S2_2( ...
                    cfg,safetyVelCmd,safetyAccelCmd,desiredVel);
                desiredVel=safetyVelCmd;
                velocityLocked=isempty(refXY)|| ...
                    norm(est.v(1:2)-refXY.v(:))<=cfg.rejoinVelocityTolerance_mps;
                if isfinite(trackingError)&& ...
                        trackingError<=cfg.rejoinTolerance_m&&velocityLocked
                    state='TRACK';wasAvoiding=false;clearTimer=0;
                    safetyVelCmd=est.v(1:2);safetyAccelCmd=zeros(2,1);
                end
            else
                state='TRACK';
            end
            if norm(desiredVel)<1e-8&&norm(est.p(1:2).'-scenario.goal)>cfg.goalTolerance_m,stallTimer=stallTimer+cfg.dt;else,stallTimer=0;end
        end
    end

    if ~strcmp(previousState,state)&&any(strcmp(state, ...
            {'DYNAMIC_HOLD','OBSTACLE_NO_DATA_HOLD','REPLAN_BRAKE','FAILSAFE'}))
        holdPosition=est.p;
    end
    ref3=make_controller_reference(cfg,state,est,refXY,desiredVel, ...
        safetyAccelCmd,holdPosition);
    cmd=geometric_controller_S2_2(cfg,est,ref3,previousCmdAccel);
    previousCmdAccel=cmd.aCmd;
    oldTruthP=truth.p;truth=quadrotor_dynamics_S2_2(cfg,truth,cmd);pathLength=pathLength+norm(truth.p-oldTruthP);

    [packet,sensorModel]=simulate_sensor_packet_S2_2(cfg,scenario,truth,sensorModel,t,k);
    oldEstV=est.v;[nav,est]=multi_lane_eskf_S2_2('step',nav,cfg,packet,t,cfg.dt);
    rawEstAcc=(est.v-oldEstV)/cfg.dt;
    estAcc=(1-cfg.estAccelerationFilterAlpha)*estAcc+ ...
        cfg.estAccelerationFilterAlpha*rawEstAcc;
    hAcc=norm(estAcc(1:2));
    if hAcc>cfg.maxReplanStartAccel_mps2
        estAcc(1:2)=estAcc(1:2)*(cfg.maxReplanStartAccel_mps2/hAcc);
    end
    previousEstV=est.v; %#ok<NASGU>

    if strcmp(state,'TRACK')&&~modified&&traj.valid&&isfinite(trackingError)&&trackingError<=cfg.trajectoryClockMaxError_m
        trajClock=min(traj.duration_s,trajClock+cfg.dt);
    end

    % Truth-based safety and performance metrics.
    currentTruthJerk=(truth.a-previousTruthA)/cfg.dt;previousTruthA=truth.a;
    truthTrackErr=norm(truth.p-ref3.p);
    if strcmp(state,'TRACK'),maxTrackingError=max(maxTrackingError,truthTrackErr);else,maxSafetyOverrideDeviation=max(maxSafetyOverrideDeviation,truthTrackErr);end
    maxExecutedSpeed=max(maxExecutedSpeed,norm(truth.v(1:2)));maxExecutedAccel=max(maxExecutedAccel,norm(truth.a(1:2)));maxExecutedJerk=max(maxExecutedJerk,norm(currentTruthJerk(1:2)));
    rpyTruth=q2rpy_S2_2(truth.q);rpyEst=q2rpy_S2_2(est.q);maxTilt=max(maxTilt,norm(rpyTruth(1:2)));maxAltitudeError=max(maxAltitudeError,abs(truth.p(3)-cfg.altitudeNominal_m));
    posEstErr=norm(est.p-truth.p);attEstErr=norm(qlog_S2_2(qmul_S2_2(qconj_S2_2(est.q),truth.q)));maxEstimatorPositionError=max(maxEstimatorPositionError,posEstErr);maxEstimatorAttitudeError=max(maxEstimatorAttitudeError,attEstErr);
    wallRaw=min([truth.p(1),cfg.room(1)-truth.p(1),truth.p(2),cfg.room(2)-truth.p(2)]);minWall=min(minWall,wallRaw);if wallRaw<cfg.collisionRadius,geofenceViolationCount=geofenceViolationCount+1;end
    obsClear=inf;for j=1:size(activeObstacles,1),dObs=dist_point_rect_S2_2(truth.p(1:2).',activeObstacles(j,:));obsClear=min(obsClear,dObs);if dObs<cfg.collisionRadius,collisionCount=collisionCount+1;end,end;minObs=min(minObs,obsClear);
    for j=1:numel(actualStates),d=norm(truth.p(1:2).'-actualStates(j).p)-cfg.collisionRadius-actualStates(j).radius;minDynamicClearance=min(minDynamicClearance,d);if d<0,collisionCount=collisionCount+1;end,end

    store_sample();
    if strcmp(state,'FAILSAFE')||strcmp(state,'COMPLETE'),break;end
end

valid=isfinite(T);fields={'T','Ptrue','Vtrue','Atrue','Jtrue','Qtrue','RpyTrue','Pest','Vest','Qest','RpyEst','Pref','Vref','Aref','trackingLog','estErrorLog','attErrorLog','stateId','laneId','laneScores','laneEligible','inflationLog','xySigmaLog','thrustLog','momentLog','sensorAidLog','predMargin','actualDyn','estimatedDyn'}; %#ok<NASGU>
T=T(valid);Ptrue=Ptrue(valid,:);Vtrue=Vtrue(valid,:);Atrue=Atrue(valid,:);Jtrue=Jtrue(valid,:);Qtrue=Qtrue(valid,:);RpyTrue=RpyTrue(valid,:);
Pest=Pest(valid,:);Vest=Vest(valid,:);Qest=Qest(valid,:);RpyEst=RpyEst(valid,:);Pref=Pref(valid,:);Vref=Vref(valid,:);Aref=Aref(valid,:);
trackingLog=trackingLog(valid);estErrorLog=estErrorLog(valid);attErrorLog=attErrorLog(valid);stateId=stateId(valid);laneId=laneId(valid);laneScores=laneScores(valid,:);laneEligible=laneEligible(valid,:);
inflationLog=inflationLog(valid);xySigmaLog=xySigmaLog(valid);thrustLog=thrustLog(valid);momentLog=momentLog(valid,:);sensorAidLog=sensorAidLog(valid,:);predMargin=predMargin(valid);actualDyn=actualDyn(valid,:,:);estimatedDyn=estimatedDyn(valid,:,:);

staticPass=(isempty(activeObstacles)||minObs>=cfg.baseInflationRadius-0.03)&&minWall>=cfg.baseInflationRadius-0.03;
dynamicPass=~isfinite(minDynamicClearance)||minDynamicClearance>=cfg.minDynamicPhysicalClearance_m;
referenceKinematicPass=maxReferenceSpeed<=cfg.maxSpeedXY_mps+1e-6&&maxReferenceAccel<=cfg.maxAccelXY_mps2+1e-6&&maxReferenceJerk<=cfg.maxJerkXY_mps3+1e-6;
executedKinematicPass=maxExecutedSpeed<=cfg.maxExecutedSpeed_mps&&maxExecutedAccel<=cfg.maxExecutedAccel_mps2&&maxExecutedJerk<=cfg.maxExecutedJerk_mps3;
controllerPass=maxTrackingError<=cfg.maxPositionTrackingError_m&&maxAltitudeError<=cfg.maxAltitudeError_m&&rad2deg(maxTilt)<=cfg.maxAttitudeError_deg;
if scenario.expectedFailsafe,estimatorPositionPass=maxEstimatorPositionError<=cfg.maxEstimatorFailsafeBound_m;else,estimatorPositionPass=maxEstimatorPositionError<=cfg.maxEstimatorPositionError_m;end
estimatorAttitudePass=rad2deg(maxEstimatorAttitudeError)<=cfg.maxEstimatorAttitudeError_deg;
continuityPass=all(maxReferenceContinuity<=cfg.maxReferenceContinuityJump)&&all(maxReplanStateJump<=cfg.maxReplanStateJump);
missionOutcomePass=(scenario.expectedGoalReached&&goalReached&&~failsafe)||(~scenario.expectedGoalReached&&failsafe&&~goalReached);
failsafeExpectationPass=(failsafe==scenario.expectedFailsafe);replanEventPass=~scenario.expectedReplan||replanCount>=1;
dynamicEventPass=~scenario.expectedDynamicAvoidance||(dynamicAvoidSteps>=1||dynamicHoldCount>=1);promotionEventPass=~scenario.expectedPromotion||promotionCount>=1;
noDataEventPass=~scenario.expectedObstacleNoDataHold||obstacleNoDataHoldCount>=1;laneSwitchEventPass=~scenario.expectedLaneSwitch||nav.selector.switchCount>=1;
rtlEventPass=~scenario.expectedRTLRequest||rtlRequested;uncertaintyPass=maxInflation>=cfg.baseInflationRadius&&maxInflation<=cfg.maxInflationRadius+1e-9;
eventPass=replanEventPass&&dynamicEventPass&&promotionEventPass&&noDataEventPass&&laneSwitchEventPass&&rtlEventPass;
pass=missionOutcomePass&&failsafeExpectationPass&&collisionCount==0&&geofenceViolationCount==0&&staticPass&&dynamicPass&& ...
    referenceKinematicPass&&executedKinematicPass&&controllerPass&&estimatorPositionPass&&estimatorAttitudePass&&continuityPass&& ...
    uncertaintyPass&&trajectoryGenerationCount>=1&&eventPass;

summary=struct('goalReached',goalReached,'pass',pass,'failsafeTriggered',failsafe,'rtlRequested',rtlRequested, ...
    'collisionCount',collisionCount,'geofenceViolationCount',geofenceViolationCount,'replanCount',replanCount, ...
    'promotionCount',promotionCount,'dynamicAvoidSteps',dynamicAvoidSteps,'dynamicHoldCount',dynamicHoldCount, ...
    'obstacleNoDataHoldCount',obstacleNoDataHoldCount,'rejoinCount',rejoinCount, ...
    'replanBrakeCount',replanBrakeCount,'replanRetryCount',replanRetryCount, ...
    'inflationReplanCount',inflationReplanCount, ...
    'dstarInitialExpanded',dstarInit.expanded,'dstarRepairExpanded',dstarRepairExpanded,'astarScratchExpanded',astarScratchExpanded, ...
    'astarRecoveryCount',astarRecoveryCount,'timeToGoal_s',timeToGoal,'pathLength_m',pathLength, ...
    'minObstacleClearance_m',minObs,'minWallClearance_m',minWall,'minDynamicClearance_m',minDynamicClearance, ...
    'minPredictedMargin_m',minPredictedMargin,'maxExecutedSpeed_mps',maxExecutedSpeed,'maxExecutedAccel_mps2',maxExecutedAccel, ...
    'maxExecutedJerk_mps3',maxExecutedJerk,'maxReferenceSpeed_mps',maxReferenceSpeed,'maxReferenceAccel_mps2',maxReferenceAccel, ...
    'maxReferenceJerk_mps3',maxReferenceJerk,'maxTilt_deg',rad2deg(maxTilt),'maxAltitudeError_m',maxAltitudeError, ...
    'maxEstimatorPositionError_m',maxEstimatorPositionError,'maxEstimatorAttitudeError_deg',rad2deg(maxEstimatorAttitudeError), ...
    'maxTrackingError_m',maxTrackingError,'maxSafetyOverrideDeviation_m',maxSafetyOverrideDeviation,'maxInflationRadius_m',maxInflation, ...
    'finalInflationRadius_m',currentInflation,'activeLaneFinal',est.activeLane,'laneSwitches',nav.selector.switchCount, ...
    'trajectoryGenerationCount',trajectoryGenerationCount,'trajectoryFallbackCount',trajectoryFallbackCount, ...
    'maxTrajectoryTimeScale',maxTrajectoryTimeScale,'maxReferenceContinuityJump',maxReferenceContinuity,'maxReplanStateJump',maxReplanStateJump, ...
    'staticPass',staticPass,'dynamicPass',dynamicPass,'referenceKinematicPass',referenceKinematicPass, ...
    'executedKinematicPass',executedKinematicPass,'controllerPass',controllerPass,'estimatorPositionPass',estimatorPositionPass, ...
    'estimatorAttitudePass',estimatorAttitudePass,'continuityPass',continuityPass,'uncertaintyPass',uncertaintyPass, ...
    'missionOutcomePass',missionOutcomePass,'failsafeExpectationPass',failsafeExpectationPass,'replanEventPass',replanEventPass, ...
    'dynamicEventPass',dynamicEventPass,'promotionEventPass',promotionEventPass,'noDataEventPass',noDataEventPass, ...
    'laneSwitchEventPass',laneSwitchEventPass,'rtlEventPass',rtlEventPass,'eventPass',eventPass, ...
    'finalTruthPosition',truth.p.','finalEstimatedPosition',est.p.');

log=struct('t',T,'truthP',Ptrue,'truthV',Vtrue,'truthA',Atrue,'truthJ',Jtrue,'truthQ',Qtrue,'truthRpy',RpyTrue, ...
    'estP',Pest,'estV',Vest,'estQ',Qest,'estRpy',RpyEst,'pRef',Pref,'vRef',Vref,'aRef',Aref, ...
    'trackingError',trackingLog,'estimatorPositionError',estErrorLog,'estimatorAttitudeError_deg',attErrorLog, ...
    'stateId',stateId,'laneId',laneId,'laneScores',laneScores,'laneEligible',laneEligible,'inflationRadius',inflationLog, ...
    'xySigma',xySigmaLog,'thrust_N',thrustLog,'moment_Nm',momentLog,'sensorAids',sensorAidLog,'predictedMargin',predMargin, ...
    'actualDynamic',actualDyn,'estimatedDynamic',estimatedDyn,'pathHistory',{pathHistory},'trajectoryHistory',{trajectoryHistory}, ...
    'activeObstacleHistory',{activeObstacleHistory},'inflationHistory',inflationHistory,'switchLog',{nav.switchLog});
maps=struct('finalGrid',grid,'activeObstacles',activeObstacles,'planner',planner);

    function store_sample()
        T(k)=(k-1)*cfg.dt;Ptrue(k,:)=truth.p.';Vtrue(k,:)=truth.v.';Atrue(k,:)=truth.a.';
        Jtrue(k,:)=currentTruthJerk.';
        Qtrue(k,:)=truth.q;RpyTrue(k,:)=q2rpy_S2_2(truth.q);Pest(k,:)=est.p.';Vest(k,:)=est.v.';Qest(k,:)=est.q;RpyEst(k,:)=q2rpy_S2_2(est.q);
        Pref(k,:)=ref3.p.';Vref(k,:)=ref3.v.';Aref(k,:)=ref3.a.';trackingLog(k)=norm(truth.p-ref3.p);estErrorLog(k)=norm(est.p-truth.p);
        attErrorLog(k)=rad2deg(norm(qlog_S2_2(qmul_S2_2(qconj_S2_2(est.q),truth.q))));stateId(k)=state_id(state);laneId(k)=est.activeLane;
        if ~isempty(est.scores),laneScores(k,:)=est.scores(:).';end;if ~isempty(est.eligible),laneEligible(k,:)=est.eligible(:).';end
        inflationLog(k)=currentInflation;xySigmaLog(k)=est.xySigma_m;thrustLog(k)=cmd.thrust_N;momentLog(k,:)=cmd.moment_Nm(:).';
        sensorAidLog(k,:)=[packet.hasVio packet.hasLidar packet.hasRange packet.hasBaro];predMargin(k)=stepMargin;
    end
end

function cfg2=grid_cfg(cfg,inflation)
cfg2=cfg;cfg2.inflationRadius=inflation;
end

function ref=make_controller_reference(cfg,state,est,refXY,desiredVel,safetyAccel,holdPosition)
if strcmp(state,'TRACK')
    if isempty(refXY)
        ref=hold_ref(est,cfg);
    else
        ref=struct('p',[refXY.p(:);cfg.altitudeNominal_m], ...
            'v',[refXY.v(:);0],'a',[refXY.a(:);0],'yaw',0);
    end
elseif any(strcmp(state,{'DYNAMIC_AVOID','DYNAMIC_HOLD','REJOIN','REPLAN_BRAKE'}))
    % Velocity-mode safety/rejoin reference. Anchor XY position at the
    % current local estimate so a paused trajectory position error cannot
    % create a second, unbounded acceleration demand.
    ref=struct('p',[est.p(1:2);cfg.altitudeNominal_m], ...
        'v',[desiredVel(:);0],'a',[safetyAccel(:);0],'yaw',0);
else
    ref=struct('p',holdPosition(:),'v',zeros(3,1), ...
        'a',zeros(3,1),'yaw',0);ref.p(3)=cfg.altitudeNominal_m;
end
end

function ref=hold_ref(est,cfg)
ref=struct('p',est.p(:),'v',zeros(3,1),'a',zeros(3,1),'yaw',0);ref.p(3)=cfg.altitudeNominal_m;
end

function [traj,clock,jump]=new_trajectory(cfg,grid,path,est,estAcc,initialScale)
traj=generate_min_snap_trajectory_S2_2(cfg,grid,path,est.v(1:2),estAcc(1:2),initialScale);clock=0;jump=inf(1,3);
if traj.valid,r=sample_min_snap_state_S2_2(traj,0);jump=[norm(r.p-est.p(1:2)) norm(r.v-est.v(1:2)) norm(r.a-estAcc(1:2))];end
end

function [ng,nf,ms,mc,mr,mv,ma,mj]=update_traj_metrics(traj,jump,ng,nf,ms,mc,mr,mv,ma,mj)
if ~traj.valid,return;end
ng=ng+1;nf=nf+double(traj.fallbackUsed);ms=max(ms,traj.timeScale);mc=max(mc,traj.continuity);mr=max(mr,jump);mv=max(mv,traj.maxSpeed_mps);ma=max(ma,traj.maxAccel_mps2);mj=max(mj,traj.maxJerk_mps3);
end

function path=prepare_path(grid,startXY,raw)
path=zeros(0,2);if isempty(raw),return;end
startXY=double(startXY(:).');raw=double(raw);if size(raw,2)~=2||any(~isfinite(raw(:)))||any(~isfinite(startXY)),return;end
raw=[startXY;raw];raw=remove_duplicates(raw,grid.resolution/4);raw(1,:)=startXY;candidate=smooth_path_S2_2(grid,raw);if ~isempty(candidate),candidate(1,:)=startXY;end
if path_valid(grid,candidate),path=candidate;elseif path_valid(grid,raw),path=raw;end
end
function tf=path_valid(grid,path)
tf=~isempty(path)&&size(path,1)>=2;if ~tf,return;end
for i=1:size(path,1)-1,if segment_occupied_grid_S2_2(grid,path(i,:),path(i+1,:)),tf=false;return;end,end
end
function p=remove_duplicates(p,tol)
if isempty(p),return;end
keep=true(size(p,1),1);last=1;for i=2:size(p,1),if norm(p(i,:)-p(last,:))<tol,keep(i)=false;else,last=i;end,end;p=p(keep,:);
end
function tf=inside_windows(t,w)
tf=false;for i=1:size(w,1),if t>=w(i,1)&&t<=w(i,2),tf=true;return;end,end
end
function id=state_id(s)
switch s
    case 'TRACK',id=1;case 'DYNAMIC_AVOID',id=2;case 'DYNAMIC_HOLD',id=3;case 'OBSTACLE_NO_DATA_HOLD',id=4;
    case 'FAILSAFE',id=5;case 'COMPLETE',id=6;case 'REJOIN',id=7;case 'REPLAN_BRAKE',id=8;otherwise,id=0;
end
end
function x=finite_or_zero(x),if ~isfinite(x),x=0;end,end

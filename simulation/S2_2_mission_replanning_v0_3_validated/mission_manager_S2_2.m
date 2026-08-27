function [log,summary,maps] = mission_manager_S2_2(cfg,scenario)
% MISSION_MANAGER_S2_2  S2.2 v0.3 incremental/dynamic + smooth trajectory logic.
activeObstacles=scenario.knownObstacles;grid=build_occupancy_grid_S2_2(cfg,activeObstacles);
[planner,rawPath,dstarInit]=dstar_lite_S2_2('init',grid,scenario.start,scenario.goal);
path=prepare_path(grid,scenario.start,rawPath);pos=scenario.start(:);vel=zeros(2,1);acc=zeros(2,1);t=0;
traj=generate_min_snap_trajectory_S2_2(cfg,grid,path,vel,acc,scenario.trajectoryInitialTimeScale);trajClock=0;
if isempty(path)
    warning('S2_2:InitialPathFailed', ...
        'D* Lite did not provide a valid initial path from [%.2f %.2f] to [%.2f %.2f].', ...
        scenario.start(1),scenario.start(2),scenario.goal(1),scenario.goal(2));
    state='FAILSAFE';
elseif ~traj.valid
    warning('S2_2:InitialTrajectoryFailed', ...
        ['The initial path contained %d waypoints, but trajectory generation failed. ' ...
         'Inspect path/trajectory collision consistency and kinematic limits.'],size(path,1));
    state='FAILSAFE';
else
    state='TRACK';
end
insertedDone=false(1,numel(scenario.insertedObstacles));nDyn=numel(scenario.dynamicObstacles);
tracks=cell(1,nDyn);stopTimers=zeros(1,nDyn);promoted=false(1,nDyn);
replanCount=0;dynamicAvoidSteps=0;dynamicHoldCount=0;noDataHoldCount=0;promotionCount=0;
localRefreshCount=0;astarRecoveryCount=0;dstarRepairExpanded=0;astarScratchExpanded=0;
collisionCount=0;geofenceViolationCount=0;goalReached=false;failsafe=strcmp(state,'FAILSAFE');
timeToGoal=nan;noDataElapsed=0;clearTimer=0;wasAvoiding=false;stallTimer=0;pathLength=0;
maxSpeed=0;maxAccel=0;maxJerk=0;maxTrackingError=0;maxSafetyOverrideDeviation=0;
minPredictedMargin=inf;minDynamicClearance=inf;rejoinCount=0;referenceLocked=traj.valid;
minWall=inf;minObs=inf;trajectoryGenerationCount=double(traj.valid);trajectoryFallbackCount=double(traj.valid&&traj.fallbackUsed);
maxTrajectoryTimeScale=finite_or_zero(traj.timeScale);maxReferenceContinuity=zeros(1,4);
if traj.valid,maxReferenceContinuity=max(maxReferenceContinuity,traj.continuity);end
maxReplanStateJump=zeros(1,3);
maxSteps=ceil(cfg.maxTime_s/cfg.dt)+1;
T=nan(maxSteps,1);P=nan(maxSteps,2);V=nan(maxSteps,2);Alog=nan(maxSteps,2);Jlog=nan(maxSteps,2);
Pref=nan(maxSteps,2);Vref=nan(maxSteps,2);Aref=nan(maxSteps,2);trackingLog=nan(maxSteps,1);
trackingValidationLog=nan(maxSteps,1);referenceLockedLog=false(maxSteps,1);
stateId=nan(maxSteps,1);sensorValid=false(maxSteps,1);predMargin=nan(maxSteps,1);dynAvoid=false(maxSteps,1);
actualDyn=nan(maxSteps,max(1,nDyn),2);estimatedDyn=nan(maxSteps,max(1,nDyn),2);
pathHistory={path};trajectoryHistory={traj};activeObstacleHistory={activeObstacles};
k=1;T(k)=0;P(k,:)=pos.';V(k,:)=vel.';Alog(k,:)=acc.';Jlog(k,:)=[0 0];stateId(k)=state_id(state);sensorValid(k)=true;
referenceLockedLog(k)=referenceLocked;
if traj.valid,r0=sample_min_snap_state_S2_2(traj,0);Pref(k,:)=r0.p.';Vref(k,:)=r0.v.';Aref(k,:)=r0.a.';end
while t<cfg.maxTime_s
    k=k+1;t=t+cfg.dt;mapChanged=false;changedMask=false(grid.ny,grid.nx);
    for i=1:numel(scenario.insertedObstacles)
        if ~insertedDone(i)&&t>=scenario.insertedObstacles(i).time
            oldOcc=grid.occ;activeObstacles=[activeObstacles;scenario.insertedObstacles(i).rect]; %#ok<AGROW>
            grid=build_occupancy_grid_S2_2(cfg,activeObstacles);changedMask=changedMask|xor(oldOcc,grid.occ);
            insertedDone(i)=true;mapChanged=true;activeObstacleHistory{end+1}=activeObstacles; %#ok<AGROW>
        end
    end
    coverageOK=~inside_windows(t,scenario.sensorDropoutWindows);
    sensedDyn=struct('p',{},'v',{},'radius',{},'id',{});actualStates=struct('p',{},'v',{},'radius',{},'id',{});
    for i=1:nDyn
        [pTrue,vTrue,active]=dynamic_obstacle_state_S2_2(scenario.dynamicObstacles(i),t);
        if ~active
            % Do not retain stale kinematics or stopped-time evidence across
            % disappearance/reappearance intervals.
            if ~promoted(i),tracks{i}=[];stopTimers(i)=0;end
            continue;
        end
        actualStates(end+1)=struct('p',pTrue,'v',vTrue,'radius',scenario.dynamicObstacles(i).radius,'id',i); %#ok<AGROW>
        actualDyn(k,i,:)=pTrue;if ~coverageOK,continue;end
        z=pTrue+cfg.dynamicPositionNoise_m*randn(1,2);tracks{i}=alpha_beta_track_S2_2(cfg,tracks{i},z,t);
        estimatedDyn(k,i,:)=tracks{i}.p;
        % Require several tracker updates before interpreting low estimated
        % velocity as a persistent stopped object. The first alpha-beta
        % update intentionally starts with zero velocity.
        if tracks{i}.updates>=3 && norm(tracks{i}.v)<=cfg.stoppedSpeedThreshold_mps
            stopTimers(i)=stopTimers(i)+cfg.dt;
        else
            stopTimers(i)=0;
        end
        if stopTimers(i)>=cfg.stoppedPersistence_s&&~promoted(i)
            r=scenario.dynamicObstacles(i).radius;rect=[tracks{i}.p(1)-r tracks{i}.p(2)-r 2*r 2*r];oldOcc=grid.occ;
            activeObstacles=[activeObstacles;rect];grid=build_occupancy_grid_S2_2(cfg,activeObstacles); %#ok<AGROW>
            changedMask=changedMask|xor(oldOcc,grid.occ);promoted(i)=true;promotionCount=promotionCount+1;mapChanged=true;
            activeObstacleHistory{end+1}=activeObstacles; %#ok<AGROW>
        end
        if ~promoted(i),sensedDyn(end+1)=struct('p',tracks{i}.p,'v',tracks{i}.v,'radius',scenario.dynamicObstacles(i).radius,'id',i);end %#ok<AGROW>
    end
    if mapChanged
        [planner,rawPath,st]=dstar_lite_S2_2('repair',planner,grid,pos.',changedMask);
        dstarRepairExpanded=dstarRepairExpanded+st.expanded;
        [rawA,aInfo]=astar_grid_S2_2(grid,pos.',scenario.goal);
        astarScratchExpanded=astarScratchExpanded+aInfo.expanded;

        path=prepare_path(grid,pos.',rawPath);
        [traj,trajClock,jump]=new_trajectory(cfg,grid,path,pos,vel,acc,1.0);

        % D* Lite remains the primary incremental planner. A fresh A* path
        % is already computed for the expansion benchmark; use it as a real
        % recovery path only when the repaired route or its polynomial is
        % invalid. This prevents a false failsafe from one planner-specific
        % extraction/generation failure while preserving the D* metric.
        if (isempty(path)||~traj.valid) && ~isempty(rawA)
            [planner,~,~]=dstar_lite_S2_2('init',grid,pos.',scenario.goal);
            path=prepare_path(grid,pos.',rawA);
            [traj,trajClock,jump]=new_trajectory(cfg,grid,path,pos,vel,acc,1.0);
            astarRecoveryCount=astarRecoveryCount+1;
        end

        replanCount=replanCount+1;wasAvoiding=false;stallTimer=0;
        pathHistory{end+1}=path;trajectoryHistory{end+1}=traj; %#ok<AGROW>
        [trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump]= ...
            update_traj_metrics(traj,jump,trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump);
        referenceLocked=traj.valid;
        if isempty(path)||~traj.valid,state='FAILSAFE';failsafe=true;end
    end
    cmdVel=zeros(2,1);modified=false;stepMargin=inf;trackingError=nan;ref=[];
    if strcmp(state,'FAILSAFE')
        failsafe=true;cmdVel(:)=0;
    elseif norm(pos.'-scenario.goal)<=cfg.goalTolerance_m
        goalReached=true;timeToGoal=t;state='COMPLETE';cmdVel(:)=0;
    elseif ~coverageOK
        referenceLocked=false;
        noDataElapsed=noDataElapsed+cfg.dt;cmdVel(:)=0;
        if noDataElapsed>=cfg.noDataStopTimeout_s
            if ~strcmp(state,'NO_DATA_HOLD'),noDataHoldCount=noDataHoldCount+1;end
            state='NO_DATA_HOLD';
        end
        if noDataElapsed>=cfg.noDataFailsafeTimeout_s,state='FAILSAFE';failsafe=true;end
    else
        if strcmp(state,'NO_DATA_HOLD')
            [planner,rawPath]=dstar_lite_S2_2('refresh',planner,grid,pos.');
            path=prepare_path(grid,pos.',rawPath);
            [traj,trajClock,jump]=new_trajectory(cfg,grid,path,pos,vel,acc,1.0);

            % A recovered sensor stream must not immediately become a false
            % failsafe merely because an incremental refresh cannot provide
            % a valid smooth trajectory. Recompute once with fresh A*.
            if isempty(path)||~traj.valid
                [rawA,aInfo]=astar_grid_S2_2(grid,pos.',scenario.goal);
                astarScratchExpanded=astarScratchExpanded+aInfo.expanded;
                if ~isempty(rawA)
                    [planner,~,~]=dstar_lite_S2_2('init',grid,pos.',scenario.goal);
                    path=prepare_path(grid,pos.',rawA);
                    [traj,trajClock,jump]=new_trajectory(cfg,grid,path,pos,vel,acc,1.0);
                    astarRecoveryCount=astarRecoveryCount+1;
                end
            end

            localRefreshCount=localRefreshCount+1;pathHistory{end+1}=path;trajectoryHistory{end+1}=traj; %#ok<AGROW>
            [trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump]= ...
                update_traj_metrics(traj,jump,trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump);
            referenceLocked=traj.valid;
        end
        noDataElapsed=0;
        if isempty(path)||~traj.valid
            state='FAILSAFE';failsafe=true;
        else
            [desired,trackingError,ref]=track_smooth_trajectory_S2_2(cfg,pos,vel,traj,trajClock);
            if norm(desired)>1e-9
                direction=desired/norm(desired);vLimit=static_braking_speed_S2_2(cfg,pos,direction,grid);
                if norm(desired)>vLimit,desired=desired*(vLimit/norm(desired));end
            end
            if norm(desired)<1e-8&&norm(pos.'-scenario.goal)>cfg.goalTolerance_m,stallTimer=stallTimer+cfg.dt;else,stallTimer=0;end
            if stallTimer>=cfg.stallRecoveryTime_s
                [rawA,aInfo]=astar_grid_S2_2(grid,pos.',scenario.goal);astarScratchExpanded=astarScratchExpanded+aInfo.expanded;
                if isempty(rawA),state='FAILSAFE';failsafe=true;desired(:)=0;
                else
                    [planner,~,~]=dstar_lite_S2_2('init',grid,pos.',scenario.goal);path=prepare_path(grid,pos.',rawA);
                    astarRecoveryCount=astarRecoveryCount+1;stallTimer=0;pathHistory{end+1}=path; %#ok<AGROW>
                    [traj,trajClock,jump]=new_trajectory(cfg,grid,path,pos,vel,acc,1.0);trajectoryHistory{end+1}=traj; %#ok<AGROW>
                    [trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump]= ...
                        update_traj_metrics(traj,jump,trajectoryGenerationCount,trajectoryFallbackCount,maxTrajectoryTimeScale,maxReferenceContinuity,maxReplanStateJump);
                    referenceLocked=traj.valid;
                    if traj.valid,[desired,trackingError,ref]=track_smooth_trajectory_S2_2(cfg,pos,vel,traj,trajClock);else,state='FAILSAFE';failsafe=true;end
                end
            end
            [cmdVel,modified,stepMargin]=velocity_obstacle_filter_S2_2(cfg,pos,desired,sensedDyn,grid);
            minPredictedMargin=min(minPredictedMargin,stepMargin);

            if modified
                % The collision-avoidance layer intentionally overrides the
                % polynomial tracker. Freeze the reference clock and evaluate
                % this excursion separately from nominal TRACK performance.
                referenceLocked=false;
                wasAvoiding=true;dynamicAvoidSteps=dynamicAvoidSteps+1;dynAvoid(k)=true;
                if norm(cmdVel)<1e-6
                    if ~strcmp(state,'DYNAMIC_HOLD'),dynamicHoldCount=dynamicHoldCount+1;end
                    state='DYNAMIC_HOLD';clearTimer=0;
                else
                    state='DYNAMIC_AVOID';
                end

            elseif strcmp(state,'DYNAMIC_HOLD')
                % Require a short obstacle-clear interval before attempting
                % to rejoin the paused smooth reference.
                referenceLocked=false;
                clearTimer=clearTimer+cfg.dt;cmdVel(:)=0;
                if clearTimer>=cfg.holdClearTime_s
                    state='REJOIN';
                    rejoinCount=rejoinCount+1;
                    stallTimer=0;
                end

            elseif wasAvoiding || strcmp(state,'REJOIN') || ~referenceLocked
                % Recover toward the paused polynomial reference. This is a
                % separate hybrid mode, not a nominal tracking-error sample.
                if ~strcmp(state,'REJOIN'),rejoinCount=rejoinCount+1;end
                state='REJOIN';cmdVel=desired;
                if isfinite(trackingError) && trackingError<=cfg.trajectoryClockMaxError_m
                    state='TRACK';
                    referenceLocked=true;
                    wasAvoiding=false;
                    clearTimer=0;
                    stallTimer=0;
                end

            else
                state='TRACK';cmdVel=desired;
            end

            % Mode-aware validation: intentional safety overrides and the
            % subsequent rejoin transient are not controller tracking error.
            % They remain visible through maxSafetyOverrideDeviation_m.
            if isfinite(trackingError)
                if referenceLocked && strcmp(state,'TRACK')
                    maxTrackingError=max(maxTrackingError,trackingError);
                else
                    maxSafetyOverrideDeviation=max(maxSafetyOverrideDeviation,trackingError);
                end
            end
        end
    end
    oldPos=pos;oldAcc=acc;
    if strcmp(state,'COMPLETE')
        vel(:)=0;acc(:)=0;jerk=zeros(2,1);
    else
        [pos,vel,acc,jerk]=jerk_limited_step_S2_2(cfg,pos,vel,acc,cmdVel);pathLength=pathLength+norm(pos-oldPos);
    end
    if referenceLocked&&~modified&&strcmp(state,'TRACK')&&traj.valid&& ...
            isfinite(trackingError)&&trackingError<=cfg.trajectoryClockMaxError_m
        trajClock=min(traj.duration_s,trajClock+cfg.dt);
    end
    maxSpeed=max(maxSpeed,norm(vel));maxAccel=max(maxAccel,norm(acc));maxJerk=max(maxJerk,norm(jerk));
    wallRaw=min([pos(1),cfg.room(1)-pos(1),pos(2),cfg.room(2)-pos(2)]);minWall=min(minWall,wallRaw);
    if wallRaw<cfg.collisionRadius,geofenceViolationCount=geofenceViolationCount+1;end
    obstacleClearanceNow=inf;
    for j=1:size(activeObstacles,1)
        dObs=dist_point_rect_S2_2(pos.',activeObstacles(j,:));obstacleClearanceNow=min(obstacleClearanceNow,dObs);
        if dObs<cfg.collisionRadius,collisionCount=collisionCount+1;end
    end
    minObs=min(minObs,obstacleClearanceNow);
    for j=1:numel(actualStates)
        d=norm(pos.'-actualStates(j).p)-cfg.collisionRadius-actualStates(j).radius;minDynamicClearance=min(minDynamicClearance,d);
        if d<0,collisionCount=collisionCount+1;end
    end
    T(k)=t;P(k,:)=pos.';V(k,:)=vel.';Alog(k,:)=acc.';Jlog(k,:)=jerk.';stateId(k)=state_id(state);
    sensorValid(k)=coverageOK;predMargin(k)=stepMargin;trackingLog(k)=trackingError;
    referenceLockedLog(k)=referenceLocked;
    if referenceLocked&&strcmp(state,'TRACK')&&isfinite(trackingError)
        trackingValidationLog(k)=trackingError;
    end
    if ~isempty(ref),Pref(k,:)=ref.p.';Vref(k,:)=ref.v.';Aref(k,:)=ref.a.';end
    if strcmp(state,'FAILSAFE')||strcmp(state,'COMPLETE'),break;end
end
valid=isfinite(T);T=T(valid);P=P(valid,:);V=V(valid,:);Alog=Alog(valid,:);Jlog=Jlog(valid,:);
Pref=Pref(valid,:);Vref=Vref(valid,:);Aref=Aref(valid,:);trackingLog=trackingLog(valid);
trackingValidationLog=trackingValidationLog(valid);referenceLockedLog=referenceLockedLog(valid);stateId=stateId(valid);
sensorValid=sensorValid(valid);predMargin=predMargin(valid);dynAvoid=dynAvoid(valid);actualDyn=actualDyn(valid,:,:);estimatedDyn=estimatedDyn(valid,:,:);
staticPass=(isempty(activeObstacles)||minObs>=cfg.inflationRadius-0.03)&&minWall>=cfg.inflationRadius-0.03;
dynamicPass=~isfinite(minDynamicClearance)||minDynamicClearance>=cfg.minDynamicPhysicalClearance_m;
kinematicPass=maxSpeed<=cfg.maxSpeedXY_mps+1e-6&&maxAccel<=cfg.maxAccelXY_mps2+1e-6&&maxJerk<=cfg.maxJerkXY_mps3+1e-6;
referenceContinuityPass=all(maxReferenceContinuity<=cfg.maxReferenceContinuityJump);
replanContinuityPass=all(maxReplanStateJump<=cfg.maxReplanStateJump);
continuityPass=referenceContinuityPass&&replanContinuityPass;
trackingPass=maxTrackingError<=cfg.maxTrajectoryTrackingError_m;
missionOutcomePass=(scenario.expectedGoalReached&&goalReached&&~failsafe)|| ...
    (~scenario.expectedGoalReached&&failsafe&&~goalReached);
failsafeExpectationPass=(failsafe==scenario.expectedFailsafe);
incrementalEventPass=~scenario.expectedIncrementalReplan||replanCount>=1;
dynamicEventPass=~scenario.expectedDynamicAvoidance||(dynamicAvoidSteps>=1||dynamicHoldCount>=1);
noDataEventPass=~scenario.expectedNoDataHold||noDataHoldCount>=1;
promotionEventPass=~scenario.expectedPromotion||promotionCount>=1;
timeRescalePass=~scenario.expectedTimeRescale||maxTrajectoryTimeScale>=1.5;
searchEfficiencyPass=~strcmp(scenario.name,'INCREMENTAL_STATIC_INSERT')|| ...
    (dstarRepairExpanded>0&&astarScratchExpanded>dstarRepairExpanded);
eventPass=incrementalEventPass&&dynamicEventPass&&noDataEventPass&& ...
    promotionEventPass&&timeRescalePass&&searchEfficiencyPass;
pass=missionOutcomePass&&failsafeExpectationPass&&collisionCount==0&& ...
    geofenceViolationCount==0&&staticPass&&dynamicPass&&kinematicPass&& ...
    continuityPass&&trackingPass&&trajectoryGenerationCount>=1&&eventPass;
summary=struct('goalReached',goalReached,'pass',pass,'failsafeTriggered',failsafe,'collisionCount',collisionCount, ...
    'geofenceViolationCount',geofenceViolationCount,'replanCount',replanCount,'dynamicAvoidSteps',dynamicAvoidSteps, ...
    'dynamicHoldCount',dynamicHoldCount,'noDataHoldCount',noDataHoldCount,'promotionCount',promotionCount, ...
    'localRefreshCount',localRefreshCount,'astarRecoveryCount',astarRecoveryCount,'dstarInitialExpanded',dstarInit.expanded, ...
    'dstarRepairExpanded',dstarRepairExpanded,'astarScratchExpanded',astarScratchExpanded,'timeToGoal_s',timeToGoal, ...
    'pathLength_m',pathLength,'minObstacleClearance_m',minObs,'minWallClearance_m',minWall, ...
    'minDynamicClearance_m',minDynamicClearance,'minPredictedMargin_m',minPredictedMargin,'maxSpeed_mps',maxSpeed, ...
    'maxAccel_mps2',maxAccel,'maxJerk_mps3',maxJerk,'maxTrackingError_m',maxTrackingError, ...
    'maxSafetyOverrideDeviation_m',maxSafetyOverrideDeviation,'rejoinCount',rejoinCount, ...
    'trajectoryGenerationCount',trajectoryGenerationCount,'trajectoryFallbackCount',trajectoryFallbackCount, ...
    'maxTrajectoryTimeScale',maxTrajectoryTimeScale,'maxReferenceContinuityJump',maxReferenceContinuity, ...
    'maxReplanStateJump',maxReplanStateJump,'staticPass',staticPass,'dynamicPass',dynamicPass, ...
    'kinematicPass',kinematicPass,'referenceContinuityPass',referenceContinuityPass, ...
    'replanContinuityPass',replanContinuityPass,'continuityPass',continuityPass, ...
    'trackingPass',trackingPass,'missionOutcomePass',missionOutcomePass, ...
    'failsafeExpectationPass',failsafeExpectationPass,'incrementalEventPass',incrementalEventPass, ...
    'dynamicEventPass',dynamicEventPass,'noDataEventPass',noDataEventPass, ...
    'promotionEventPass',promotionEventPass,'timeRescalePass',timeRescalePass, ...
    'searchEfficiencyPass',searchEfficiencyPass,'eventPass',eventPass, ...
    'finalPosition',P(end,:));
log=struct('t',T,'p',P,'v',V,'a',Alog,'j',Jlog,'pRef',Pref,'vRef',Vref,'aRef',Aref, ...
    'trackingError',trackingLog,'trackingValidationError',trackingValidationLog, ...
    'referenceLocked',referenceLockedLog,'stateId',stateId,'sensorValid',sensorValid,'predictedMargin',predMargin, ...
    'dynamicAvoid',dynAvoid,'pathHistory',{pathHistory},'trajectoryHistory',{trajectoryHistory}, ...
    'actualDynamic',actualDyn,'estimatedDynamic',estimatedDyn,'activeObstacleHistory',{activeObstacleHistory});
maps=struct('finalGrid',grid,'activeObstacles',activeObstacles,'planner',planner);
end

function [traj,clock,jump]=new_trajectory(cfg,grid,path,pos,vel,acc,initialScale)
traj=generate_min_snap_trajectory_S2_2(cfg,grid,path,vel,acc,initialScale);clock=0;jump=inf(1,3);
if traj.valid
    r=sample_min_snap_state_S2_2(traj,0);jump=[norm(r.p-pos(:)) norm(r.v-vel(:)) norm(r.a-acc(:))];
end
end
function [ng,nf,ms,mc,mr]=update_traj_metrics(traj,jump,ng,nf,ms,mc,mr)
if ~traj.valid,return;end
ng=ng+1;nf=nf+double(traj.fallbackUsed);ms=max(ms,traj.timeScale);mc=max(mc,traj.continuity);mr=max(mr,jump);
end
function path=prepare_path(grid,startXY,raw)
% Anchor every planned or repaired route to the exact continuous vehicle
% position. D* Lite returns a grid-cell centre, which is not a valid
% continuous replanning initial condition when the vehicle is off-centre.
path=zeros(0,2);
if isempty(raw),return;end
startXY=double(startXY(:).');raw=double(raw);
if size(raw,2)~=2||numel(startXY)~=2||any(~isfinite(raw(:)))||any(~isfinite(startXY)),return;end
raw=[startXY;raw];
raw=remove_duplicates(raw,grid.resolution/4);
raw(1,:)=startXY;
candidate=smooth_path_S2_2(grid,raw);
if ~isempty(candidate),candidate(1,:)=startXY;end
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
switch s,case 'TRACK',id=1;case 'DYNAMIC_AVOID',id=2;case 'DYNAMIC_HOLD',id=3;case 'NO_DATA_HOLD',id=4;case 'FAILSAFE',id=5;case 'COMPLETE',id=6;case 'REJOIN',id=7;otherwise,id=0;end
end
function x=finite_or_zero(x),if ~isfinite(x),x=0;end,end

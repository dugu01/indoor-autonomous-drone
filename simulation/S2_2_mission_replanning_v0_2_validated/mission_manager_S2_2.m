function [log,summary,maps] = mission_manager_S2_2(cfg,scenario)
% MISSION_MANAGER_S2_2  S2.2 v0.2 incremental/dynamic mission logic.
%
% Architecture:
%   D* Lite repairs persistent map changes.
%   A finite-horizon velocity-obstacle filter handles transient movers.
%   An alpha-beta tracker estimates mover velocity from noisy positions.
%   Stopped persistent movers are promoted to the static costmap.
%   Missing obstacle data forces XY hold and then failsafe.

activeObstacles=scenario.knownObstacles;
grid=build_occupancy_grid_S2_2(cfg,activeObstacles);
[planner,rawPath,dstarInit]=dstar_lite_S2_2('init',grid,scenario.start,scenario.goal);
path=prepare_path(grid,scenario.start,rawPath);pathIndex=2;
if isempty(path),state='FAILSAFE';else,state='TRACK';end

pos=scenario.start(:);vel=zeros(2,1);t=0;
insertedDone=false(1,numel(scenario.insertedObstacles));
nDyn=numel(scenario.dynamicObstacles);tracks=cell(1,nDyn);stopTimers=zeros(1,nDyn);promoted=false(1,nDyn);

replanCount=0;dynamicAvoidSteps=0;dynamicHoldCount=0;noDataHoldCount=0;
promotionCount=0;localRefreshCount=0;astarRecoveryCount=0;
dstarRepairExpanded=0;astarScratchExpanded=0;
collisionCount=0;geofenceViolationCount=0;goalReached=false;failsafe=strcmp(state,'FAILSAFE');
timeToGoal=nan;noDataElapsed=0;clearTimer=0;wasAvoiding=false;stallTimer=0;
pathLength=0;maxSpeed=0;maxTrackingError=0;minPredictedMargin=inf;minDynamicClearance=inf;
minWall=inf;minObs=inf;

maxSteps=ceil(cfg.maxTime_s/cfg.dt)+1;
T=nan(maxSteps,1);P=nan(maxSteps,2);V=nan(maxSteps,2);stateId=nan(maxSteps,1);
sensorValid=false(maxSteps,1);predMargin=nan(maxSteps,1);dynAvoid=false(maxSteps,1);
actualDyn=nan(maxSteps,max(1,nDyn),2);estimatedDyn=nan(maxSteps,max(1,nDyn),2);
pathHistory={path};activeObstacleHistory={activeObstacles};

k=1;T(k)=0;P(k,:)=pos.';V(k,:)=vel.';stateId(k)=state_id(state);sensorValid(k)=true;

while t<cfg.maxTime_s
    k=k+1;t=t+cfg.dt;mapChanged=false;changedMask=false(grid.ny,grid.nx);

    % Scheduled unknown static obstacles.
    for i=1:numel(scenario.insertedObstacles)
        if ~insertedDone(i) && t>=scenario.insertedObstacles(i).time
            oldOcc=grid.occ;activeObstacles=[activeObstacles;scenario.insertedObstacles(i).rect]; %#ok<AGROW>
            grid=build_occupancy_grid_S2_2(cfg,activeObstacles);
            changedMask=changedMask | xor(oldOcc,grid.occ);insertedDone(i)=true;mapChanged=true;
            activeObstacleHistory{end+1}=activeObstacles; %#ok<AGROW>
        end
    end

    coverageOK=~inside_windows(t,scenario.sensorDropoutWindows);
    sensedDyn=struct('p',{},'v',{},'radius',{},'id',{});
    actualStates=struct('p',{},'v',{},'radius',{},'id',{});

    for i=1:nDyn
        [pTrue,vTrue,active]=dynamic_obstacle_state_S2_2(scenario.dynamicObstacles(i),t);
        if ~active,continue;end
        actualStates(end+1)=struct('p',pTrue,'v',vTrue,'radius',scenario.dynamicObstacles(i).radius,'id',i); %#ok<AGROW>
        actualDyn(k,i,:)=pTrue;
        if ~coverageOK,continue;end
        z=pTrue+cfg.dynamicPositionNoise_m*randn(1,2);
        tracks{i}=alpha_beta_track_S2_2(cfg,tracks{i},z,t);
        estimatedDyn(k,i,:)=tracks{i}.p;
        if norm(tracks{i}.v)<=cfg.stoppedSpeedThreshold_mps,stopTimers(i)=stopTimers(i)+cfg.dt;else,stopTimers(i)=0;end
        if stopTimers(i)>=cfg.stoppedPersistence_s && ~promoted(i)
            r=scenario.dynamicObstacles(i).radius;rect=[tracks{i}.p(1)-r,tracks{i}.p(2)-r,2*r,2*r];
            oldOcc=grid.occ;activeObstacles=[activeObstacles;rect]; %#ok<AGROW>
            grid=build_occupancy_grid_S2_2(cfg,activeObstacles);
            changedMask=changedMask | xor(oldOcc,grid.occ);promoted(i)=true;promotionCount=promotionCount+1;mapChanged=true;
            activeObstacleHistory{end+1}=activeObstacles; %#ok<AGROW>
        end
        if ~promoted(i)
            sensedDyn(end+1)=struct('p',tracks{i}.p,'v',tracks{i}.v,'radius',scenario.dynamicObstacles(i).radius,'id',i); %#ok<AGROW>
        end
    end

    if mapChanged
        [planner,rawPath,st]=dstar_lite_S2_2('repair',planner,grid,pos.',changedMask);
        dstarRepairExpanded=dstarRepairExpanded+st.expanded;
        [~,aInfo]=astar_grid_S2_2(grid,pos.',scenario.goal);astarScratchExpanded=astarScratchExpanded+aInfo.expanded;
        path=prepare_path(grid,pos.',rawPath);pathIndex=2;replanCount=replanCount+1;wasAvoiding=false;
        pathHistory{end+1}=path; %#ok<AGROW>
        if isempty(path),state='FAILSAFE';failsafe=true;end
    end

    cmd=zeros(2,1);modified=false;stepMargin=inf;
    if strcmp(state,'FAILSAFE')
        failsafe=true;vel(:)=0;
    elseif norm(pos.'-scenario.goal)<=cfg.goalTolerance_m
        goalReached=true;timeToGoal=t;state='COMPLETE';vel(:)=0;
    elseif ~coverageOK
        noDataElapsed=noDataElapsed+cfg.dt;
        if noDataElapsed>=cfg.noDataStopTimeout_s
            if ~strcmp(state,'NO_DATA_HOLD'),noDataHoldCount=noDataHoldCount+1;end
            state='NO_DATA_HOLD';cmd(:)=0;
        else
            cmd=vel;
        end
        if noDataElapsed>=cfg.noDataFailsafeTimeout_s
            state='FAILSAFE';failsafe=true;cmd(:)=0;
        end
    else
        if strcmp(state,'NO_DATA_HOLD')
            [planner,rawPath]=dstar_lite_S2_2('refresh',planner,grid,pos.');
            path=prepare_path(grid,pos.',rawPath);pathIndex=2;localRefreshCount=localRefreshCount+1;
            pathHistory{end+1}=path; %#ok<AGROW>
        end
        noDataElapsed=0;
        if isempty(path)
            state='FAILSAFE';failsafe=true;
        else
            while pathIndex<size(path,1) && norm(pos.'-path(pathIndex,:))<=cfg.waypointTolerance_m,pathIndex=pathIndex+1;end
            target=path(min(pathIndex,size(path,1)),:);err=target(:)-pos;trackingError=norm(err);
            maxTrackingError=max(maxTrackingError,trackingError);
            if trackingError<1e-12,dirn=zeros(2,1);else,dirn=err/trackingError;end
            vLimit=static_braking_speed_S2_2(cfg,pos,dirn,grid);
            desired=dirn*min([cfg.maxSpeedXY_mps,vLimit,trackingError/max(cfg.dt,eps)]);
            if norm(desired)<1e-8 && trackingError>cfg.goalTolerance_m,stallTimer=stallTimer+cfg.dt;else,stallTimer=0;end

            if stallTimer>=cfg.stallRecoveryTime_s
                [rawA,aInfo]=astar_grid_S2_2(grid,pos.',scenario.goal);astarScratchExpanded=astarScratchExpanded+aInfo.expanded;
                if isempty(rawA)
                    state='FAILSAFE';failsafe=true;desired(:)=0;
                else
                    [planner,~,~]=dstar_lite_S2_2('init',grid,pos.',scenario.goal);
                    path=prepare_path(grid,pos.',rawA);pathIndex=2;astarRecoveryCount=astarRecoveryCount+1;stallTimer=0;
                    pathHistory{end+1}=path; %#ok<AGROW>
                    target=path(min(pathIndex,size(path,1)),:);err=target(:)-pos;trackingError=norm(err);dirn=err/max(trackingError,eps);
                    desired=dirn*min([cfg.maxSpeedXY_mps,static_braking_speed_S2_2(cfg,pos,dirn,grid),trackingError/max(cfg.dt,eps)]);
                end
            end

            [cmd,modified,stepMargin]=velocity_obstacle_filter_S2_2(cfg,pos,desired,sensedDyn,grid);
            minPredictedMargin=min(minPredictedMargin,stepMargin);
            if modified
                wasAvoiding=true;dynamicAvoidSteps=dynamicAvoidSteps+1;dynAvoid(k)=true;
                if norm(cmd)<1e-6
                    if ~strcmp(state,'DYNAMIC_HOLD'),dynamicHoldCount=dynamicHoldCount+1;end
                    state='DYNAMIC_HOLD';clearTimer=0;
                else
                    state='DYNAMIC_AVOID';
                end
            elseif strcmp(state,'DYNAMIC_HOLD')
                clearTimer=clearTimer+cfg.dt;cmd(:)=0;
                if clearTimer>=cfg.holdClearTime_s,state='TRACK';end
            else
                if wasAvoiding
                    [planner,rawPath]=dstar_lite_S2_2('refresh',planner,grid,pos.');
                    refreshed=prepare_path(grid,pos.',rawPath);
                    if ~isempty(refreshed),path=refreshed;pathIndex=2;localRefreshCount=localRefreshCount+1;pathHistory{end+1}=path;end %#ok<AGROW>
                    wasAvoiding=false;
                end
                state='TRACK';cmd=desired;
            end
        end

        dv=cmd-vel;maxDv=cfg.maxAccelXY_mps2*cfg.dt;
        if norm(dv)>maxDv,dv=dv*(maxDv/norm(dv));end
        oldPos=pos;vel=vel+dv;
        if norm(vel)>cfg.maxSpeedXY_mps,vel=vel*(cfg.maxSpeedXY_mps/norm(vel));end
        pos=pos+vel*cfg.dt;pathLength=pathLength+norm(pos-oldPos);
    end

    maxSpeed=max(maxSpeed,norm(vel));

    % Time-consistent safety metrics: compare the vehicle only with
    % obstacles that actually exist at the current simulation time.
    wallRaw=min([pos(1),cfg.room(1)-pos(1),pos(2),cfg.room(2)-pos(2)]);
    minWall=min(minWall,wallRaw);
    if wallRaw<cfg.collisionRadius,geofenceViolationCount=geofenceViolationCount+1;end

    obstacleClearanceNow=inf;
    for j=1:size(activeObstacles,1)
        dObs=dist_point_rect_S2_2(pos.',activeObstacles(j,:));
        obstacleClearanceNow=min(obstacleClearanceNow,dObs);
        if dObs<cfg.collisionRadius,collisionCount=collisionCount+1;end
    end
    minObs=min(minObs,obstacleClearanceNow);
    for j=1:numel(actualStates)
        d=norm(pos.'-actualStates(j).p)-cfg.collisionRadius-actualStates(j).radius;
        minDynamicClearance=min(minDynamicClearance,d);
        if d<0,collisionCount=collisionCount+1;end
    end

    T(k)=t;P(k,:)=pos.';V(k,:)=vel.';stateId(k)=state_id(state);sensorValid(k)=coverageOK;predMargin(k)=stepMargin;
    if strcmp(state,'FAILSAFE')||strcmp(state,'COMPLETE'),break;end
end

valid=isfinite(T);T=T(valid);P=P(valid,:);V=V(valid,:);stateId=stateId(valid);sensorValid=sensorValid(valid);predMargin=predMargin(valid);dynAvoid=dynAvoid(valid);
actualDyn=actualDyn(valid,:,:);estimatedDyn=estimatedDyn(valid,:,:);
staticPass=(isempty(activeObstacles)||minObs>=cfg.inflationRadius-0.03) && minWall>=cfg.inflationRadius-0.03;
dynamicPass=~isfinite(minDynamicClearance)||minDynamicClearance>=cfg.minDynamicPhysicalClearance_m;

if scenario.expectedGoalReached,pass=goalReached;else,pass=failsafe&&~goalReached;end
pass=pass&&collisionCount==0&&geofenceViolationCount==0&&staticPass&&dynamicPass;
if scenario.expectedIncrementalReplan,pass=pass&&replanCount>=1;end
if scenario.expectedDynamicAvoidance,pass=pass&&(dynamicAvoidSteps>=1||dynamicHoldCount>=1);end
if scenario.expectedNoDataHold,pass=pass&&noDataHoldCount>=1;end
if scenario.expectedPromotion,pass=pass&&promotionCount>=1;end
if strcmp(scenario.name,'INCREMENTAL_STATIC_INSERT')
    pass=pass&&dstarRepairExpanded>0&&astarScratchExpanded>dstarRepairExpanded;
end

summary=struct('goalReached',goalReached,'pass',pass,'failsafeTriggered',failsafe, ...
    'collisionCount',collisionCount,'geofenceViolationCount',geofenceViolationCount, ...
    'replanCount',replanCount,'dynamicAvoidSteps',dynamicAvoidSteps,'dynamicHoldCount',dynamicHoldCount, ...
    'noDataHoldCount',noDataHoldCount,'promotionCount',promotionCount,'localRefreshCount',localRefreshCount, ...
    'astarRecoveryCount',astarRecoveryCount,'dstarInitialExpanded',dstarInit.expanded, ...
    'dstarRepairExpanded',dstarRepairExpanded,'astarScratchExpanded',astarScratchExpanded, ...
    'timeToGoal_s',timeToGoal,'pathLength_m',pathLength,'minObstacleClearance_m',minObs, ...
    'minWallClearance_m',minWall,'minDynamicClearance_m',minDynamicClearance, ...
    'minPredictedMargin_m',minPredictedMargin,'maxSpeed_mps',maxSpeed, ...
    'maxTrackingError_m',maxTrackingError,'finalPosition',P(end,:));

log=struct('t',T,'p',P,'v',V,'stateId',stateId,'sensorValid',sensorValid, ...
    'predictedMargin',predMargin,'dynamicAvoid',dynAvoid,'pathHistory',{pathHistory}, ...
    'actualDynamic',actualDyn,'estimatedDynamic',estimatedDyn,'activeObstacleHistory',{activeObstacleHistory});
maps=struct('finalGrid',grid,'activeObstacles',activeObstacles,'planner',planner);
end

function path=prepare_path(grid,startXY,raw)
path=zeros(0,2);if isempty(raw),return;end
if norm(raw(1,:)-startXY)>grid.resolution/4,raw=[startXY;raw];end
raw=remove_duplicates(raw,grid.resolution/4);candidate=smooth_path_S2_2(grid,raw);
if path_valid(grid,candidate),path=candidate;elseif path_valid(grid,raw),path=raw;end
end

function tf=path_valid(grid,path)
tf=~isempty(path)&&size(path,1)>=2;if ~tf,return;end
for i=1:size(path,1)-1,if segment_occupied_grid_S2_2(grid,path(i,:),path(i+1,:)),tf=false;return;end,end
end

function p=remove_duplicates(p,tol)
if isempty(p),return;end
keep=true(size(p,1),1);last=1;
for i=2:size(p,1),if norm(p(i,:)-p(last,:))<tol,keep(i)=false;else,last=i;end,end
p=p(keep,:);
end

function tf=inside_windows(t,w)
tf=false;for i=1:size(w,1),if t>=w(i,1)&&t<=w(i,2),tf=true;return;end,end
end

function id=state_id(s)
switch s
    case 'TRACK',id=1;case 'DYNAMIC_AVOID',id=2;case 'DYNAMIC_HOLD',id=3;
    case 'NO_DATA_HOLD',id=4;case 'FAILSAFE',id=5;case 'COMPLETE',id=6;otherwise,id=0;
end
end

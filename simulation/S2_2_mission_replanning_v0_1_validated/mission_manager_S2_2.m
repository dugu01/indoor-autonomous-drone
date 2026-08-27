function [log, summary, maps] = mission_manager_S2_2(cfg, scenario)
% MISSION_MANAGER_S2_2  Stage S2.2 v0.1 finite-state mission simulation.
%
% Patch 2: separate "map changed" from "path blocked".  When a persistent
% unknown obstacle appears, the old plan is invalidated once, the vehicle
% hovers, replans on the updated costmap, and then tracks the new plan.
% This prevents a hover/replan oscillation caused by repeatedly checking a
% path that was already regenerated on the same unchanged map.

activeObstacles = scenario.knownObstacles;
mapVersion = 0;
grid = build_occupancy_grid_S2_2(cfg, activeObstacles);
[path, planInfo] = plan_path_safe_S2_2(cfg, grid, scenario.start, scenario.goal);
plannedMapVersion = mapVersion;

pos = scenario.start(:);
vel = [0;0];
t = 0;
pathIndex = 1;
state = 'TRACK';
if isempty(path)
    state = 'FAILSAFE';
end

hoverTimer = 0;
replanCount = 0;
hoverStopCount = 0;
collisionCount = 0;
geofenceViolationCount = 0;
failsafe = isempty(path);
goalReached = false;
timeToGoal = nan;
unknownInserted = false;
obstacleHitCount = 0;
maxSpeed = 0;
maxTrackingError = 0;

maxSteps = ceil(cfg.maxTime_s/cfg.dt) + 1;
T = nan(maxSteps,1); P = nan(maxSteps,2); V = nan(maxSteps,2);
stateId = nan(maxSteps,1); pathIdxLog = nan(maxSteps,1); blockedLog = false(maxSteps,1);
pathHistory = {path};
activeObstacleHistory = {activeObstacles};

k = 1;
T(k)=t; P(k,:)=pos.'; V(k,:)=vel.'; stateId(k)=state_id_S2_2(state); pathIdxLog(k)=pathIndex;

while t < cfg.maxTime_s
    k = k + 1;
    t = t + cfg.dt;

    mapChangedThisStep = false;
    if ~unknownInserted && t >= scenario.unknownAppearTime_s && ~isempty(scenario.unknownObstacles)
        obstacleHitCount = obstacleHitCount + 1;
        if obstacleHitCount >= cfg.obstaclePersistenceHits
            activeObstacles = [activeObstacles; scenario.unknownObstacles]; %#ok<AGROW>
            unknownInserted = true;
            mapVersion = mapVersion + 1;
            mapChangedThisStep = true;
            activeObstacleHistory{end+1} = activeObstacles; %#ok<AGROW>
        end
    end

    grid = build_occupancy_grid_S2_2(cfg, activeObstacles);
    blocked = false;

    switch state
        case 'FAILSAFE'
            failsafe = true;
            vel(:) = 0;

        case 'HOVER_REPLAN'
            vel(:) = 0;
            hoverTimer = hoverTimer + cfg.dt;
            if hoverTimer >= cfg.hoverTimeBeforeReplan_s
                [newPath, planInfo] = plan_path_safe_S2_2(cfg, grid, pos.', scenario.goal);
                if isempty(newPath)
                    state = 'FAILSAFE';
                    failsafe = true;
                    path = [];
                    pathIndex = 1;
                else
                    path = newPath;
                    pathHistory{end+1} = path; %#ok<AGROW>
                    pathIndex = 1;
                    plannedMapVersion = mapVersion;
                    state = 'TRACK';
                end
            end

        case 'TRACK'
            if isempty(path)
                state = 'FAILSAFE';
                failsafe = true;
            elseif norm(pos(:).'-scenario.goal) <= cfg.goalTolerance_m
                goalReached = true;
                timeToGoal = t;
                vel(:) = 0;
                state = 'COMPLETE';
            elseif mapVersion > plannedMapVersion || mapChangedThisStep
                % A new persistent obstacle invalidates the previous plan.
                % Hover first, then plan once on the updated inflated map.
                [blocked, ~] = collision_monitor_S2_2(grid, pos.', path, pathIndex, cfg.lookaheadDistance_m);
                state = 'HOVER_REPLAN';
                hoverTimer = 0;
                hoverStopCount = hoverStopCount + 1;
                replanCount = replanCount + 1;
                vel(:) = 0;
            else
                % The current path was generated on the current map.  It is
                % already validated in plan_path_safe_S2_2, so do not replan
                % repeatedly on the same map.  Unexpected dynamic-map updates
                % should be represented by increasing mapVersion above.
                [pos, vel, pathIndex, eTrack] = track_trajectory_S2_2(cfg,pos,vel,path,pathIndex);
                if isfinite(eTrack), maxTrackingError = max(maxTrackingError,eTrack); end
            end

        case 'COMPLETE'
            vel(:) = 0;
            break;
    end

    p = pos(:).';
    if continuous_collision_S2_2(p,cfg,activeObstacles)
        collisionCount = collisionCount + 1;
    end
    if p(1) < 0 || p(1) > cfg.room(1) || p(2) < 0 || p(2) > cfg.room(2)
        geofenceViolationCount = geofenceViolationCount + 1;
    end
    maxSpeed = max(maxSpeed,norm(vel));

    T(k)=t; P(k,:)=p; V(k,:)=vel.'; stateId(k)=state_id_S2_2(state); pathIdxLog(k)=pathIndex; blockedLog(k)=blocked;

    if strcmp(state,'FAILSAFE')
        break;
    end
end

validRows = isfinite(T) & all(isfinite(P),2) & all(isfinite(V),2) & isfinite(stateId);
T = T(validRows);
P = P(validRows,:);
V = V(validRows,:);
stateId = stateId(validRows);
pathIdxLog = pathIdxLog(validRows);
blockedLog = blockedLog(validRows);

[minWall, minObsRaw] = min_clearance_S2_2(P,cfg,activeObstacles);
pathLength = sum(sqrt(sum(diff(P,1,1).^2,2)));

if isempty(activeObstacles)
    obstacleClearancePass = true;
else
    obstacleClearancePass = minObsRaw >= cfg.inflationRadius - 1e-3;
end
wallClearancePass = minWall >= cfg.inflationRadius - 1e-3;
clearancePass = obstacleClearancePass && wallClearancePass;

if scenario.expectedGoalReached
    pass = goalReached && collisionCount==0 && geofenceViolationCount==0 && clearancePass;
    if scenario.expectedReplan
        pass = pass && replanCount >= 1 && hoverStopCount >= 1;
    end
elseif scenario.expectedFailsafe
    pass = ~goalReached && failsafe && collisionCount==0 && geofenceViolationCount==0 && wallClearancePass;
else
    pass = collisionCount==0 && geofenceViolationCount==0 && clearancePass;
end

summary = struct('goalReached',goalReached,'pass',pass,'collisionCount',collisionCount, ...
    'geofenceViolationCount',geofenceViolationCount,'replanCount',replanCount, ...
    'hoverStopCount',hoverStopCount,'failsafeTriggered',failsafe,'timeToGoal_s',timeToGoal, ...
    'pathLength_m',pathLength,'minObstacleClearance_m',minObsRaw, ...
    'minWallClearance_m',minWall,'clearancePass',clearancePass, ...
    'maxSpeed_mps',maxSpeed,'maxTrackingError_m',maxTrackingError,'finalPosition',P(end,:), ...
    'mapVersion',mapVersion,'plannedMapVersion',plannedMapVersion);

log = struct('t',T,'p',P,'v',V,'stateId',stateId,'pathIndex',pathIdxLog, ...
    'blocked',blockedLog,'pathHistory',{pathHistory},'activeObstacleHistory',{activeObstacleHistory});
maps = struct('finalGrid',grid,'initialPlanInfo',planInfo,'activeObstacles',activeObstacles);
end

function [path, info] = plan_path_safe_S2_2(cfg, grid, startXY, goalXY) %#ok<INUSD>
% PLAN_PATH_SAFE_S2_2  Plan then validate every smoothed segment.  If the
% shortcut-smoothed path is not valid, fall back to the raw A* grid path.
[pathRaw, info] = astar_grid_S2_2(grid, startXY, goalXY);
path = [];
if isempty(pathRaw)
    return;
end

pathRaw = remove_near_duplicates_S2_2([startXY; pathRaw], grid.resolution/4);
pathSmooth = smooth_path_S2_2(grid, pathRaw);
if path_valid_S2_2(grid,pathSmooth)
    path = pathSmooth;
elseif path_valid_S2_2(grid,pathRaw)
    path = pathRaw;
else
    info.success = false;
    info.reason = 'A* path failed post-validation';
    path = [];
end
end

function ok = path_valid_S2_2(grid,path)
ok = ~isempty(path) && size(path,1) >= 2;
if ~ok, return; end
for i = 1:size(path,1)-1
    if segment_occupied_grid_S2_2(grid,path(i,:),path(i+1,:))
        ok = false;
        return;
    end
end
end

function pathOut = remove_near_duplicates_S2_2(pathIn,tol)
if isempty(pathIn), pathOut = pathIn; return; end
keep = true(size(pathIn,1),1);
for i = 2:size(pathIn,1)
    if norm(pathIn(i,:)-pathIn(find(keep,1,'last'),:)) < tol
        keep(i) = false;
    end
end
pathOut = pathIn(keep,:);
end

function id = state_id_S2_2(state)
switch state
    case 'TRACK', id = 1;
    case 'HOVER_REPLAN', id = 2;
    case 'FAILSAFE', id = 3;
    case 'COMPLETE', id = 4;
    otherwise, id = 0;
end
end

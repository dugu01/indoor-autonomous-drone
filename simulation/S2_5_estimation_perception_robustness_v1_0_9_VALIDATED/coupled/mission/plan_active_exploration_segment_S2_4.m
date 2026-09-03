function [planner,path,traj,stats,jump,routeExists,meta,frontierState,request] = ...
    plan_active_exploration_segment_S2_4(cfg,c,grid,mapState,u,frontierState, ...
    est,estAcc,goalXY,initialScale,tNow)
% PLAN_ACTIVE_EXPLORATION_SEGMENT_S2_4 Direct target route or safe next view.
%
% The inherited S2.3 planner and trajectory generator remain authoritative
% for execution. S2.4 is used only when no known-free route to the mission
% target exists. It then proposes one known-free sensing viewpoint.
arguments
    cfg (1,1) struct
    c (1,1) struct
    grid (1,1) struct
    mapState (1,1) struct
    u (1,1) struct
    frontierState struct
    est (1,1) struct
    estAcc (3,1) double
    goalXY (1,2) double
    initialScale (1,1) double
    tNow (1,1) double
end
meta=struct('segmentTarget',goalXY(:).','isFinal',true,'frontierUsed',false, ...
    'goalCurrentlyReachable',false,'stopFeasible',false,'reason','', ...
    'explorationUsed',false,'selectedFrontierTrackId',uint64(0), ...
    'selectedCandidateId',uint64(0),'frontierDiagnostics',struct(), ...
    'viewpointDiagnostics',struct(),'decisionEvidence',struct());
request=exploration_request_S2_4(c,build_execution_grid_S2_4(grid),[], ...
    est.p(1:2).',tNow);

% First preserve the normal behaviour: if a direct known-free target route
% exists, plan it using the inherited D* Lite / A* / trajectory machinery.
[planner,path,traj,stats,jump,routeExists]=planDirect( ...
    cfg,grid,est,estAcc,goalXY,initialScale);
if routeExists
    meta.goalCurrentlyReachable=true;
    meta.reason='direct_goal';
    meta.stopFeasible=validate_known_free_stop_S2_3(cfg,grid,path,est.v(1:2));
    if ~meta.stopFeasible
        traj=invalidTraj();
    end
    return
end

% No direct route. Build the read-only uncertainty/frontier decision input.
g24=build_execution_grid_S2_4(grid);
uncertainty=project_uncertainty_2d_S2_4(u,mapState,c.nominalAltitude_m);
[frontierState,frontiers,fdiag]=extract_frontiers_incremental_S2_4( ...
    c,frontierState,g24);
[candidates,selected,vdiag]=generate_safe_viewpoints_S2_4( ...
    c,g24,frontiers,est.p(1:2).',goalXY,uncertainty);
meta.frontierDiagnostics=fdiag;
meta.viewpointDiagnostics=vdiag;
meta.decisionEvidence=summarize_viewpoint_decision_S2_4( ...
    candidates,selected,frontiers,g24);
if isempty(selected)
    planner=[];path=zeros(0,2);traj=invalidTraj();jump=inf(1,3);
    routeExists=false;meta.reason='no_safe_active_viewpoint';
    return
end

request=exploration_request_S2_4(c,g24,selected,est.p(1:2).',tNow);
requestStatus=validate_exploration_request_S2_4(c,g24,request,tNow);
if ~requestStatus.valid
    request.valid=false;request.statusReason=requestStatus.reason;
    planner=[];path=zeros(0,2);traj=invalidTraj();jump=inf(1,3);
    routeExists=false;meta.reason='active_viewpoint_request_invalid';
    return
end

meta.segmentTarget=request.positionXY;
meta.isFinal=false;
meta.frontierUsed=true;
meta.explorationUsed=true;
meta.selectedFrontierTrackId=request.frontierTrackId;
meta.selectedCandidateId=request.candidateId;
meta.reason='active_viewpoint';

% Convert the selected cell route to metric coordinates, preserving the
% exact estimated start state before smoothing and trajectory generation.
path=preparePath(grid,est.p(1:2).',request.knownFreeRouteXY);
[planner,~,dstat]=dstar_lite_S2_2('init',grid,est.p(1:2).',meta.segmentTarget);
stats=struct('dstarExpanded',dstat.expanded,'astarExpanded',0,'astarRecovery',0);
routeExists=~isempty(path);
[traj,jump]=makeTrajectory(cfg,grid,path,est,estAcc,initialScale);
meta.stopFeasible=routeExists && ...
    validate_known_free_stop_S2_3(cfg,grid,path,est.v(1:2));
if ~meta.stopFeasible
    traj=invalidTraj();
end
end

function [planner,path,traj,stats,jump,routeExists]=planDirect(cfg,grid,est,estAcc,target,initialScale)
[planner,raw,dstat]=dstar_lite_S2_2('init',grid,est.p(1:2).',target);
[rawA,aInfo]=astar_grid_S2_2(grid,est.p(1:2).',target);
path=preparePath(grid,est.p(1:2).',raw);
[traj,jump]=makeTrajectory(cfg,grid,path,est,estAcc,initialScale);
astarRecovery=0;
if (isempty(path)||~traj.valid)&&~isempty(rawA)
    path=preparePath(grid,est.p(1:2).',rawA);
    [traj,jump]=makeTrajectory(cfg,grid,path,est,estAcc,initialScale);
    astarRecovery=1;
end
routeExists=~isempty(raw)||~isempty(rawA);
stats=struct('dstarExpanded',dstat.expanded, ...
    'astarExpanded',aInfo.expanded,'astarRecovery',astarRecovery);
end

function [traj,jump]=makeTrajectory(cfg,grid,path,est,estAcc,initialScale)
traj=generate_strict_trajectory_S2_3( ...
    cfg,grid,path,est.v(1:2),estAcc(1:2),initialScale);
jump=inf(1,3);
if traj.valid
    r=sample_min_snap_state_S2_2(traj,0);
    jump=[norm(r.p-est.p(1:2)) norm(r.v-est.v(1:2)) ...
        norm(r.a-estAcc(1:2))];
end
end

function path=preparePath(grid,startXY,raw)
path=zeros(0,2);
if isempty(raw),return,end
startXY=double(startXY(:).');raw=double(raw);
if size(raw,2)~=2||any(~isfinite(raw(:)))||any(~isfinite(startXY)),return,end
raw=[startXY;raw];raw=removeDuplicates(raw,grid.resolution/4);raw(1,:)=startXY;
candidate=smooth_path_S2_2(grid,raw);
if ~isempty(candidate),candidate(1,:)=startXY;end
if pathValid(grid,candidate)
    path=candidate;
elseif pathValid(grid,raw)
    path=raw;
end
end
function tf=pathValid(grid,path)
tf=~isempty(path)&&size(path,1)>=2;
if ~tf,return,end
for k=1:size(path,1)-1
    if segment_occupied_grid_S2_2(grid,path(k,:),path(k+1,:))
        tf=false;return
    end
end
end
function p=removeDuplicates(p,tol)
if isempty(p),return,end
keep=true(size(p,1),1);last=1;
for k=2:size(p,1)
    if norm(p(k,:)-p(last,:))<tol
        keep(k)=false;
    else
        last=k;
    end
end
p=p(keep,:);
end
function tr=invalidTraj()
tr=struct('valid',false,'fallbackUsed',false,'timeScale',0, ...
    'continuity',zeros(1,4),'maxSpeed_mps',0,'maxAccel_mps2',0, ...
    'maxJerk_mps3',0);
end

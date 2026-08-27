function [planner,path,traj,stats,jump,routeExists,meta] = plan_unknown_segment_S2_3(cfg,grid,est,estAcc,goalXY,initialScale)
% PLAN_UNKNOWN_SEGMENT_S2_3 Direct known-free route or frontier segment.
meta=struct('segmentTarget',goalXY(:).','isFinal',true,'frontierUsed',false, ...
    'goalCurrentlyReachable',false,'stopFeasible',false,'reason','');
[planner,raw,dstat]=dstar_lite_S2_2('init',grid,est.p(1:2).',goalXY);
[rawA,aInfo]=astar_grid_S2_2(grid,est.p(1:2).',goalXY);
rawBest=raw;if isempty(rawBest),rawBest=rawA;end
if isempty(rawBest)
    frontier=select_goal_frontier_S2_3(cfg,grid,est.p(1:2).',goalXY);
    aInfo.expanded=aInfo.expanded+frontier.astarExpanded;
    if frontier.valid
        meta.segmentTarget=frontier.xy;meta.isFinal=false;meta.frontierUsed=true;
        meta.reason='frontier_segment';rawBest=frontier.path;
        [planner,~,dstat2]=dstar_lite_S2_2('init',grid,est.p(1:2).',meta.segmentTarget);
        dstat.expanded=dstat.expanded+dstat2.expanded;
    else
        path=zeros(0,2);traj=invalid_traj_local();jump=inf(1,3);routeExists=false;
        stats=struct('dstarExpanded',dstat.expanded,'astarExpanded',aInfo.expanded,'astarRecovery',0);
        meta.reason='no_reachable_frontier';return;
    end
else
    meta.goalCurrentlyReachable=true;meta.reason='direct_goal';
end
path=prepare_path(grid,est.p(1:2).',rawBest);
[traj,jump]=make_traj(cfg,grid,path,est,estAcc,initialScale);
astarRecovery=0;
if (isempty(path)||~traj.valid)&&~isempty(rawA)&&meta.isFinal
    path=prepare_path(grid,est.p(1:2).',rawA);[traj,jump]=make_traj(cfg,grid,path,est,estAcc,initialScale);astarRecovery=1;
end
routeExists=~isempty(path);
meta.stopFeasible=routeExists&&validate_known_free_stop_S2_3(cfg,grid,path,est.v(1:2));
if ~meta.stopFeasible,traj=invalid_traj_local();end
stats=struct('dstarExpanded',dstat.expanded,'astarExpanded',aInfo.expanded,'astarRecovery',astarRecovery);
end

function [traj,jump]=make_traj(cfg,grid,path,est,estAcc,initialScale)
traj=generate_strict_trajectory_S2_3(cfg,grid,path,est.v(1:2),estAcc(1:2),initialScale);jump=inf(1,3);
if traj.valid,r=sample_min_snap_state_S2_2(traj,0);jump=[norm(r.p-est.p(1:2)) norm(r.v-est.v(1:2)) norm(r.a-estAcc(1:2))];end
end
function path=prepare_path(grid,startXY,raw)
path=zeros(0,2);if isempty(raw),return;end
raw=[double(startXY(:).');double(raw)];raw=remove_duplicates(raw,grid.resolution/4);raw(1,:)=startXY(:).';
candidate=smooth_path_S2_2(grid,raw);if ~isempty(candidate),candidate(1,:)=startXY(:).';end
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
function tr=invalid_traj_local(),tr=struct('valid',false,'fallbackUsed',false,'timeScale',0,'continuity',zeros(1,4),'maxSpeed_mps',0,'maxAccel_mps2',0,'maxJerk_mps3',0);end

function world = truth_world_S2_3(cfg,scenario,t,context)
% TRUTH_WORLD_S2_3 Simulation-only environment state.
% This function must never be called from mapping, planning or mission
% decisions. It is permitted only by sensor generation and validation.
if nargin<4||isempty(context),context=struct();end
rects=scenario.truthStaticObstacles;
for i=1:numel(scenario.truthInsertedObstacles)
    if t>=scenario.truthInsertedObstacles(i).time
        rects=[rects;scenario.truthInsertedObstacles(i).rect5]; %#ok<AGROW>
    end
end
if isfield(context,'rtlObstacleActive')&&context.rtlObstacleActive&&scenario.rtlObstacle.enabled
    r=scenario.rtlObstacle.rect;
    rects=[rects;r(:).' 1.90]; %#ok<AGROW>
end
if isfield(context,'homeBlockActive')&&context.homeBlockActive&&scenario.truthHomeBlockAtRTL
    rects=[rects;scenario.truthHomeBlockRect5]; %#ok<AGROW>
end

dyn=struct('p',{},'v',{},'radius',{},'id',{},'stopped',{});
for i=1:numel(scenario.truthDynamicObstacles)
    [p,v,active]=dynamic_obstacle_state_S2_2(scenario.truthDynamicObstacles(i),t);
    if active
        dyn(end+1)=struct('p',p(:).','v',v(:).','radius', ...
            scenario.truthDynamicObstacles(i).radius,'id',i, ...
            'stopped',norm(v)<cfg.stoppedSpeedThreshold_mps); %#ok<AGROW>
    end
end
world=struct('staticRects5',rects,'dynamic',{dyn},'room',cfg.room,'timestamp',t);
end

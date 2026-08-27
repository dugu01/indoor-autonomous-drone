function videoFile = animate_S2_3_flight(cfg,scenario,log,summary,maps,resultsDir) %#ok<INUSD>
% ANIMATE_S2_3_FLIGHT 3-D truth/estimate/autonomous mission replay.
% Static obstacles are replayed using their insertion timestamps instead of
% drawing the final map from the beginning of the animation.
if ~exist(resultsDir,'dir'),mkdir(resultsDir);end
label=lower(regexprep(scenario.name,'[^A-Za-z0-9]+','_'));
versionLabel=lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_'));
videoFile=fullfile(resultsDir,sprintf('S2_3_%s_%s_animation.mp4',versionLabel,label));
fig=figure('Name',sprintf('S2.3 %s animation — %s',cfg.version,scenario.name), ...
    'NumberTitle','off','Color','w','Position',[80 50 1120 820]);
ax=axes(fig);hold(ax,'on');grid(ax,'on');axis(ax,'equal');
xlim(ax,[0 cfg.room(1)]);ylim(ax,[0 cfg.room(2)]);zlim(ax,[0 cfg.room(3)]);
view(ax,-35,25);xlabel(ax,'x [m]');ylabel(ax,'y [m]');zlabel(ax,'z [m]');

obstaclePatches=gobjects(0);
currentHistoryIndex=0;
[obstaclePatches,currentHistoryIndex]=update_static_obstacles( ...
    ax,log,0,obstaclePatches,currentHistoryIndex,cfg);

plot3(ax,scenario.start(1),scenario.start(2),log.truthP(1,3),'go','MarkerFaceColor','g');
plot3(ax,scenario.goal(1),scenario.goal(2),cfg.altitudeNominal_m,'rp','MarkerFaceColor','r','MarkerSize',11);
if isfield(maps,'selectedLandingXY')&&all(isfinite(maps.selectedLandingXY))
    plot3(ax,maps.selectedLandingXY(1),maps.selectedLandingXY(2),cfg.groundHeight_m, ...
        'md','MarkerFaceColor','m','MarkerSize',8);
end
hTruth=animatedline(ax,'Color',[.1 .1 .1],'LineWidth',2);
hEst=animatedline(ax,'Color',[0 .45 .74],'LineWidth',1.2);
hNow=plot3(ax,log.truthP(1,1),log.truthP(1,2),log.truthP(1,3),'bo','MarkerFaceColor','b','MarkerSize',8);
hEstNow=plot3(ax,log.estP(1,1),log.estP(1,2),log.estP(1,3),'kx','MarkerSize',8,'LineWidth',1.2);

nDyn=size(log.actualDynamic,2);
hDyn=gobjects(nDyn,1);
for j=1:nDyn
    hDyn(j)=plot3(ax,nan,nan,cfg.altitudeNominal_m,'o', ...
        'MarkerFaceColor',[.85 .25 .15],'MarkerEdgeColor',[.45 .05 .02], ...
        'MarkerSize',7,'HandleVisibility','off');
end

writer=[];
try
    writer=VideoWriter(videoFile,'MPEG-4');writer.FrameRate=20;open(writer);
catch ME
    warning('S2_3:VideoWriter','Could not open VideoWriter: %s',ME.message);writer=[];videoFile='';
end
step=max(1,round(0.10/cfg.dt));
for k=1:step:numel(log.t)
    [obstaclePatches,currentHistoryIndex]=update_static_obstacles( ...
        ax,log,log.t(k),obstaclePatches,currentHistoryIndex,cfg);
    addpoints(hTruth,log.truthP(k,1),log.truthP(k,2),log.truthP(k,3));
    addpoints(hEst,log.estP(k,1),log.estP(k,2),log.estP(k,3));
    set(hNow,'XData',log.truthP(k,1),'YData',log.truthP(k,2),'ZData',log.truthP(k,3));
    set(hEstNow,'XData',log.estP(k,1),'YData',log.estP(k,2),'ZData',log.estP(k,3));
    for j=1:nDyn
        q=squeeze(log.actualDynamic(k,j,:));
        if numel(q)==2&&all(isfinite(q))
            set(hDyn(j),'XData',q(1),'YData',q(2),'ZData',cfg.altitudeNominal_m);
        else
            set(hDyn(j),'XData',nan,'YData',nan,'ZData',nan);
        end
    end
    stateName=state_name(log.stateId(k),isfield(summary,'lifecycleEnabled')&&summary.lifecycleEnabled);
    armedText='';if isfield(log,'armed'),armedText=sprintf(' | armed %d',log.armed(k));end
    title(ax,sprintf('%s | t %.1f s | %s | lane %d%s',scenario.name,log.t(k),stateName,log.laneId(k),armedText), ...
        'Interpreter','none');
    drawnow;
    if ~isempty(writer),writeVideo(writer,getframe(fig));end
end
if ~isempty(writer),close(writer);end
end

function [patches,index]=update_static_obstacles(ax,log,t,patches,index,cfg)
if ~isfield(log,'activeObstacleHistory')||isempty(log.activeObstacleHistory)
    return;
end
if isfield(log,'activeObstacleHistoryTime')&&~isempty(log.activeObstacleHistoryTime)
    times=log.activeObstacleHistoryTime(:);
else
    times=zeros(numel(log.activeObstacleHistory),1);
end
newIndex=find(times<=t+1e-12,1,'last');
if isempty(newIndex),newIndex=1;end
if newIndex==index,return;end
if ~isempty(patches),delete(patches(isgraphics(patches)));end
obstacles=log.activeObstacleHistory{newIndex};
patches=gobjects(size(obstacles,1),1);
for i=1:size(obstacles,1)
    patches(i)=draw_box(ax,obstacles(i,:),0,min(1.6,cfg.room(3)),[.75 .75 .75]);
end
index=newIndex;
end

function name=state_name(id,lifecycle)
if lifecycle
    names={'PREFLIGHT','ARM','TAKEOFF','INITIAL HOVER','WAIT GOAL','PLAN OUTBOUND','TRACK OUTBOUND', ...
        'GOAL HOVER','PLAN RTL','TRACK RTL','LAND HOVER','LAND DESCENT','DISARM','COMPLETE', ...
        'PREFLIGHT REJECT','EMERGENCY HOLD','EMERGENCY LAND','REPLAN BRAKE','FAILSAFE','NAV DEG HOLD','SCAN HOLD','MAP DEG HOLD','GOAL UNREACHABLE'};
else
    names={'TRACK','DYNAMIC AVOID','DYNAMIC HOLD','NO DATA','FAILSAFE','COMPLETE','REJOIN','REPLAN BRAKE'};
end
if id>=1&&id<=numel(names),name=names{id};else,name=sprintf('STATE %d',id);end
end

function h=draw_box(ax,r,z0,z1,c)
x=[r(1) r(1)+r(3)];y=[r(2) r(2)+r(4)];
[X,Y,Z]=ndgrid(x,y,[z0 z1]);v=[X(:) Y(:) Z(:)];
f=[1 2 4 3;5 6 8 7;1 2 6 5;3 4 8 7;1 3 7 5;2 4 8 6];
h=patch(ax,'Vertices',v,'Faces',f,'FaceColor',c,'FaceAlpha',0.45, ...
    'EdgeColor',[.35 .35 .35],'HandleVisibility','off');
end

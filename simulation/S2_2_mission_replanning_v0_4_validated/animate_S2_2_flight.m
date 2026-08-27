function videoFile = animate_S2_2_flight(cfg,scenario,log,summary,maps,resultsDir) %#ok<INUSD>
% ANIMATE_S2_2_FLIGHT 3-D truth/estimate/replanning replay for v0.4.
if ~exist(resultsDir,'dir'),mkdir(resultsDir);end
label=lower(regexprep(scenario.name,'[^A-Za-z0-9]+','_'));
videoFile=fullfile(resultsDir,sprintf('S2_2_v0_4_%s_animation.mp4',label));
fig=figure('Name',sprintf('S2.2 v0.4 animation — %s',scenario.name), ...
    'NumberTitle','off','Color','w','Position',[80 50 1120 820]);
ax=axes(fig);hold(ax,'on');grid(ax,'on');axis(ax,'equal');
xlim(ax,[0 cfg.room(1)]);ylim(ax,[0 cfg.room(2)]);zlim(ax,[0 cfg.room(3)]);
view(ax,-35,25);xlabel(ax,'x [m]');ylabel(ax,'y [m]');zlabel(ax,'z [m]');
for i=1:size(maps.activeObstacles,1)
    r=maps.activeObstacles(i,:);draw_box(ax,r,0,min(1.6,cfg.room(3)),[.75 .75 .75]);
end
plot3(ax,scenario.start(1),scenario.start(2),cfg.altitudeNominal_m,'go','MarkerFaceColor','g');
plot3(ax,scenario.goal(1),scenario.goal(2),cfg.altitudeNominal_m,'rp','MarkerFaceColor','r','MarkerSize',11);
hTruth=animatedline(ax,'Color',[.1 .1 .1],'LineWidth',2);
hEst=animatedline(ax,'Color',[0 .45 .74],'LineWidth',1.2);
hNow=plot3(ax,log.truthP(1,1),log.truthP(1,2),log.truthP(1,3),'bo','MarkerFaceColor','b','MarkerSize',8);
hEstNow=plot3(ax,log.estP(1,1),log.estP(1,2),log.estP(1,3),'kx','MarkerSize',8,'LineWidth',1.2);
writer=[];
try
    writer=VideoWriter(videoFile,'MPEG-4');writer.FrameRate=20;open(writer);
catch ME
    warning('S2_2:VideoWriter','Could not open VideoWriter: %s',ME.message);writer=[];videoFile='';
end
step=max(1,round(0.10/cfg.dt));
for k=1:step:numel(log.t)
    addpoints(hTruth,log.truthP(k,1),log.truthP(k,2),log.truthP(k,3));
    addpoints(hEst,log.estP(k,1),log.estP(k,2),log.estP(k,3));
    set(hNow,'XData',log.truthP(k,1),'YData',log.truthP(k,2),'ZData',log.truthP(k,3));
    set(hEstNow,'XData',log.estP(k,1),'YData',log.estP(k,2),'ZData',log.estP(k,3));
    title(ax,sprintf('%s | t %.1f s | state %d | lane %d',scenario.name,log.t(k),log.stateId(k),log.laneId(k)),'Interpreter','none');
    drawnow;
    if ~isempty(writer),writeVideo(writer,getframe(fig));end
end
if ~isempty(writer),close(writer);end
end

function draw_box(ax,r,z0,z1,c)
x=[r(1) r(1)+r(3)];y=[r(2) r(2)+r(4)];
[X,Y,Z]=ndgrid(x,y,[z0 z1]);v=[X(:) Y(:) Z(:)];
f=[1 2 4 3;5 6 8 7;1 2 6 5;3 4 8 7;1 3 7 5;2 4 8 6];
patch(ax,'Vertices',v,'Faces',f,'FaceColor',c,'FaceAlpha',0.45,'EdgeColor',[.35 .35 .35],'HandleVisibility','off');
end

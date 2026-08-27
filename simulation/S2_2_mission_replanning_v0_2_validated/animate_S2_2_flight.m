function videoFile = animate_S2_2_flight(cfg, scenario, log, summary, maps, resultsDir) %#ok<INUSD>
% ANIMATE_S2_2_FLIGHT  Simple 2-D mission replay animation.
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end
videoFile = fullfile(resultsDir,sprintf('S2_2_%s_animation.mp4',lower(regexprep(scenario.name,'[^A-Za-z0-9]+','_'))));
fig = figure('Name','Stage S2.2 Mission Animation','Color','w','Position',[120 80 900 820]);
ax = axes(fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
xlim(ax,[0 cfg.room(1)]); ylim(ax,[0 cfg.room(2)]); xlabel(ax,'x [m]'); ylabel(ax,'y [m]');
title(ax,sprintf('Stage S2.2 animation — %s',scenario.name),'Interpreter','none');
rectangle(ax,'Position',[0 0 cfg.room(1) cfg.room(2)],'EdgeColor','k','LineWidth',1.5);
r = cfg.inflationRadius;
rectangle(ax,'Position',[r r cfg.room(1)-2*r cfg.room(2)-2*r],'EdgeColor',[0 0.55 0.15],'LineStyle','--');
for i=1:size(maps.activeObstacles,1)
    rectangle(ax,'Position',maps.activeObstacles(i,:),'FaceColor',[0.75 0.75 0.75],'EdgeColor',[0.2 0.2 0.2]);
end
plot(ax,scenario.start(1),scenario.start(2),'go','MarkerFaceColor','g');
plot(ax,scenario.goal(1),scenario.goal(2),'rp','MarkerSize',12,'MarkerFaceColor','r');
hTrail = animatedline(ax,'Color','b','LineWidth',2);
hNow = plot(ax,log.p(1,1),log.p(1,2),'bo','MarkerFaceColor','b','MarkerSize',8);

writer = [];
try
    writer = VideoWriter(videoFile,'MPEG-4');
    writer.FrameRate = 20;
    open(writer);
catch ME
    warning('S2_2:VideoWriter','Could not open VideoWriter: %s',ME.message);
    writer = [];
    videoFile = '';
end

step = max(1,round(0.25/cfg.dt));
for k = 1:step:numel(log.t)
    addpoints(hTrail,log.p(k,1),log.p(k,2));
    set(hNow,'XData',log.p(k,1),'YData',log.p(k,2));
    title(ax,sprintf('Stage S2.2 — %s | t %.1f s | state %d',scenario.name,log.t(k),log.stateId(k)),'Interpreter','none');
    drawnow;
    if ~isempty(writer)
        writeVideo(writer,getframe(fig));
    end
end
if ~isempty(writer), close(writer); end
end

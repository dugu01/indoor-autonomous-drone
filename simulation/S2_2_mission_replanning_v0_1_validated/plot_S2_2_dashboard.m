function plotFiles = plot_S2_2_dashboard(cfg, scenario, log, summary, maps, resultsDir)
% PLOT_S2_2_DASHBOARD  Plot Stage S2.2 v0.1 mission-replanning results.
% Compatible with older MATLAB releases where rectangle objects do not
% accept a DisplayName name-value argument.
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end
fig = figure('Name','Stage S2.2 Mission Replanning Dashboard','Color','w','Position',[80 60 1400 850]);
tl = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
title(tl,sprintf('Stage S2.2 v0.1 OMR-FailSafe — %s',scenario.name),'Interpreter','none');

% Map and path
ax = nexttile(tl); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
xlim(ax,[0 cfg.room(1)]); ylim(ax,[0 cfg.room(2)]);
xlabel(ax,'x [m]'); ylabel(ax,'y [m]'); title(ax,'Inflated map, plans and executed path','Interpreter','none');
rectangle(ax,'Position',[0 0 cfg.room(1) cfg.room(2)], ...
    'EdgeColor',[0.1 0.1 0.1],'LineWidth',1.5,'HandleVisibility','off');

r = cfg.inflationRadius;
rectangle(ax,'Position',[r r cfg.room(1)-2*r cfg.room(2)-2*r], ...
    'EdgeColor',[0.0 0.55 0.15],'LineStyle','--','LineWidth',1.2, ...
    'HandleVisibility','off');
plot(ax,nan,nan,'--','Color',[0.0 0.55 0.15],'LineWidth',1.2, ...
    'DisplayName','Centre geofence');

knownLegendDone = false;
for i=1:size(scenario.knownObstacles,1)
    draw_rect(ax,scenario.knownObstacles(i,:),[0.65 0.65 0.65], ...
        'Known obstacle',~knownLegendDone);
    knownLegendDone = true;
end

unknownLegendDone = false;
for i=1:size(scenario.unknownObstacles,1)
    draw_rect(ax,scenario.unknownObstacles(i,:),[0.95 0.55 0.25], ...
        'Unknown obstacle',~unknownLegendDone);
    unknownLegendDone = true;
end

nPlans = numel(log.pathHistory);
for k=1:nPlans
    pth = log.pathHistory{k};
    if isempty(pth), continue; end
    if k==1
        ls='--'; displayName='Initial plan'; showLegend='on';
    elseif k==nPlans
        ls='-.'; displayName='Final replan'; showLegend='on';
    else
        ls=':'; displayName='Intermediate replan'; showLegend='off';
    end
    plot(ax,pth(:,1),pth(:,2),ls,'LineWidth',1.2, ...
        'DisplayName',displayName,'HandleVisibility',showLegend);
end
plot(ax,log.p(:,1),log.p(:,2),'b-','LineWidth',2.0,'DisplayName','Executed');
plot(ax,scenario.start(1),scenario.start(2),'go','MarkerFaceColor','g','DisplayName','Start');
plot(ax,scenario.goal(1),scenario.goal(2),'rp','MarkerSize',12,'MarkerFaceColor','r','DisplayName','Goal');
legend(ax,'Location','bestoutside');

% Position and speed
ax = nexttile(tl); hold(ax,'on'); grid(ax,'on');
plot(ax,log.t,log.p(:,1),'DisplayName','x');
plot(ax,log.t,log.p(:,2),'DisplayName','y');
plot(ax,log.t,sqrt(sum(log.v.^2,2)),'DisplayName','speed');
yline(ax,cfg.maxSpeedXY_mps,'--','DisplayName','speed limit');
xlabel(ax,'Time [s]'); ylabel(ax,'m / m/s'); title(ax,'Position and speed','Interpreter','none'); legend(ax,'Location','best');

% State machine
ax = nexttile(tl); hold(ax,'on'); grid(ax,'on');
stairs(ax,log.t,log.stateId,'LineWidth',1.5);
yticks(ax,[1 2 3 4]); yticklabels(ax,{'TRACK','HOVER_REPLAN','FAILSAFE','COMPLETE'});
xlabel(ax,'Time [s]'); ylabel(ax,'Mission state');
title(ax,sprintf('Replans %d | Hover stops %d | Failsafe %d',summary.replanCount,summary.hoverStopCount,summary.failsafeTriggered),'Interpreter','none');

% Summary bars
ax = nexttile(tl); hold(ax,'on'); grid(ax,'on');
vals = [summary.minObstacleClearance_m, summary.minWallClearance_m, cfg.inflationRadius, summary.maxTrackingError_m];
bar(ax,vals); xticks(ax,1:4); xticklabels(ax,{'obs clear','wall clear','required','max track'}); ylabel(ax,'m');
title(ax,sprintf('PASS %d | collision %d | geofence %d',summary.pass,summary.collisionCount,summary.geofenceViolationCount),'Interpreter','none');
yline(ax,cfg.inflationRadius,'--','Required clearance');

plotFile = fullfile(resultsDir,sprintf('S2_2_%s_dashboard.png',lower(regexprep(scenario.name,'[^A-Za-z0-9]+','_'))));
exportgraphics(fig,plotFile,'Resolution',160);
plotFiles = {plotFile};
end

function draw_rect(ax,rectSpec,color,labelText,includeLegend)
rectangle(ax,'Position',rectSpec,'FaceColor',color, ...
    'EdgeColor',[0.2 0.2 0.2],'HandleVisibility','off');
if includeLegend
    plot(ax,nan,nan,'s','MarkerFaceColor',color,'MarkerEdgeColor',[0.2 0.2 0.2], ...
        'LineStyle','none','DisplayName',labelText);
end
end

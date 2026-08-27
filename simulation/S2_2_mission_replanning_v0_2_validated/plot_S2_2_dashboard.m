function plotFiles = plot_S2_2_dashboard(cfg,scenario,log,summary,maps,resultsDir) %#ok<INUSD>
% PLOT_S2_2_DASHBOARD  S2.2 v0.2 incremental/dynamic dashboard.
% All scenario dashboards are docked as tabs in one MATLAB Figures window.
% Each dashboard is saved as PNG and FIG in the versioned results folder.

if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

label = lower(regexprep(scenario.name,'[^A-Za-z0-9]+','_'));
figTag = sprintf('S2_2_%s_%s',lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_')),label);

% Prevent duplicate tabs when the same scenario is rerun.
oldFig = findall(groot,'Type','figure','Tag',figTag);
if ~isempty(oldFig), close(oldFig); end

fig = figure('Name',sprintf('S2.2 %s — %s',cfg.version,scenario.name), ...
    'Tag',figTag,'NumberTitle','off','Color','w');
try
    set(fig,'WindowStyle','docked');
catch ME
    warning('S2_2:DockedFigureUnavailable', ...
        'Could not dock the dashboard figure on this MATLAB setup: %s',ME.message);
    set(fig,'Position',[80 50 1450 880]);
end

% A single tab contains all related plots for this scenario.
tl = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
title(tl,sprintf('Stage S2.2 %s OMR-IDS — %s',cfg.version,scenario.name), ...
    'Interpreter','none');

ax=nexttile(tl);hold(ax,'on');grid(ax,'on');axis(ax,'equal');xlim(ax,[0 cfg.room(1)]);ylim(ax,[0 cfg.room(2)]);
xlabel(ax,'x [m]');ylabel(ax,'y [m]');title(ax,'Incremental plans, executed path and dynamic obstacles');
rectangle(ax,'Position',[0 0 cfg.room(1) cfg.room(2)],'EdgeColor',[0.1 0.1 0.1],'LineWidth',1.5,'HandleVisibility','off');
r=cfg.inflationRadius;rectangle(ax,'Position',[r r cfg.room(1)-2*r cfg.room(2)-2*r], ...
    'EdgeColor',[0 0.55 0.15],'LineStyle','--','LineWidth',1.2,'HandleVisibility','off');
plot(ax,nan,nan,'--','Color',[0 0.55 0.15],'DisplayName','Centre geofence');
for i=1:size(scenario.knownObstacles,1),draw_rect(ax,scenario.knownObstacles(i,:),[0.65 0.65 0.65]);end
plot(ax,nan,nan,'s','MarkerFaceColor',[0.65 0.65 0.65],'MarkerEdgeColor',[0.2 0.2 0.2],'LineStyle','none','DisplayName','Known/static obstacle');
for i=1:numel(log.activeObstacleHistory)
    obs=log.activeObstacleHistory{i};
    if size(obs,1)>size(scenario.knownObstacles,1)
        for j=size(scenario.knownObstacles,1)+1:size(obs,1),draw_rect(ax,obs(j,:),[0.95 0.55 0.25]);end
    end
end
if numel(log.activeObstacleHistory)>1,plot(ax,nan,nan,'s','MarkerFaceColor',[0.95 0.55 0.25],'MarkerEdgeColor',[0.2 0.2 0.2],'LineStyle','none','DisplayName','Inserted/promoted obstacle');end
nPlans=numel(log.pathHistory);
for i=1:nPlans
    p=log.pathHistory{i};if isempty(p),continue;end
    if i==1,hv='on';dn='Initial D* Lite path';ls='--';elseif i==nPlans,hv='on';dn='Latest path';ls='-.';else,hv='off';dn='Intermediate';ls=':';end
    plot(ax,p(:,1),p(:,2),ls,'LineWidth',1.0,'HandleVisibility',hv,'DisplayName',dn);
end
plot(ax,log.p(:,1),log.p(:,2),'b-','LineWidth',2,'DisplayName','Executed');
if ~isempty(log.actualDynamic)
    nd=size(log.actualDynamic,2);
    for j=1:nd
        q=squeeze(log.actualDynamic(:,j,:));valid=all(isfinite(q),2);
        if any(valid),plot(ax,q(valid,1),q(valid,2),'-','LineWidth',1.2,'DisplayName',sprintf('Dynamic %d truth',j));end
    end
end
plot(ax,scenario.start(1),scenario.start(2),'go','MarkerFaceColor','g','DisplayName','Start');
plot(ax,scenario.goal(1),scenario.goal(2),'rp','MarkerSize',12,'MarkerFaceColor','r','DisplayName','Goal');
legend(ax,'Location','bestoutside');

ax=nexttile(tl);hold(ax,'on');grid(ax,'on');
plot(ax,log.t,sqrt(sum(log.v.^2,2)),'DisplayName','Speed');
yline(ax,cfg.maxSpeedXY_mps,'--','DisplayName','Speed limit');
plot(ax,log.t,double(log.dynamicAvoid)*cfg.maxSpeedXY_mps,':','DisplayName','Dynamic avoidance active');
stairs(ax,log.t,double(log.sensorValid)*0.8*cfg.maxSpeedXY_mps,'-.','DisplayName','Obstacle data valid');
xlabel(ax,'Time [s]');ylabel(ax,'m/s / flags');title(ax,'Speed, dynamic avoidance and sensor coverage');legend(ax,'Location','best');

ax=nexttile(tl);hold(ax,'on');grid(ax,'on');
stairs(ax,log.t,log.stateId,'LineWidth',1.4);
yticks(ax,1:6);yticklabels(ax,{'TRACK','DYN AVOID','DYN HOLD','NO-DATA HOLD','FAILSAFE','COMPLETE'});
xlabel(ax,'Time [s]');ylabel(ax,'Mission state');
title(ax,sprintf('Avoid steps %d | Replans %d | Promotions %d | No-data holds %d', ...
    summary.dynamicAvoidSteps,summary.replanCount,summary.promotionCount,summary.noDataHoldCount),'Interpreter','none');

ax=nexttile(tl);hold(ax,'on');grid(ax,'on');
vals=[summary.minObstacleClearance_m,summary.minWallClearance_m,summary.minDynamicClearance_m, ...
    cfg.inflationRadius,summary.dstarRepairExpanded,summary.astarScratchExpanded];
yyaxis(ax,'left');bar(ax,1:4,vals(1:4));ylabel(ax,'Clearance [m]');
yline(ax,cfg.inflationRadius,'--','Static required');yline(ax,0,':','Physical collision boundary');
yyaxis(ax,'right');bar(ax,5:6,vals(5:6));ylabel(ax,'Expanded nodes');
xticks(ax,1:6);xticklabels(ax,{'obs','wall','dynamic','static req.','D* repair','A* scratch'});
title(ax,sprintf('PASS %d | collision %d | geofence %d',summary.pass,summary.collisionCount,summary.geofenceViolationCount));

baseName=sprintf('S2_2_%s_%s_dashboard',lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_')),label);
pngFile=fullfile(resultsDir,[baseName '.png']);
figFile=fullfile(resultsDir,[baseName '.fig']);
exportgraphics(fig,pngFile,'Resolution',160);
try
    savefig(fig,figFile);
    plotFiles={pngFile,figFile};
catch ME
    warning('S2_2:SaveFigFailed','PNG saved, but FIG save failed: %s',ME.message);
    plotFiles={pngFile};
end
end

function draw_rect(ax,r,c)
rectangle(ax,'Position',r,'FaceColor',c,'EdgeColor',[0.2 0.2 0.2],'HandleVisibility','off');
end

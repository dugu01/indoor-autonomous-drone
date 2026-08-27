function plotFiles = plot_S2_2_dashboard(cfg,scenario,log,summary,maps,resultsDir) %#ok<INUSD>
% PLOT_S2_2_DASHBOARD One docked tab per scenario, saved by version.
if ~exist(resultsDir,'dir'),mkdir(resultsDir);end
label=lower(regexprep(scenario.name,'[^A-Za-z0-9]+','_'));figTag=sprintf('S2_2_%s_%s',lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_')),label);
oldFig=findall(groot,'Type','figure','Tag',figTag);if ~isempty(oldFig),close(oldFig);end
fig=figure('Name',sprintf('S2.2 %s — %s',cfg.version,scenario.name),'Tag',figTag,'NumberTitle','off','Color','w');
try,set(fig,'WindowStyle','docked');catch,set(fig,'Position',[80 40 1500 920]);end
tl=tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
title(tl,sprintf('Stage S2.2 %s OMR-IDS-MS — %s',cfg.version,scenario.name),'Interpreter','none');
ax=nexttile(tl);hold(ax,'on');grid(ax,'on');axis(ax,'equal');xlim(ax,[0 cfg.room(1)]);ylim(ax,[0 cfg.room(2)]);
rectangle(ax,'Position',[0 0 cfg.room(1) cfg.room(2)],'EdgeColor',[.1 .1 .1],'LineWidth',1.5,'HandleVisibility','off');
r=cfg.inflationRadius;rectangle(ax,'Position',[r r cfg.room(1)-2*r cfg.room(2)-2*r],'EdgeColor',[0 .55 .15],'LineStyle','--','HandleVisibility','off');
plot(ax,nan,nan,'--','Color',[0 .55 .15],'DisplayName','Centre geofence');
for i=1:size(scenario.knownObstacles,1),draw_rect(ax,scenario.knownObstacles(i,:),[.65 .65 .65]);end
plot(ax,nan,nan,'s','MarkerFaceColor',[.65 .65 .65],'MarkerEdgeColor',[.2 .2 .2],'LineStyle','none','DisplayName','Static obstacle');
for i=1:numel(log.pathHistory),p=log.pathHistory{i};if ~isempty(p),plot(ax,p(:,1),p(:,2),':','HandleVisibility','off');end,end
for i=1:numel(log.trajectoryHistory)
    tr=log.trajectoryHistory{i};if isstruct(tr)&&isfield(tr,'valid')&&tr.valid
        hv='off';dn='Reference trajectory';if i==numel(log.trajectoryHistory),hv='on';end
        plot(ax,tr.sample.p(:,1),tr.sample.p(:,2),'--','LineWidth',1.1,'HandleVisibility',hv,'DisplayName',dn);
    end
end
plot(ax,log.p(:,1),log.p(:,2),'b-','LineWidth',2,'DisplayName','Executed');
if ~isempty(log.actualDynamic),for j=1:size(log.actualDynamic,2),q=squeeze(log.actualDynamic(:,j,:));v=all(isfinite(q),2);if any(v),plot(ax,q(v,1),q(v,2),'-','DisplayName',sprintf('Dynamic %d',j));end,end,end
plot(ax,scenario.start(1),scenario.start(2),'go','MarkerFaceColor','g','DisplayName','Start');plot(ax,scenario.goal(1),scenario.goal(2),'rp','MarkerSize',11,'MarkerFaceColor','r','DisplayName','Goal');
xlabel(ax,'x [m]');ylabel(ax,'y [m]');title(ax,'Planned polynomial trajectories and executed path');legend(ax,'Location','bestoutside');
ax=nexttile(tl);hold(ax,'on');grid(ax,'on');plot(ax,log.t,vecnorm(log.v,2,2),'DisplayName','speed');plot(ax,log.t,vecnorm(log.a,2,2),'DisplayName','acceleration');plot(ax,log.t,vecnorm(log.j,2,2),'DisplayName','jerk');
yline(ax,cfg.maxSpeedXY_mps,'--','DisplayName','v max');yline(ax,cfg.maxAccelXY_mps2,'--','DisplayName','a max');yline(ax,cfg.maxJerkXY_mps3,'--','DisplayName','j max');xlabel(ax,'Time [s]');ylabel(ax,'Magnitude');title(ax,'Executed kinematic limits');legend(ax,'Location','best');
ax=nexttile(tl);hold(ax,'on');grid(ax,'on');
plot(ax,log.t,log.trackingError,'DisplayName','reference deviation: all modes');
plot(ax,log.t,log.trackingValidationError,'LineWidth',1.5,'DisplayName','validated TRACK error');
yline(ax,cfg.maxTrajectoryTrackingError_m,'--','DisplayName','tracking threshold');
xlabel(ax,'Time [s]');ylabel(ax,'Error [m]');
title(ax,sprintf('TRACK %.3f m | safety override %.3f m', ...
    summary.maxTrackingError_m,summary.maxSafetyOverrideDeviation_m));legend(ax,'Location','best');
ax=nexttile(tl);stairs(ax,log.t,log.stateId,'LineWidth',1.3);grid(ax,'on');yticks(ax,1:7);
yticklabels(ax,{'TRACK','DYN AVOID','DYN HOLD','NO-DATA HOLD','FAILSAFE','COMPLETE','REJOIN'});
xlabel(ax,'Time [s]');ylabel(ax,'Mission state');
title(ax,sprintf('Replans %d | avoids %d | rejoins %d', ...
    summary.replanCount,summary.dynamicAvoidSteps,summary.rejoinCount));
ax=nexttile(tl);dynBar=summary.minDynamicClearance_m;if ~isfinite(dynBar),dynBar=nan;end
vals=[summary.minObstacleClearance_m summary.minWallClearance_m dynBar cfg.inflationRadius];bar(ax,vals);grid(ax,'on');xticklabels(ax,{'obstacle','wall','dynamic','static req.'});ylabel(ax,'Clearance [m]');title(ax,sprintf('Collision %d | geofence %d',summary.collisionCount,summary.geofenceViolationCount));
if ~isfinite(summary.minDynamicClearance_m),text(ax,3,0.05,'N/A','HorizontalAlignment','center','VerticalAlignment','bottom');end
ax=nexttile(tl);bar(ax,[summary.trajectoryGenerationCount summary.trajectoryFallbackCount summary.maxTrajectoryTimeScale]);grid(ax,'on');xticklabels(ax,{'generated','fallbacks','max time scale'});title(ax,sprintf('C3 jump %.1e | replan v jump %.1e',summary.maxReferenceContinuityJump(4),summary.maxReplanStateJump(2)));
baseName=sprintf('S2_2_%s_%s_dashboard',lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_')),label);pngFile=fullfile(resultsDir,[baseName '.png']);figFile=fullfile(resultsDir,[baseName '.fig']);
exportgraphics(fig,pngFile,'Resolution',160);try,savefig(fig,figFile);plotFiles={pngFile,figFile};catch,plotFiles={pngFile};end
end
function draw_rect(ax,r,c),rectangle(ax,'Position',r,'FaceColor',c,'EdgeColor',[.2 .2 .2],'HandleVisibility','off');end

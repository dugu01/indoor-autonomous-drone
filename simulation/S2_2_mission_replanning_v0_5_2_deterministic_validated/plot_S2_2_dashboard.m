function plotFiles = plot_S2_2_dashboard(cfg,scenario,log,summary,maps,resultsDir)
% PLOT_S2_2_DASHBOARD Stage S2.2 v0.5.2 tabbed mission dashboard.
% Each scenario is one docked MATLAB figure tab containing eight panels.
% PNG and editable FIG copies are saved in the version/scenario result folder.

if ~exist(resultsDir,'dir'), mkdir(resultsDir); end
label=lower(regexprep(scenario.name,'[^A-Za-z0-9]+','_'));
versionLabel=lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_'));
figTag=sprintf('S2_2_%s_%s',versionLabel,label);
oldFig=findall(groot,'Type','figure','Tag',figTag);
if ~isempty(oldFig), close(oldFig); end

fig=figure('Name',sprintf('S2.2 %s — %s',cfg.version,scenario.name), ...
    'Tag',figTag,'NumberTitle','off','Color','w');
try
    set(fig,'WindowStyle','docked');
catch
    set(fig,'Position',[50 30 1540 920]);
end

tl=tiledlayout(fig,4,2,'TileSpacing','compact','Padding','compact');
title(tl,sprintf('Stage S2.2 %s — autonomous mission, estimation and replanning — %s', ...
    cfg.version,scenario.name),'Interpreter','none','FontWeight','bold');

cTruth=[0.10 0.10 0.10];
cEst=[0.00 0.45 0.74];
cRef=[0.85 0.33 0.10];
cWarn=[0.75 0.10 0.10];
cSafe=[0.00 0.55 0.20];

%% 1. XY mission map
ax=nexttile(tl);hold(ax,'on');grid(ax,'on');axis(ax,'equal');
xlim(ax,[0 cfg.room(1)]);ylim(ax,[0 cfg.room(2)]);
rectangle(ax,'Position',[0 0 cfg.room(1) cfg.room(2)], ...
    'EdgeColor',[.1 .1 .1],'LineWidth',1.5,'HandleVisibility','off');
r=max(log.inflationRadius(1),cfg.baseInflationRadius);
if cfg.room(1)>2*r && cfg.room(2)>2*r
    rectangle(ax,'Position',[r r cfg.room(1)-2*r cfg.room(2)-2*r], ...
        'EdgeColor',cSafe,'LineStyle','--','LineWidth',1.1,'HandleVisibility','off');
end
for i=1:size(maps.activeObstacles,1)
    rectangle(ax,'Position',maps.activeObstacles(i,:), ...
        'FaceColor',[.76 .76 .76],'EdgeColor',[.25 .25 .25], ...
        'HandleVisibility','off');
end
for i=1:numel(log.pathHistory)
    p=log.pathHistory{i};
    if ~isempty(p),plot(ax,p(:,1),p(:,2),':','Color',[.55 .55 .55],'HandleVisibility','off');end
end
for i=1:numel(log.trajectoryHistory)
    tr=log.trajectoryHistory{i};
    if isstruct(tr)&&isfield(tr,'valid')&&tr.valid&&isfield(tr,'sample')
        plot(ax,tr.sample.p(:,1),tr.sample.p(:,2),'--','Color',cRef, ...
            'LineWidth',1.0,'HandleVisibility','off');
    end
end
hTruth=plot(ax,log.truthP(:,1),log.truthP(:,2),'-','Color',cTruth,'LineWidth',2.0);
hEst=plot(ax,log.estP(:,1),log.estP(:,2),'-','Color',cEst,'LineWidth',1.2);
hRef=plot(ax,log.pRef(:,1),log.pRef(:,2),'--','Color',cRef,'LineWidth',1.0);
hStart=plot(ax,scenario.start(1),scenario.start(2),'go','MarkerFaceColor','g','MarkerSize',6);
hGoal=plot(ax,scenario.goal(1),scenario.goal(2),'rp','MarkerFaceColor','r','MarkerSize',10);
hLanding=[];
if isfield(maps,'selectedLandingXY')&&all(isfinite(maps.selectedLandingXY))
    hLanding=plot(ax,maps.selectedLandingXY(1),maps.selectedLandingXY(2),'md', ...
        'MarkerFaceColor','m','MarkerSize',7);
end
hDyn=[];
if ~isempty(log.actualDynamic)
    for j=1:size(log.actualDynamic,2)
        q=squeeze(log.actualDynamic(:,j,:));v=all(isfinite(q),2);
        if any(v)
            h=plot(ax,q(v,1),q(v,2),'-','LineWidth',1.1);
            if isempty(hDyn),hDyn=h;end
        end
    end
end
plot(ax,nan,nan,'--','Color',cSafe,'LineWidth',1.1,'HandleVisibility','off');
xlabel(ax,'x [m]');ylabel(ax,'y [m]');title(ax,'XY planning, truth and selected local estimate');
handles=[hTruth hEst hRef hStart hGoal];names={'Truth','Selected ESKF','Controller reference','Start/Home','Goal'};
if ~isempty(hLanding),handles=[handles hLanding];names{end+1}='Selected landing';end %#ok<AGROW>
if ~isempty(hDyn),handles=[handles hDyn];names{end+1}='Dynamic obstacle';end %#ok<AGROW>
legend(ax,handles,names,'Location','bestoutside');

%% 2. 3-D trajectory and altitude
ax=nexttile(tl);hold(ax,'on');grid(ax,'on');
plot3(ax,log.truthP(:,1),log.truthP(:,2),log.truthP(:,3),'-','Color',cTruth,'LineWidth',1.8);
plot3(ax,log.estP(:,1),log.estP(:,2),log.estP(:,3),'-','Color',cEst,'LineWidth',1.1);
plot3(ax,log.pRef(:,1),log.pRef(:,2),log.pRef(:,3),'--','Color',cRef,'LineWidth',1.0);
view(ax,-35,25);axis(ax,'equal');xlim(ax,[0 cfg.room(1)]);ylim(ax,[0 cfg.room(2)]);zlim(ax,[0 cfg.room(3)]);
xlabel(ax,'x [m]');ylabel(ax,'y [m]');zlabel(ax,'z [m]');title(ax,'6-DOF translational trajectory');
legend(ax,{'Truth','Selected ESKF','Reference'},'Location','best');

%% 3. Estimator position and attitude error
ax=nexttile(tl);hold(ax,'on');grid(ax,'on');
plot(ax,log.t,log.estimatorPositionError,'Color',cEst,'LineWidth',1.2);
if scenario.expectedFailsafe
    yline(ax,cfg.maxEstimatorFailsafeBound_m,'--','Color',cWarn,'LineWidth',1.0);
else
    yline(ax,cfg.maxEstimatorPositionError_m,'--','Color',cWarn,'LineWidth',1.0);
end
xlabel(ax,'Time [s]');ylabel(ax,'Position error [m]');
yyaxis(ax,'right');plot(ax,log.t,log.estimatorAttitudeError_deg,'Color',cRef,'LineWidth',1.0);
yline(ax,cfg.maxEstimatorAttitudeError_deg,':','Color',cWarn,'LineWidth',1.0);
ylabel(ax,'Attitude error [deg]');
title(ax,sprintf('Estimator max %.3f m / %.2f deg',summary.maxEstimatorPositionError_m,summary.maxEstimatorAttitudeError_deg));

%% 4. Tracking, altitude and tilt
ax=nexttile(tl);hold(ax,'on');grid(ax,'on');
plot(ax,log.t,log.trackingError,'Color',cTruth,'LineWidth',1.0);
if isfield(summary,'lifecycleEnabled')&&summary.lifecycleEnabled
    altErr=abs(log.truthP(:,3)-log.pRef(:,3));
else
    altErr=abs(log.truthP(:,3)-cfg.altitudeNominal_m);
end
plot(ax,log.t,altErr,'Color',cEst,'LineWidth',1.0);
tiltDeg=rad2deg(vecnorm(log.truthRpy(:,1:2),2,2));
plot(ax,log.t,tiltDeg/10,'Color',cRef,'LineWidth',1.0);
yline(ax,cfg.maxPositionTrackingError_m,'--','Color',cWarn,'LineWidth',1.0);
xlabel(ax,'Time [s]');ylabel(ax,'Error [m], tilt/10');
title(ax,sprintf('Track %.3f m | altitude %.3f m | tilt %.2f deg', ...
    summary.maxTrackingError_m,summary.maxAltitudeError_m,summary.maxTilt_deg));
legend(ax,{'Reference deviation','Altitude error','Tilt [deg]/10','tracking limit'},'Location','best');

%% 5. Mission state and selected estimator lane
ax=nexttile(tl);hold(ax,'on');grid(ax,'on');
stairs(ax,log.t,log.stateId,'LineWidth',1.3,'Color',cTruth);
stairs(ax,log.t,log.laneId+0.08,'LineWidth',1.1,'Color',cEst);
if isfield(summary,'lifecycleEnabled')&&summary.lifecycleEnabled
    yticks(ax,1:20);
    yticklabels(ax,{'PREFLIGHT','ARM','TAKEOFF','INIT HOVER','WAIT GOAL','PLAN OUT','TRACK OUT', ...
        'GOAL HOVER','PLAN RTL','TRACK RTL','LAND HOVER','LAND','DISARM','COMPLETE', ...
        'PREFLIGHT REJECT','EMERG HOLD','EMERG LAND','REPLAN BRAKE','FAILSAFE','NAV DEG HOLD'});
else
    yticks(ax,1:8);yticklabels(ax,{'TRACK','DYN AVOID','DYN HOLD','NO-DATA','FAILSAFE','COMPLETE','REJOIN','REPLAN BRAKE'});
end
xlabel(ax,'Time [s]');ylabel(ax,'State / lane+0.08');
title(ax,sprintf('Lane %d final | %d switches | %d replans',summary.activeLaneFinal,summary.laneSwitches,summary.replanCount));
legend(ax,{'Mission state','Selected lane + 0.08'},'Location','best');

%% 6. Covariance-aware inflation and safety margins
ax=nexttile(tl);hold(ax,'on');grid(ax,'on');
plot(ax,log.t,log.inflationRadius,'Color',cWarn,'LineWidth',1.3);
plot(ax,log.t,cfg.baseInflationRadius+cfg.uncertaintySigmaGain*log.xySigma, ...
    '--','Color',cEst,'LineWidth',1.0);
yline(ax,cfg.baseInflationRadius,':','Color',cSafe,'LineWidth',1.1);
yline(ax,cfg.maxInflationRadius,'--','Color',[.45 .1 .55],'LineWidth',1.0);
xlabel(ax,'Time [s]');ylabel(ax,'Radius [m]');
title(ax,sprintf('Inflation max %.3f m | min obs/wall %.3f/%.3f m', ...
    summary.maxInflationRadius_m,summary.minObstacleClearance_m,summary.minWallClearance_m));
legend(ax,{'Applied inflation','Base + 2\sigma_{xy}','Base radius','Cap'},'Location','best');

%% 7. Controller effort and executed kinematics
ax=nexttile(tl);hold(ax,'on');grid(ax,'on');
plot(ax,log.t,log.thrust_N,'LineWidth',1.1);
plot(ax,log.t,vecnorm(log.moment_Nm,2,2),'LineWidth',1.0);
plot(ax,log.t,vecnorm(log.truthV(:,1:2),2,2),'LineWidth',1.0);
plot(ax,log.t,vecnorm(log.truthA(:,1:2),2,2),'LineWidth',1.0);
plot(ax,log.t,abs(log.truthV(:,3)),'--','LineWidth',0.9);
plot(ax,log.t,abs(log.truthA(:,3)),':','LineWidth',0.9);
xlabel(ax,'Time [s]');ylabel(ax,'Mixed units');
if isfield(summary,'maxExecutedVerticalSpeed_mps')
    title(ax,sprintf('XY v/a/j %.2f/%.2f/%.2f | Z v/a/j %.2f/%.2f/%.2f', ...
        summary.maxExecutedSpeed_mps,summary.maxExecutedAccel_mps2,summary.maxExecutedJerk_mps3, ...
        summary.maxExecutedVerticalSpeed_mps,summary.maxExecutedVerticalAccel_mps2,summary.maxExecutedVerticalJerk_mps3));
else
    title(ax,sprintf('Executed v/a/j %.2f / %.2f / %.2f', ...
        summary.maxExecutedSpeed_mps,summary.maxExecutedAccel_mps2,summary.maxExecutedJerk_mps3));
end
legend(ax,{'Thrust [N]','|Moment| [N m]','XY speed [m/s]','XY accel [m/s^2]', ...
    '|Z speed| [m/s]','|Z accel| [m/s^2]'},'Location','best');

%% 8. Lane health and aid availability
ax=nexttile(tl);hold(ax,'on');grid(ax,'on');
plot(ax,log.t,log.laneScores,'LineWidth',0.9);
ymax=max(log.laneScores(isfinite(log.laneScores)));
if isempty(ymax)||~isfinite(ymax),ymax=5;end
scale=max(1,ymax/1.5);
stairs(ax,log.t,double(log.sensorAids)*scale,'LineWidth',0.8);
xlabel(ax,'Time [s]');ylabel(ax,'Lane score / scaled aid flag');
title(ax,'Lane health and VIO/LiDAR/range/barometer availability');
legend(ax,{'Lane 1','Lane 2','Lane 3','Lane 4','VIO','LiDAR','Range','Baro'},'Location','bestoutside');

baseName=sprintf('S2_2_%s_%s_dashboard',versionLabel,label);
pngFile=fullfile(resultsDir,[baseName '.png']);
figFile=fullfile(resultsDir,[baseName '.fig']);
drawnow;
try
    exportgraphics(fig,pngFile,'Resolution',170);
catch
    print(fig,pngFile,'-dpng','-r170');
end
try
    savefig(fig,figFile);plotFiles={pngFile,figFile};
catch
    plotFiles={pngFile};
end
end

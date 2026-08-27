function plotFiles = plot_S2_3_dashboard(cfg,scenario,log,summary,maps,resultsDir)
% PLOT_S2_3_DASHBOARD One MATLAB figure with tabbed S2.3 diagnostics.
if ~exist(resultsDir,'dir'),mkdir(resultsDir);end
label=lower(regexprep(scenario.name,'[^A-Za-z0-9]+','_'));
versionLabel=lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_'));
tag=sprintf('S2_3_%s_%s',versionLabel,label);old=findall(groot,'Type','figure','Tag',tag);if ~isempty(old),close(old);end
fig=figure('Name',sprintf('S2.3 %s — %s',cfg.version,scenario.name),'Tag',tag, ...
    'NumberTitle','off','Color','w','Position',[40 30 1500 900]);
try,set(fig,'WindowStyle','docked');catch,end
G=uitabgroup(fig);

% Mission and final map.
tab=uitab(G,'Title','Mission & map');
ax1=axes(tab,'Position',[.055 .10 .42 .82]);hold(ax1,'on');grid(ax1,'on');axis(ax1,'equal');
imagesc(ax1,maps.finalGrid.xs,maps.finalGrid.ys,double(maps.finalGrid.occ));set(ax1,'YDir','normal');
plot(ax1,log.truthP(:,1),log.truthP(:,2),'k-','LineWidth',1.8);
plot(ax1,log.estP(:,1),log.estP(:,2),'-','LineWidth',1.1);
plot(ax1,scenario.start(1),scenario.start(2),'go','MarkerFaceColor','g');
plot(ax1,scenario.goal(1),scenario.goal(2),'rp','MarkerFaceColor','r','MarkerSize',10);
plot(ax1,maps.selectedLandingXY(1),maps.selectedLandingXY(2),'md','MarkerFaceColor','m');
xlabel(ax1,'x [m]');ylabel(ax1,'y [m]');title(ax1,'Final fail-closed planner map and executed mission');
legend(ax1,{'Blocked/unknown','Truth','Selected ESKF','Home','Goal','Landing'},'Location','bestoutside');
ax2=axes(tab,'Position',[.56 .10 .40 .82]);hold(ax2,'on');grid(ax2,'on');
plot3(ax2,log.truthP(:,1),log.truthP(:,2),log.truthP(:,3),'k-','LineWidth',1.6);
plot3(ax2,log.estP(:,1),log.estP(:,2),log.estP(:,3),'-','LineWidth',1.0);
plot3(ax2,log.pRef(:,1),log.pRef(:,2),log.pRef(:,3),'--','LineWidth',1.0);
view(ax2,-35,25);axis(ax2,'equal');xlim(ax2,[0 cfg.room(1)]);ylim(ax2,[0 cfg.room(2)]);zlim(ax2,[0 cfg.room(3)]);
xlabel(ax2,'x');ylabel(ax2,'y');zlabel(ax2,'z');title(ax2,'6-DOF truth, estimate and reference');legend(ax2,{'Truth','Estimate','Reference'});

% Belief layers.
tab=uitab(G,'Title','Occupancy belief');
ax1=axes(tab,'Position',[.055 .10 .28 .82]);imagesc(ax1,maps.finalGrid.xs,maps.finalGrid.ys,double(maps.finalGrid.knownFree));set(ax1,'YDir','normal');axis(ax1,'equal');title(ax1,'Known free');xlabel(ax1,'x');ylabel(ax1,'y');colorbar(ax1);
ax2=axes(tab,'Position',[.37 .10 .28 .82]);imagesc(ax2,maps.finalGrid.xs,maps.finalGrid.ys,double(maps.finalGrid.staticOccupied));set(ax2,'YDir','normal');axis(ax2,'equal');title(ax2,'Static occupied');xlabel(ax2,'x');ylabel(ax2,'y');colorbar(ax2);
ax3=axes(tab,'Position',[.685 .10 .28 .82]);imagesc(ax3,maps.finalGrid.xs,maps.finalGrid.ys,double(maps.finalGrid.dynamicOccupied));set(ax3,'YDir','normal');axis(ax3,'equal');title(ax3,'Temporary dynamic');xlabel(ax3,'x');ylabel(ax3,'y');colorbar(ax3);

% Map evolution.
tab=uitab(G,'Title','Map evolution');
ax1=axes(tab,'Position',[.07 .56 .88 .35]);hold(ax1,'on');grid(ax1,'on');stairs(ax1,log.t,log.mapVersion,'LineWidth',1.2);ylabel(ax1,'Map version');title(ax1,'Versioned occupancy changes');
ax2=axes(tab,'Position',[.07 .10 .88 .35]);hold(ax2,'on');grid(ax2,'on');plot(ax2,log.t,log.knownFreeFraction,'LineWidth',1.2);plot(ax2,log.t,log.unknownFraction,'LineWidth',1.2);stairs(ax2,log.t,double(log.perceptionFresh),'--','LineWidth',1.0);xlabel(ax2,'Time [s]');ylabel(ax2,'Fraction / flag');legend(ax2,{'Known free','Unknown','Perception fresh'});title(ax2,'Observed-space growth and perception freshness');

% State and replanning.
tab=uitab(G,'Title','Lifecycle & replanning');
ax1=axes(tab,'Position',[.07 .55 .88 .37]);hold(ax1,'on');grid(ax1,'on');stairs(ax1,log.t,log.stateId,'LineWidth',1.2);stairs(ax1,log.t,log.laneId+0.08,'LineWidth',1.0);ylabel(ax1,'State / lane');title(ax1,'Mission state and selected ESKF lane');
ax2=axes(tab,'Position',[.07 .10 .88 .34]);hold(ax2,'on');grid(ax2,'on');stairs(ax2,log.t,double(log.segmentIsFinal),'LineWidth',1.0);stairs(ax2,log.t,double(log.mapUpdateAccepted)+1.1,'LineWidth',.9);stairs(ax2,log.t,double(log.perceptionFresh)+2.2,'LineWidth',.9);xlabel(ax2,'Time [s]');ylabel(ax2,'Flags with offsets');legend(ax2,{'Final segment','Map update +1.1','Perception fresh +2.2'});title(ax2,sprintf('Extensions %d | scans %d | safety replans %d',summary.mapExtensionCount,summary.scanHoldCount,summary.mapSafetyReplanCount));

% Estimator.
tab=uitab(G,'Title','Estimator');
ax1=axes(tab,'Position',[.07 .55 .88 .37]);hold(ax1,'on');grid(ax1,'on');plot(ax1,log.t,log.estimatorPositionError,'LineWidth',1.1);plot(ax1,log.t,log.estimatorAttitudeError_deg/10,'LineWidth',1.0);ylabel(ax1,'m / deg÷10');legend(ax1,{'Position error','Attitude error/10'});title(ax1,'Estimator accuracy');
ax2=axes(tab,'Position',[.07 .10 .88 .34]);hold(ax2,'on');grid(ax2,'on');plot(ax2,log.t,log.laneScores,'LineWidth',.8);stairs(ax2,log.t,log.laneId,'k-','LineWidth',1.2);xlabel(ax2,'Time [s]');ylabel(ax2,'Score / lane');title(ax2,sprintf('Final lane %d | switches %d',summary.activeLaneFinal,summary.laneSwitches));

% Tracking and kinematics.
tab=uitab(G,'Title','Control & limits');
ax1=axes(tab,'Position',[.07 .55 .88 .37]);hold(ax1,'on');grid(ax1,'on');plot(ax1,log.t,log.trackingError,'LineWidth',1.1);plot(ax1,log.t,abs(log.truthP(:,3)-log.pRef(:,3)),'LineWidth',1.0);yline(ax1,cfg.maxPositionTrackingError_m,'--');ylabel(ax1,'Error [m]');legend(ax1,{'Reference deviation','Altitude error','Tracking limit'});title(ax1,'Tracking performance');
ax2=axes(tab,'Position',[.07 .10 .88 .34]);hold(ax2,'on');grid(ax2,'on');plot(ax2,log.t,vecnorm(log.truthV(:,1:2),2,2));plot(ax2,log.t,vecnorm(log.truthA(:,1:2),2,2));plot(ax2,log.t,vecnorm(log.truthJ(:,1:2),2,2));xlabel(ax2,'Time [s]');ylabel(ax2,'XY v/a/j');legend(ax2,{'Speed','Acceleration','Jerk'});title(ax2,'Executed horizontal kinematics');

% Safety.
tab=uitab(G,'Title','Safety');
ax1=axes(tab,'Position',[.07 .55 .88 .37]);hold(ax1,'on');grid(ax1,'on');plot(ax1,log.t,log.inflationRadius,'LineWidth',1.2);plot(ax1,log.t,cfg.baseInflationRadius+cfg.uncertaintySigmaGain*log.xySigma,'--');ylabel(ax1,'Radius [m]');legend(ax1,{'Applied','Base + covariance'});title(ax1,'Estimator-aware inflation');
ax2=axes(tab,'Position',[.07 .10 .88 .34]);axis(ax2,'off');text(ax2,.02,.90,sprintf(['Collision: %d\nGeofence: %d\nUnknown commitment: %d\n' ...
    'False-free rate: %.6f\nOccupied recall: %.3f\nTruth isolation: %d\nFinal PASS: %d'], ...
    summary.collisionCount,summary.geofenceViolationCount,summary.unknownCommitmentCount, ...
    summary.mapFalseFreeRate,summary.mapOccupiedRecall,summary.truthIsolationPass,summary.pass), ...
    'FontName','FixedWidth','FontSize',12,'VerticalAlignment','top');title(ax2,'Acceptance summary');

base=sprintf('S2_3_%s_%s_dashboard',versionLabel,label);figFile=fullfile(resultsDir,[base '.fig']);
drawnow;plotFiles={};try,savefig(fig,figFile);plotFiles{end+1}=figFile;catch,end
% Export each tab without creating additional figure windows.
for i=1:numel(G.Children)
    tabObj=G.Children(i);safe=lower(regexprep(tabObj.Title,'[^A-Za-z0-9]+','_'));
    png=fullfile(resultsDir,sprintf('%s_tab_%s.png',base,safe));
    try,exportgraphics(tabObj,png,'Resolution',160);plotFiles{end+1}=png;catch,end %#ok<AGROW>
end
end

function videoFile = animate_S2_flight(gt,est,vio,alt,lidar,localLidar,globalLidar, ...
    cfg,label,results_dir,varargin)
% ANIMATE_S2_FLIGHT  Animate the 6-DOF indoor-drone estimate and save MP4.
%
% This function is designed for run_S2_lidar_slam.m.  It opens one live
% animation window, reuses that same window for nominal/stress trials, and
% optionally saves an MPEG-4 video into the trial results folder.
%
% Required simulation data already exist in Stage S2:
%   gt.t, gt.p, gt.q       ground-truth time, position and quaternion
%   est.p, est.q           ESKF position and quaternion
%   est.posErr, est.attErr navigation errors
%   est.health             estimator-health flag
%   lidar.scans, lidar.t   raw planar LiDAR scans and timestamps
%   localLidar.valid       accepted/rejected LiDAR updates
%   globalLidar            globally consistent map-frame trajectory
%   cfg.room/obstacles     room geometry
%
% Optional name-value arguments:
%   'PlaybackSpeed'  simulation seconds per wall-clock second (default 4)
%   'FrameRate'      displayed/recorded frames per second (default 30)
%   'RecordVideo'    save MP4 in results_dir (default true)
%   'ShowLidar'      animate current accepted LiDAR scan (default true)
%   'KeepWindow'     keep window open after playback (default true)
%
% Example
%   animate_S2_flight(gt,est,vio,alt,lidar,localLidar,globalLidar, ...
%       cfg,'NOMINAL',results_dir,'PlaybackSpeed',3,'RecordVideo',true)
%
% No UAV Toolbox is required. The preferred drone model is loaded from
% the selected F450 assembly STL parts under assets/F450. A lightweight
% geometric model is used automatically if the assets cannot be loaded.

p=inputParser;
p.addParameter('PlaybackSpeed',4,@(x)isnumeric(x)&&isscalar(x)&&x>0);
p.addParameter('FrameRate',30,@(x)isnumeric(x)&&isscalar(x)&&x>0);
p.addParameter('RecordVideo',true,@(x)islogical(x)||ismember(x,[0 1]));
p.addParameter('ShowLidar',true,@(x)islogical(x)||ismember(x,[0 1]));
p.addParameter('KeepWindow',true,@(x)islogical(x)||ismember(x,[0 1]));
p.addParameter('UseSTL',true,@(x)islogical(x)||ismember(x,[0 1]));
p.addParameter('ShowCameraFOV',true,@(x)islogical(x)||ismember(x,[0 1]));
p.addParameter('ShowCollisionEnvelope',true,@(x)islogical(x)||ismember(x,[0 1]));
p.addParameter('ShowGeofence',true,@(x)islogical(x)||ismember(x,[0 1]));
p.parse(varargin{:});
opt=p.Results;

if ~exist(results_dir,'dir'),mkdir(results_dir);end
labelLower=lower(label);
videoFile=fullfile(results_dir,sprintf('S2_1_%s_flight_animation.mp4',labelLower));

% Reuse one animation window rather than opening another window per trial.
fig=findall(groot,'Type','figure','Tag','S2FlightAnimationWindow');
if isempty(fig)||~isgraphics(fig)
    fig=figure('Name','Stage S2.1 — 6-DOF flight animation', ...
        'Tag','S2FlightAnimationWindow','Color','w', ...
        'Position',[60 50 1450 900],'NumberTitle','off');
else
    fig=fig(1);clf(fig);figure(fig);
end
set(fig,'CloseRequestFcn',@closeAnimation);
setappdata(fig,'S2Stop',false);
setappdata(fig,'S2Pause',false);

% Reserve a bottom strip for controls/status.
plotPanel=uipanel(fig,'Units','normalized','Position',[0 0.075 1 0.925], ...
    'BorderType','none','BackgroundColor','w');
tl=tiledlayout(plotPanel,2,3,'TileSpacing','compact','Padding','compact');
title(tl,sprintf('Stage S2.1 — 6-DOF drone motion (%s)',label), ...
    'FontWeight','normal');

%% Main 3-D scene
ax3=nexttile(tl,[2 2]);hold(ax3,'on');grid(ax3,'on');axis(ax3,'equal');
xlabel(ax3,'x [m]');ylabel(ax3,'y [m]');zlabel(ax3,'z [m]');
title(ax3,'Local ESKF flight pose with room, obstacles and live LiDAR');
view(ax3,-38,25);
xlim(ax3,[-0.25 cfg.room(1)+0.25]);
ylim(ax3,[-0.25 cfg.room(2)+0.25]);
zlim(ax3,[0 cfg.room(3)+0.25]);
drawRoom3D(ax3,cfg,opt.ShowGeofence);

% Full reference paths stay visible; dynamic trails show progress.
refPath=gt.p;if isfield(gt,'pRef'),refPath=gt.pRef;end
plot3(ax3,refPath(:,1),refPath(:,2),refPath(:,3),'--', ...
    'Color',[0.25 0.25 0.25],'LineWidth',1.0,'DisplayName','Desired/reference path');
plot3(ax3,globalLidar(:,1),globalLidar(:,2),0.025*ones(size(globalLidar,1),1), ...
    ':','Color',[0.00 0.55 0.35],'LineWidth',1.0,'DisplayName','Global map path');
hTruthTrail=animatedline(ax3,'Color',[0.20 0.20 0.20], ...
    'LineStyle','--','LineWidth',1.4,'DisplayName','Truth trail');
hEstTrail=animatedline(ax3,'Color',[0.00 0.45 0.74], ...
    'LineWidth',2.0,'DisplayName','ESKF trail');
hTruthNow=plot3(ax3,nan,nan,nan,'o','MarkerSize',6, ...
    'MarkerFaceColor',[0.15 0.15 0.15],'MarkerEdgeColor','none', ...
    'DisplayName','Current truth');
hLidar=plot3(ax3,nan,nan,nan,'.','Color',[0.49 0.18 0.56], ...
    'MarkerSize',5,'DisplayName','Current LiDAR scan');

% Drone geometry is attached to one transform.  Updating one 4x4 matrix
% moves every arm, body part, rotor and body-axis indicator consistently.
droneTf=hgtransform('Parent',ax3);
model=drawF450Model(droneTf,cfg,opt.UseSTL);
if opt.ShowCameraFOV,drawCameraFrustum(droneTf,cfg);end
if opt.ShowCollisionEnvelope
    model.envelopeHandle=drawCollisionEnvelope(droneTf,cfg);
else
    model.envelopeHandle=gobjects(0);
end
try,camlight(ax3,'headlight');lighting(ax3,'gouraud');catch,end
legend(ax3,'Location','northeastoutside');

%% Top view
axTop=nexttile(tl);hold(axTop,'on');grid(axTop,'on');axis(axTop,'equal');
xlabel(axTop,'x [m]');ylabel(axTop,'y [m]');title(axTop,'Top view');
xlim(axTop,[-0.25 cfg.room(1)+0.25]);ylim(axTop,[-0.25 cfg.room(2)+0.25]);
drawRoom2D(axTop,cfg,opt.ShowGeofence);
plot(axTop,refPath(:,1),refPath(:,2),'--','Color',[0.30 0.30 0.30]);
plot(axTop,est.p(:,1),est.p(:,2),'Color',[0.72 0.82 0.92]);
hTopTruth=plot(axTop,nan,nan,'o','MarkerFaceColor',[0.15 0.15 0.15], ...
    'MarkerEdgeColor','none','MarkerSize',6);
hTopEst=plot(axTop,nan,nan,'o','MarkerFaceColor',[0.00 0.45 0.74], ...
    'MarkerEdgeColor','none','MarkerSize',7);
hHeading=quiver(axTop,nan,nan,nan,nan,0,'LineWidth',1.8, ...
    'MaxHeadSize',1.8,'Color',[0.85 0.33 0.10]);

%% Error timeline
axErr=nexttile(tl);hold(axErr,'on');grid(axErr,'on');
title(axErr,'Navigation error and playback cursor');xlabel(axErr,'Time [s]');
xlim(axErr,[0 cfg.duration]);
yyaxis(axErr,'left');
plot(axErr,gt.t,100*est.posErr,'Color',[0.00 0.45 0.74], ...
    'DisplayName','3-D position error');
yline(axErr,100*cfg.requirement_m,'--','Color',[0.75 0.10 0.10], ...
    'Label','10 cm','HandleVisibility','off');
ylabel(axErr,'Position error [cm]');
yyaxis(axErr,'right');
plot(axErr,gt.t,rad2deg(est.attErr),'Color',[0.85 0.33 0.10], ...
    'DisplayName','Attitude error');
yline(axErr,cfg.attRequirement_deg,':','Color',[0.75 0.10 0.10], ...
    'Label','2 deg','HandleVisibility','off');
ylabel(axErr,'Attitude error [deg]');
hCursor=xline(axErr,0,'k-','LineWidth',1.3,'HandleVisibility','off');
legend(axErr,'Location','best');

% Controls and status use one dedicated strip below the axes.
pauseButton=uicontrol(fig,'Style','pushbutton','String','Pause', ...
    'Units','normalized','Position',[0.015 0.016 0.075 0.038], ...
    'FontSize',10,'Callback',@togglePause);
stopButton=uicontrol(fig,'Style','pushbutton','String','Stop', ...
    'Units','normalized','Position',[0.098 0.016 0.075 0.038], ...
    'FontSize',10,'Callback',@stopAnimation);
statusText=uicontrol(fig,'Style','text','String','Preparing animation...', ...
    'Units','normalized','Position',[0.19 0.009 0.79 0.052], ...
    'BackgroundColor','w','HorizontalAlignment','left','FontSize',10);

%% Video writer
writer=[];
recording=false;
if opt.RecordVideo
    try
        writer=VideoWriter(videoFile,'MPEG-4');
        writer.FrameRate=opt.FrameRate;
        writer.Quality=92;
        open(writer);
        recording=true;
    catch ME
        warning('Could not open MPEG-4 writer: %s. Animation will still display.',ME.message);
        writer=[];
    end
end
writerCleanup=onCleanup(@()closeWriterSafely(writer)); %#ok<NASGU>

% Display step: a 200 Hz estimator does not need 200 graphics updates/s.
imuRate=1/median(diff(gt.t));
indexStep=max(1,round(imuRate*opt.PlaybackSpeed/opt.FrameRate));
frames=1:indexStep:numel(gt.t);
if frames(end)~=numel(gt.t),frames(end+1)=numel(gt.t);end

lastLidarIdx=0;
rotorPhase=zeros(1,4);
lastAnimationTime=gt.t(frames(1));
for frameNo=1:numel(frames)
    if ~isgraphics(fig)||getappdata(fig,'S2Stop'),break;end
    while isgraphics(fig)&&getappdata(fig,'S2Pause')&&~getappdata(fig,'S2Stop')
        drawnow;pause(0.04);
    end
    if ~isgraphics(fig)||getappdata(fig,'S2Stop'),break;end

    ticFrame=tic;
    k=frames(frameNo);tNow=gt.t(k);
    pNow=est.p(k,:).';RNow=q2RAnimation(est.q(k,:));
    droneTf.Matrix=[RNow pNow;0 0 0 1];

    rpm=getMotorRPM(est,k,cfg);
    dtAnim=max(0,tNow-lastAnimationTime);
    rotorPhase=rotorPhase+model.rotorDirection.*(2*pi*rpm/60)*dtAnim;
    updatePropellerTransforms(model,rotorPhase);
    lastAnimationTime=tNow;

    addpoints(hTruthTrail,gt.p(k,1),gt.p(k,2),gt.p(k,3));
    addpoints(hEstTrail,est.p(k,1),est.p(k,2),est.p(k,3));
    set(hTruthNow,'XData',gt.p(k,1),'YData',gt.p(k,2),'ZData',gt.p(k,3));
    set(hTopTruth,'XData',gt.p(k,1),'YData',gt.p(k,2));
    set(hTopEst,'XData',est.p(k,1),'YData',est.p(k,2));
    heading=RNow(:,1);
    set(hHeading,'XData',est.p(k,1),'YData',est.p(k,2), ...
        'UData',0.35*heading(1),'VData',0.35*heading(2));
    hCursor.Value=tNow;

    % Show the most recently available LiDAR scan, transformed using the
    % current ESKF body pose and calibrated body-to-LiDAR extrinsics.
    li=min(numel(lidar.t),max(1,round(tNow*cfg.lidarRate)+1));
    if opt.ShowLidar && li~=lastLidarIdx && localLidar.valid(li)
        scan=lidar.scans{li}(1:3:end,:);
        RWL=RNow*cfg.R_BL;
        pL=pNow+RNow*cfg.r_BL;
        pointsW=RWL*[scan.';zeros(1,size(scan,1))]+pL;
        set(hLidar,'XData',pointsW(1,:),'YData',pointsW(2,:),'ZData',pointsW(3,:));
        lastLidarIdx=li;
    elseif ~opt.ShowLidar
        set(hLidar,'XData',nan,'YData',nan,'ZData',nan);
    end

    vi=min(numel(vio.t),max(1,round(tNow*cfg.vioRate)+1));
    ri=min(numel(alt.tr),max(1,round(tNow*cfg.rangeRate)+1));
    li=min(numel(lidar.t),max(1,round(tNow*cfg.lidarRate)+1));
    stateWord='HEALTHY';if ~est.health(k),stateWord='UNHEALTHY';end
    [insideFence,safetyWord]=insideSafetyVolume(pNow,cfg);
    if insideFence
        set(statusText,'BackgroundColor','w');
        if isgraphics(model.envelopeHandle),set(model.envelopeHandle,'Color',[0.90 0.50 0.05]);end
    else
        set(statusText,'BackgroundColor',[1.00 0.82 0.82]);
        if isgraphics(model.envelopeHandle),set(model.envelopeHandle,'Color',[0.85 0.05 0.05]);end
    end
    set(statusText,'String',sprintf([ ...
        't=%5.2f s | p=[%5.2f,%5.2f,%4.2f] m | error=%5.2f cm | ' ...
        'att=%4.2f deg | motors~%4.0f rpm | VIO %s  LiDAR %s  Range %s | ' ...
        'ESKF %s | GEOFENCE %s'], ...
        tNow,est.p(k,1),est.p(k,2),est.p(k,3),100*est.posErr(k), ...
        rad2deg(est.attErr(k)),mean(rpm),onOff(vio.valid(vi)), ...
        onOff(localLidar.valid(li)),onOff(alt.validR(ri)),stateWord,safetyWord));

    drawnow limitrate;
    if recording&&isgraphics(fig)
        try
            writeVideo(writer,getframe(fig));
        catch ME
            warning('Video frame write stopped: %s',ME.message);
            recording=false;
        end
    end

    remaining=1/opt.FrameRate-toc(ticFrame);
    if remaining>0,pause(remaining);end
end

if ~isempty(writer)
    closeWriterSafely(writer);
    writer=[];
end
if isgraphics(fig)
    set(statusText,'String',sprintf('%s animation complete. Video: %s',label,videoFile));
    set(pauseButton,'Enable','off');set(stopButton,'Enable','off');
    try
        exportgraphics(fig,fullfile(results_dir, ...
            sprintf('S2_visual_slam_%s_animation_final.png',labelLower)), ...
            'Resolution',180);
        savefig(fig,fullfile(results_dir, ...
            sprintf('S2_visual_slam_%s_animation_window.fig',labelLower)));
    catch ME
        warning('Could not save final animation window: %s',ME.message);
    end
    if ~opt.KeepWindow,delete(fig);end
end

    function togglePause(src,~)
        if ~isgraphics(fig),return;end
        paused=~getappdata(fig,'S2Pause');
        setappdata(fig,'S2Pause',paused);
        if paused,set(src,'String','Resume');else,set(src,'String','Pause');end
    end

    function stopAnimation(~,~)
        if isgraphics(fig),setappdata(fig,'S2Stop',true);end
    end

    function closeAnimation(~,~)
        if isgraphics(fig)
            setappdata(fig,'S2Stop',true);
            delete(fig);
        end
    end
end

% ========================================================================
function model=drawF450Model(parent,cfg,useSTL)
% Draw the selected F450 CAD assembly. Source STL coordinates use X-Z as
% the horizontal plane and Y as vertical; loadF450Mesh converts them to the
% body convention x-forward, y-left, z-up.
model=struct('propTf',gobjects(1,4),'propPivot',zeros(4,3), ...
    'rotorDirection',[1 -1 1 -1],'usedSTL',false,'envelopeHandle',gobjects(0));
if useSTL && isfield(cfg,'modelAssetDir') && exist(cfg.modelAssetDir,'dir')==7
    try
        mesh=loadF450Mesh(cfg.modelAssetDir);
        for i=1:numel(mesh.frame)
            patch('Parent',parent,'Faces',mesh.frame{i}.F,'Vertices',mesh.frame{i}.V, ...
                'FaceColor',mesh.frame{i}.color,'EdgeColor','none', ...
                'FaceLighting','gouraud','AmbientStrength',0.45,'DiffuseStrength',0.65);
        end
        for i=1:4
            model.propTf(i)=hgtransform('Parent',parent);
            model.propPivot(i,:)=mesh.prop{i}.pivot;
            patch('Parent',model.propTf(i),'Faces',mesh.prop{i}.F, ...
                'Vertices',mesh.prop{i}.V,'FaceColor',[0.86 0.88 0.92], ...
                'FaceAlpha',0.48,'EdgeColor','none','FaceLighting','gouraud');
        end
        updatePropellerTransforms(model,zeros(1,4));
        model.usedSTL=true;
    catch ME
        warning('F450 STL model could not be loaded (%s). Using fallback.',ME.message);
        model=drawFallbackF450(parent,cfg);
    end
else
    model=drawFallbackF450(parent,cfg);
end

% Body-axis indicators: red x/front, green y/left, blue z/up.
line('Parent',parent,'XData',[0 0.18],'YData',[0 0], ...
    'ZData',[0.10 0.10],'LineWidth',3,'Color',[0.85 0.10 0.10]);
line('Parent',parent,'XData',[0 0],'YData',[0 0.18], ...
    'ZData',[0.10 0.10],'LineWidth',3,'Color',[0.10 0.65 0.20]);
line('Parent',parent,'XData',[0 0],'YData',[0 0], ...
    'ZData',[0.10 0.25],'LineWidth',3,'Color',[0.10 0.30 0.90]);
end

function mesh=loadF450Mesh(assetDir)
cacheFile=fullfile(assetDir,'f450_animation_mesh_cache.mat');
if exist(cacheFile,'file')==2
    S=load(cacheFile,'mesh');mesh=S.mesh;return;
end
frameFiles={'base_top.stl','base_bottom.stl','arm_1.stl','arm_2.stl', ...
    'arm_3.stl','arm_4.stl','motor_1.stl','motor_2.stl','motor_3.stl','motor_4.stl'};
targetFaces=[3500 3000 4500 4500 4500 4500 1800 1800 1800 1800];
colors={[0.12 0.12 0.14],[0.12 0.12 0.14], ...
    [0.72 0.08 0.06],[0.88 0.88 0.90],[0.72 0.08 0.06],[0.88 0.88 0.90], ...
    [0.10 0.10 0.11],[0.10 0.10 0.11],[0.10 0.10 0.11],[0.10 0.10 0.11]};
mesh.frame=cell(1,numel(frameFiles));
for i=1:numel(frameFiles)
    [F,V]=readReducedSTL(fullfile(assetDir,frameFiles{i}),targetFaces(i));
    mesh.frame{i}=struct('F',F,'V',sourceToBody(V),'color',colors{i});
end
mesh.prop=cell(1,4);
for i=1:4
    [F,V]=readReducedSTL(fullfile(assetDir,sprintf('prop_%d.stl',i)),1800);
    V=sourceToBody(V);
    pivot=(min(V,[],1)+max(V,[],1))/2;
    mesh.prop{i}=struct('F',F,'V',V-pivot,'pivot',pivot);
end
try,save(cacheFile,'mesh','-v7.3');catch,end
end

function [F,V]=readReducedSTL(fileName,targetFaces)
if exist(fileName,'file')~=2,error('Missing model asset: %s',fileName);end
TR=stlread(fileName);
if isa(TR,'triangulation')
    F=TR.ConnectivityList;V=TR.Points;
elseif isstruct(TR)
    if isfield(TR,'ConnectivityList'),F=TR.ConnectivityList;V=TR.Points;
    else,F=TR.faces;V=TR.vertices;end
else
    error('Unsupported stlread output for %s',fileName);
end
if size(F,1)>targetFaces
    [F,V]=reducepatch(F,V,targetFaces);
end
end

function Vb=sourceToBody(V)
% CAD: source X and Z are horizontal; source Y is vertical.
Vb=[V(:,1),V(:,3),V(:,2)];
end

function model=drawFallbackF450(parent,cfg)
rMotor=cfg.motorArmRadius;a=rMotor/sqrt(2);
motors=[a a 0;-a a 0;-a -a 0;a -a 0];
line('Parent',parent,'XData',[-a a],'YData',[-a a],'ZData',[0 0], ...
    'LineWidth',5,'Color',[0.20 0.20 0.22]);
line('Parent',parent,'XData',[-a a],'YData',[a -a],'ZData',[0 0], ...
    'LineWidth',5,'Color',[0.20 0.20 0.22]);
makeCuboid(parent,[-0.07 -0.055 -0.025],[0.14 0.11 0.05],[0.10 0.35 0.65]);
model=struct('propTf',gobjects(1,4),'propPivot',motors, ...
    'rotorDirection',[1 -1 1 -1],'usedSTL',false,'envelopeHandle',gobjects(0));
th=linspace(0,2*pi,50);
for i=1:4
    model.propTf(i)=hgtransform('Parent',parent);
    line('Parent',model.propTf(i),'XData',cfg.propRadius*cos(th), ...
        'YData',cfg.propRadius*sin(th),'ZData',zeros(size(th)), ...
        'LineWidth',2,'Color',[0.15 0.15 0.15]);
end
updatePropellerTransforms(model,zeros(1,4));
end

function updatePropellerTransforms(model,phase)
for i=1:numel(model.propTf)
    if ~isgraphics(model.propTf(i)),continue;end
    c=cos(phase(i));s=sin(phase(i));p=model.propPivot(i,:).';
    model.propTf(i).Matrix=[c -s 0 p(1);s c 0 p(2);0 0 1 p(3);0 0 0 1];
end
end

function rpm=getMotorRPM(est,k,cfg)
if isfield(est,'motorRPM') && size(est.motorRPM,1)>=k && size(est.motorRPM,2)==4
    rpm=max(0,est.motorRPM(k,:));
else
    % Stage S2 is an estimator simulation and has no ESC/motor telemetry.
    % This physically scaled hover value is for visualization only.
    rpm=cfg.motorHoverRPM*ones(1,4);
end
end

function drawCameraFrustum(parent,cfg)
p0=cfg.r_BC(:).';L=cfg.cameraFrustumRange;
h=L*tand(cfg.cameraHFOV_deg/2);v=L*tand(cfg.cameraVFOV_deg/2);
% Visualization optical axis is body +x. Extrinsic orientation can be
% replaced with the measured D435i mount transform in hardware.
C=[p0; p0+[L -h -v]; p0+[L h -v]; p0+[L h v]; p0+[L -h v]];
F=[1 2 3;1 3 4;1 4 5;1 5 2;2 3 4;2 4 5];
patch('Parent',parent,'Vertices',C,'Faces',F,'FaceColor',[0.20 0.65 0.95], ...
    'FaceAlpha',0.045,'EdgeColor',[0.20 0.55 0.90],'LineWidth',0.8);
makeCuboid(parent,p0+[-0.015 -0.045 -0.012],[0.030 0.090 0.024],[0.12 0.18 0.22]);
end

function h=drawCollisionEnvelope(parent,cfg)
th=linspace(0,2*pi,100);
h=line('Parent',parent,'XData',cfg.collisionRadius*cos(th), ...
    'YData',cfg.collisionRadius*sin(th),'ZData',0.10*ones(size(th)), ...
    'LineStyle','--','LineWidth',1.4,'Color',[0.90 0.50 0.05]);
end

function makeCuboid(parent,origin,sizeXYZ,faceColor)
x=origin(1)+[0 sizeXYZ(1)];y=origin(2)+[0 sizeXYZ(2)];z=origin(3)+[0 sizeXYZ(3)];
V=[x(1) y(1) z(1);x(2) y(1) z(1);x(2) y(2) z(1);x(1) y(2) z(1); ...
   x(1) y(1) z(2);x(2) y(1) z(2);x(2) y(2) z(2);x(1) y(2) z(2)];
F=[1 2 3 4;5 8 7 6;1 5 6 2;2 6 7 3;3 7 8 4;4 8 5 1];
patch('Parent',parent,'Vertices',V,'Faces',F,'FaceColor',faceColor, ...
    'FaceAlpha',0.92,'EdgeColor',[0.15 0.15 0.15]);
end

function drawRoom3D(ax,cfg,showGeofence)
patch(ax,[0 cfg.room(1) cfg.room(1) 0],[0 0 cfg.room(2) cfg.room(2)], ...
    [0 0 0 0],[0.92 0.92 0.92],'FaceAlpha',0.35,'EdgeColor','none');
plotBoxEdges(ax,[0 0 0],cfg.room,[0.35 0.35 0.35],1.0);
for i=1:size(cfg.obstacles,1)
    o=cfg.obstacles(i,:);
    plotObstacleBox(ax,[o(1) o(2) 0],[o(3) o(4) cfg.obstacleHeight], ...
        [0.55 0.55 0.55],0.28);
end
if showGeofence && isfield(cfg,'geofence')
    g=cfg.geofence;
    plotBoxEdges(ax,[g(1) g(3) g(5)],[g(2)-g(1) g(4)-g(3) g(6)-g(5)], ...
        [0.05 0.60 0.20],1.5);
    for i=1:size(cfg.obstacles,1)
        r=inflateObstacle(cfg.obstacles(i,:),cfg.geofenceMarginXY,cfg.room);
        plotObstacleBox(ax,[r(1) r(2) 0],[r(3) r(4) cfg.obstacleHeight+cfg.controlMargin], ...
            [0.90 0.15 0.10],0.08);
    end
end
end

function drawRoom2D(ax,cfg,showGeofence)
rectangle(ax,'Position',[0 0 cfg.room(1) cfg.room(2)], ...
    'EdgeColor',[0.25 0.25 0.25],'LineWidth',1.2);
for i=1:size(cfg.obstacles,1)
    o=cfg.obstacles(i,:);
    rectangle(ax,'Position',o,'FaceColor',[0.75 0.75 0.75], ...
        'EdgeColor',[0.25 0.25 0.25]);
end
if showGeofence && isfield(cfg,'geofence')
    g=cfg.geofence;
    rectangle(ax,'Position',[g(1) g(3) g(2)-g(1) g(4)-g(3)], ...
        'EdgeColor',[0.05 0.60 0.20],'LineStyle','--','LineWidth',1.5);
    for i=1:size(cfg.obstacles,1)
        r=inflateObstacle(cfg.obstacles(i,:),cfg.geofenceMarginXY,cfg.room);
        rectangle(ax,'Position',r,'EdgeColor',[0.85 0.10 0.05], ...
            'LineStyle',':','LineWidth',1.2);
    end
end
end

function r=inflateObstacle(o,m,room)
x0=max(0,o(1)-m);y0=max(0,o(2)-m);
x1=min(room(1),o(1)+o(3)+m);y1=min(room(2),o(2)+o(4)+m);
r=[x0 y0 x1-x0 y1-y0];
end

function [inside,word]=insideSafetyVolume(p,cfg)
g=cfg.geofence;
inside=p(1)>=g(1)&&p(1)<=g(2)&&p(2)>=g(3)&&p(2)<=g(4)&& ...
       p(3)>=g(5)&&p(3)<=g(6);
if inside
    for i=1:size(cfg.obstacles,1)
        r=inflateObstacle(cfg.obstacles(i,:),cfg.geofenceMarginXY,cfg.room);
        if p(1)>=r(1)&&p(1)<=r(1)+r(3)&&p(2)>=r(2)&&p(2)<=r(2)+r(4)&& ...
                p(3)<=cfg.obstacleHeight+cfg.controlMargin
            inside=false;break;
        end
    end
end
if inside,word='SAFE';else,word='VIOLATION';end
end

function plotObstacleBox(ax,origin,sizeXYZ,faceColor,faceAlpha)
x=origin(1)+[0 sizeXYZ(1)];y=origin(2)+[0 sizeXYZ(2)];z=origin(3)+[0 sizeXYZ(3)];
V=[x(1) y(1) z(1);x(2) y(1) z(1);x(2) y(2) z(1);x(1) y(2) z(1); ...
   x(1) y(1) z(2);x(2) y(1) z(2);x(2) y(2) z(2);x(1) y(2) z(2)];
F=[1 2 3 4;5 8 7 6;1 5 6 2;2 6 7 3;3 7 8 4;4 8 5 1];
patch(ax,'Vertices',V,'Faces',F,'FaceColor',faceColor, ...
    'FaceAlpha',faceAlpha,'EdgeColor',[0.35 0.35 0.35]);
end

function plotBoxEdges(ax,origin,sizeXYZ,color,width)
x=origin(1)+[0 sizeXYZ(1)];y=origin(2)+[0 sizeXYZ(2)];z=origin(3)+[0 sizeXYZ(3)];
V=[x(1) y(1) z(1);x(2) y(1) z(1);x(2) y(2) z(1);x(1) y(2) z(1); ...
   x(1) y(1) z(2);x(2) y(1) z(2);x(2) y(2) z(2);x(1) y(2) z(2)];
E=[1 2;2 3;3 4;4 1;5 6;6 7;7 8;8 5;1 5;2 6;3 7;4 8];
for i=1:size(E,1)
    line(ax,V(E(i,:),1),V(E(i,:),2),V(E(i,:),3), ...
        'Color',color,'LineWidth',width);
end
end

function s=onOff(flag)
if flag,s='ON';else,s='OFF';end
end

function closeWriterSafely(writer)
if isempty(writer),return;end
try,close(writer);catch,end
end

function R=q2RAnimation(q)
q=q(:).';q=q/max(norm(q),eps);if q(1)<0,q=-q;end
w=q(1);x=q(2);y=q(3);z=q(4);
R=[1-2*(y^2+z^2),2*(x*y-z*w),2*(x*z+y*w); ...
   2*(x*y+z*w),1-2*(x^2+z^2),2*(y*z-x*w); ...
   2*(x*z-y*w),2*(y*z+x*w),1-2*(x^2+y^2)];
end

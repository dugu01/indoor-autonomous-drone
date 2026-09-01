function traj = generate_min_snap_trajectory_S2_2(cfg,grid,path,startVelocity,startAcceleration,initialTimeScale)
% GENERATE_MIN_SNAP_TRAJECTORY_S2_2 Piecewise C3 seventh-order trajectory.
%
% Each segment is the minimum-snap polynomial for fixed endpoint position,
% velocity, acceleration and jerk constraints. Shared waypoint derivatives
% provide C3 continuity. Segment times are lengthened until sampled speed,
% acceleration and jerk limits are satisfied. If smooth internal waypoint
% velocities cause shape overshoot into the inflated costmap, the generator
% retries with zero internal derivatives, which follows the collision-free
% line segments monotonically.

if nargin<4||isempty(startVelocity),startVelocity=zeros(2,1);end
if nargin<5||isempty(startAcceleration),startAcceleration=zeros(2,1);end
if nargin<6||isempty(initialTimeScale),initialTimeScale=1.0;end
traj=invalid_traj();
if isempty(path)||size(path,1)<2||any(~isfinite(path(:))),return;end
path=remove_duplicates(path,grid.resolution/4);
if size(path,1)<2,return;end
segmentLength=vecnorm(diff(path,1,1),2,2);
base=max([segmentLength/(cfg.trajectoryNominalSpeedFraction*cfg.maxSpeedXY_mps), ...
    sqrt(max(segmentLength,1e-9)/(0.18*cfg.maxAccelXY_mps2)), ...
    cfg.trajectoryMinSegmentTime_s*ones(size(segmentLength))],[],2);
requestedDurations=max(base*initialTimeScale,eps);

for fallbackMode=0:1
    if fallbackMode==0
        durations=base*initialTimeScale;
    else
        durations=max([segmentLength/(0.55*cfg.maxSpeedXY_mps), ...
            sqrt(max(segmentLength,1e-9)/(0.12*cfg.maxAccelXY_mps2)), ...
            0.90*ones(size(segmentLength))],[],2)*initialTimeScale;
    end
    cumulativeScale=1.0;
    for iteration=1:cfg.trajectoryMaxScaleIterations
        [V,A,J]=waypoint_derivatives(path,durations,startVelocity,startAcceleration, ...
            fallbackMode==1,cfg);
        coefficients=zeros(numel(durations),8,2);
        for i=1:numel(durations)
            C=solve_segment(path(i,:),path(i+1,:),V(i,:),V(i+1,:), ...
                A(i,:),A(i+1,:),J(i,:),J(i+1,:),durations(i));
            coefficients(i,:,:)=reshape(C,1,8,2);
        end
        candidate=struct('valid',true,'path',path,'durations',durations(:).', ...
            'coefficients',coefficients,'duration_s',sum(durations));
        sample=sample_min_snap_trajectory_S2_2(candidate,cfg.trajectorySampleDt_s);
        maxSpeed=max(vecnorm(sample.v,2,2));
        maxAccel=max(vecnorm(sample.a,2,2));
        maxJerk=max(vecnorm(sample.j,2,2));
        collision=trajectory_occupied(sample.p,grid);
        ratio=max([maxSpeed/cfg.maxSpeedXY_mps, ...
            sqrt(maxAccel/cfg.maxAccelXY_mps2), ...
            nthroot(maxJerk/cfg.maxJerkXY_mps3,3),1.0]);
        if ratio<=1.0005 && ~collision
            continuity=continuity_metrics(coefficients,durations);
            candidate.sample=sample;candidate.maxSpeed_mps=maxSpeed;
            candidate.maxAccel_mps2=maxAccel;candidate.maxJerk_mps3=maxJerk;
            % Report the actual duration increase relative to the user's
            % requested nominal timing. This remains meaningful when the
            % conservative fallback uses a different base-duration rule.
            candidate.timeScale=max(durations./requestedDurations);
            candidate.fallbackUsed=logical(fallbackMode);
            candidate.continuity=continuity;candidate.initialTimeScale=initialTimeScale;
            traj=candidate;return;
        end
        if collision,break;end
        factor=cfg.trajectoryScaleSafety*ratio;
        durations=durations*factor;cumulativeScale=cumulativeScale*factor;
    end
end
end

function [V,A,J]=waypoint_derivatives(path,durations,startVelocity,startAcceleration,zeroInternal,cfg)
n=size(path,1);V=zeros(n,2);A=zeros(n,2);J=zeros(n,2);
V(1,:)=startVelocity(:).';A(1,:)=startAcceleration(:).';
for i=2:n-1
    if zeroInternal,continue;end
    chord=path(i+1,:)-path(i-1,:);nChord=norm(chord);if nChord<1e-12,continue;end
    speed=min(cfg.trajectoryInternalSpeedFraction*cfg.maxSpeedXY_mps, ...
        0.45*(norm(path(i,:)-path(i-1,:))/durations(i-1)+ ...
        norm(path(i+1,:)-path(i,:))/durations(i)));
    V(i,:)=speed*chord/nChord;
end
end

function C=solve_segment(p0,p1,v0,v1,a0,a1,j0,j1,T)
M=zeros(8,8);
for d=0:3
    M(d+1,d+1)=factorial(d);
    for power=d:7
        M(5+d,power+1)=factorial(power)/factorial(power-d)*T^(power-d);
    end
end
B=[p0;v0;a0;j0;p1;v1;a1;j1];C=M\B;
end

function tf = trajectory_occupied(points,grid)
% Use the same conservative segment checker for the path and polynomial.
tf = true;
if isempty(points) || size(points,2) ~= 2 || any(~isfinite(points(:)))
    return;
end
if size(points,1) == 1
    tf = segment_occupied_grid_S2_2(grid,points(1,:),points(1,:));
    return;
end
for i = 1:size(points,1)-1
    if segment_occupied_grid_S2_2(grid,points(i,:),points(i+1,:))
        return;
    end
end
tf = false;
end

function c=continuity_metrics(coefficients,durations)
c=zeros(1,4);
for i=1:numel(durations)-1
    C1=squeeze(coefficients(i,:,:));C2=squeeze(coefficients(i+1,:,:));
    for d=0:3
        c(d+1)=max(c(d+1),norm(eval_min_snap_segment_S2_2(C1,durations(i),d)- ...
            eval_min_snap_segment_S2_2(C2,0,d)));
    end
end
end

function p=remove_duplicates(p,tol)
keep=true(size(p,1),1);last=1;
for i=2:size(p,1)
    if norm(p(i,:)-p(last,:))<tol,keep(i)=false;else,last=i;end
end
p=p(keep,:);
end

function t=invalid_traj()
t=struct('valid',false,'path',zeros(0,2),'durations',zeros(1,0), ...
    'coefficients',zeros(0,8,2),'duration_s',0,'sample',struct(), ...
    'maxSpeed_mps',inf,'maxAccel_mps2',inf,'maxJerk_mps3',inf, ...
    'timeScale',inf,'fallbackUsed',false,'continuity',inf(1,4),'initialTimeScale',1);
end

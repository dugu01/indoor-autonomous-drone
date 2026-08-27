function sample = sample_min_snap_trajectory_S2_2(traj,sampleDt)
% SAMPLE_MIN_SNAP_TRAJECTORY_S2_2 Densely sample p,v,a,j for validation.
if nargin<2||isempty(sampleDt),sampleDt=0.02;end
if ~isfield(traj,'valid')||~traj.valid
    sample=struct('t',zeros(0,1),'p',zeros(0,2),'v',zeros(0,2), ...
        'a',zeros(0,2),'j',zeros(0,2),'segment',zeros(0,1));return;
end
T=[];P=[];V=[];A=[];J=[];S=[];offset=0;
for i=1:numel(traj.durations)
    duration=traj.durations(i);local=(0:sampleDt:duration).';
    if isempty(local)||local(end)<duration-1e-12,local(end+1,1)=duration;end
    if i>1&&~isempty(local),local(1)=[];end
    C=squeeze(traj.coefficients(i,:,:));
    T=[T;offset+local]; %#ok<AGROW>
    P=[P;eval_min_snap_segment_S2_2(C,local,0)]; %#ok<AGROW>
    V=[V;eval_min_snap_segment_S2_2(C,local,1)]; %#ok<AGROW>
    A=[A;eval_min_snap_segment_S2_2(C,local,2)]; %#ok<AGROW>
    J=[J;eval_min_snap_segment_S2_2(C,local,3)]; %#ok<AGROW>
    S=[S;i*ones(numel(local),1)]; %#ok<AGROW>
    offset=offset+duration;
end
sample=struct('t',T,'p',P,'v',V,'a',A,'j',J,'segment',S);
end

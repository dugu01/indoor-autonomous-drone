function ref = sample_min_snap_state_S2_2(traj,time_s)
% SAMPLE_MIN_SNAP_STATE_S2_2 Evaluate the reference state at one time.
if ~traj.valid,error('S2_2:InvalidTrajectory','Cannot sample an invalid trajectory.');end
time_s=max(0,min(time_s,traj.duration_s));cum=[0 cumsum(traj.durations)];
segment=find(time_s<=cum(2:end)+1e-12,1,'first');
if isempty(segment),segment=numel(traj.durations);end
localTime=max(0,min(time_s-cum(segment),traj.durations(segment)));
C=squeeze(traj.coefficients(segment,:,:));
ref=struct('p',eval_min_snap_segment_S2_2(C,localTime,0).', ...
    'v',eval_min_snap_segment_S2_2(C,localTime,1).', ...
    'a',eval_min_snap_segment_S2_2(C,localTime,2).', ...
    'j',eval_min_snap_segment_S2_2(C,localTime,3).', ...
    'segment',segment,'time_s',time_s);
end

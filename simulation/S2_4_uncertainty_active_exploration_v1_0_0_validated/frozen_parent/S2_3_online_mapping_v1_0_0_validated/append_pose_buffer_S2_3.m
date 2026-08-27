function buffer = append_pose_buffer_S2_3(buffer,est,t,maxAge)
% APPEND_POSE_BUFFER_S2_3 Add state and retain a bounded time history.
buffer.t(end+1,1)=t;buffer.p(end+1,:)=est.p(:).';buffer.q(end+1,:)=est.q(:).';buffer.xySigma(end+1,1)=est.xySigma_m;
keep=buffer.t>=t-maxAge;
buffer.t=buffer.t(keep);buffer.p=buffer.p(keep,:);buffer.q=buffer.q(keep,:);buffer.xySigma=buffer.xySigma(keep);
end

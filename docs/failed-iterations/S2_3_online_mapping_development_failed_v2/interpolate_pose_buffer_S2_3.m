function pose = interpolate_pose_buffer_S2_3(buffer,tQuery)
% INTERPOLATE_POSE_BUFFER_S2_3 Linear position and shortest-arc quaternion interpolation.
if isempty(buffer.t),error('S2_3:EmptyPoseBuffer','Pose buffer is empty.');end
if tQuery<=buffer.t(1),i0=1;i1=1;alpha=0;
elseif tQuery>=buffer.t(end),i0=numel(buffer.t);i1=i0;alpha=0;
else
    i1=find(buffer.t>=tQuery,1,'first');i0=i1-1;
    alpha=(tQuery-buffer.t(i0))/max(buffer.t(i1)-buffer.t(i0),eps);
end
p=(1-alpha)*buffer.p(i0,:)+alpha*buffer.p(i1,:);
q0=buffer.q(i0,:);q1=buffer.q(i1,:);if dot(q0,q1)<0,q1=-q1;end
q=qnormalize_S2_2((1-alpha)*q0+alpha*q1);
pose=struct('p',p(:),'q',q,'xySigma_m',(1-alpha)*buffer.xySigma(i0)+alpha*buffer.xySigma(i1));
end

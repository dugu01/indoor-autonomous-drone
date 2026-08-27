function buffer = init_pose_buffer_S2_3(est,t)
% INIT_POSE_BUFFER_S2_3 Short selected-estimator history for sensor alignment.
buffer=struct('t',t,'p',est.p(:).','q',est.q(:).','xySigma',est.xySigma_m);
end

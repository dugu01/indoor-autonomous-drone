function prediction = predict_dynamic_risk_S2_4(c,tracks,t0)
% PREDICT_DYNAMIC_RISK_S2_4 Class-agnostic constant-velocity prediction.
arguments
    c (1,1) struct
    tracks struct
    t0 (1,1) double
end
times=(0:c.dynamicStep_s:c.dynamicHorizon_s).';prediction=repmat(struct('trackId',uint64(0),'times',times,'means',zeros(numel(times),3),'covariances',zeros(3,3,numel(times)),'validUntil',t0+c.dynamicHorizon_s),numel(tracks),1);
for i=1:numel(tracks)
    tr=tracks(i);prediction(i).trackId=tr.trackId;dt0=max(0,t0-tr.timestamp);
    for k=1:numel(times)
        dt=dt0+times(k);prediction(i).means(k,:)=tr.position(:).'+dt*tr.velocity(:).';F=[eye(3) dt*eye(3);zeros(3) eye(3)];Q=(0.35^2)*[dt^4/4*eye(3) dt^3/2*eye(3);dt^3/2*eye(3) dt^2*eye(3)];P=F*tr.covariance*F.'+Q;prediction(i).covariances(:,:,k)=P(1:3,1:3);end
end
end

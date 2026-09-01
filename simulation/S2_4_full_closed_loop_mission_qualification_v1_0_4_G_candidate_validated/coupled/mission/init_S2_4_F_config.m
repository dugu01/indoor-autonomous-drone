function cfg = init_S2_4_F_config()
% INIT_S2_4_F_CONFIG Execution-time exploration-authority safety candidate.
cfg=init_S2_4_E_config();
cfg.stage='S2.4-F';
cfg.version='v0.4.2-execution-safety-revalidation-candidate';
% Preserve the validated E request TTL exactly. F turns the accepted request
% into a rolling execution lease: an unexpired request is renewed only after
% a CURRENT successful execution-time revalidation. An already expired
% request (F8) is never renewed.
cfg.executionSafety=struct( ...
    'enabled',true, ...
    'renewLeaseOnValidRevalidation',true, ...
    'maxAuthorityInvalidations',3, ...
    'dynamicPredictionLiveSupported',false, ...
    'validationFault',defaultFault());
end
function f=defaultFault()
f=struct('name','NONE','triggerDelay_s',0.10,'triggerProgress',0.20, ...
    'versionOffset',1000000,'repeatPerAuthority',false);
end

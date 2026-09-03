function check = preflight_check_S2_3(cfg,scenario,grid,packet,est,map,perceptionPacket,tNow) %#ok<INUSD>
% PREFLIGHT_CHECK_S2_3 Estimator, perception and launch-volume gates.
%
% Perception is qualified by the freshness of the most recently accepted
% mapping observation, not by whether a new asynchronous scan happens to
% arrive on this exact control step. This preserves the S2.2 aid-freshness
% lesson and prevents deterministic phase-dependent preflight rejection.
if nargin<8||isempty(tNow)||~isfinite(tNow)
    tNow=max([get_field(packet,'timestamp',0),get_field(map,'lastUpdateTime',0),0]);
end
estimateFinite=all(isfinite(est.p))&&all(isfinite(est.v))&&all(isfinite(est.q))&&all(isfinite(est.P(:)));
hAge=get_field(est,'horizontalAidAge_s',inf);vAge=get_field(est,'verticalAidAge_s',inf);aAge=get_field(est,'attitudeAidAge_s',inf);
activeLaneEligible=logical(get_field(est,'activeLaneEligible',false));xyCov=get_field(est,'xyCovariance',inf);zVar=get_field(est,'zVariance',inf);updates=get_field(est,'acceptedMeasurementCount',0);
hOK=hAge<=cfg.horizontalAidTimeout;vOK=~cfg.preflightRequireVerticalAid||vAge<=cfg.verticalAidTimeout;aOK=aAge<=cfg.attitudeAidTimeout;
covOK=xyCov<=cfg.preflightMaxXYCovariance&&zVar<=cfg.preflightMaxZVariance;updateOK=updates>=cfg.preflightMinAcceptedUpdates;
laneOK=activeLaneEligible&&~get_field(est,'degraded',true)&&~get_field(est,'rtlRequested',true);
lastPerceptionTime=max(get_field(map,'lastLidarTime',-inf),get_field(map,'lastDepthTime',-inf));
perceptionAge=tNow-lastPerceptionTime;
mapAcceptedPackets=double(get_field(map,'acceptedPackets',0));
mapPacketCountOK=mapAcceptedPackets>=cfg.mapPreflightMinAcceptedPackets;
perceptionOK=isfinite(perceptionAge)&&perceptionAge>=-cfg.dt&& ...
    perceptionAge<=cfg.mapPerceptionHoldTimeout_s&&mapPacketCountOK;
homeObserved=launch_footprint_observed_free(cfg,map,scenario.home,est.p(3));
startClear=homeObserved;
forced=logical(scenario.forcePreflightReject);
check=struct('horizontalAidsOK',hOK,'verticalAidOK',vOK,'attitudeAidOK',aOK, ...
    'activeLaneEligible',activeLaneEligible,'laneOK',laneOK,'covarianceOK',covOK, ...
    'updateCountOK',updateOK,'horizontalAidAge_s',hAge,'verticalAidAge_s',vAge, ...
    'attitudeAidAge_s',aAge,'xyCovariance',xyCov,'zVariance',zVar, ...
    'acceptedUpdates',updates,'perceptionOK',perceptionOK, ...
    'perceptionAge_s',perceptionAge,'lastPerceptionTime_s',lastPerceptionTime, ...
    'mapAcceptedPackets',mapAcceptedPackets,'mapPacketCountOK',mapPacketCountOK, ...
    'homeClear',homeObserved,'startCellClear',startClear, ...
    'goalCellClear',false,'goalReachable',false,'goalFree',false, ...
    'estimateFinite',estimateFinite,'forcedReject',forced, ...
    'astarExpanded',0,'preflightPath',zeros(0,2));
check.pass=hOK&&vOK&&aOK&&laneOK&&covOK&&updateOK&&perceptionOK&& ...
    homeObserved&&startClear&&estimateFinite&&~forced;
end
function value=get_field(s,name,defaultValue)
if isstruct(s)&&isfield(s,name)&&~isempty(s.(name)),value=s.(name);else,value=defaultValue;end
end

function tf=launch_footprint_observed_free(cfg,map,xy,z)
% Require measured free space around the launch footprint at the current
% sensor height. The z=0 map layer represents the near-ground air volume;
% floor contact itself is validated independently by the plant/landing model.
xy=double(xy(:).');
iz=max(1,min(map.nz,round((z-map.zs(1))/map.resolutionZ)+1));
[X,Y]=meshgrid(map.xs,map.ys);mask=(X-xy(1)).^2+(Y-xy(2)).^2<=cfg.mapTakeoffRadius_m^2;
lo=double(map.logOdds(:,:,iz));obs=double(map.observationCount(:,:,iz));
free=(lo<=log(cfg.mapFreeProbability/max(1-cfg.mapFreeProbability,eps)))& ...
    (obs>=cfg.mapMinFreeObservations);
tf=any(mask(:))&&all(free(mask));
end

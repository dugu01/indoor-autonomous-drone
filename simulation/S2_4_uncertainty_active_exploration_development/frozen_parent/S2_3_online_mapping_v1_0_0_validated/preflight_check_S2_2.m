function check = preflight_check_S2_2(cfg,scenario,grid,packet,est) %#ok<INUSD>
% PREFLIGHT_CHECK_S2_2 Estimator, aid-freshness and mission-geometry gates.
%
% Preflight uses the selected ESKF lane and the ages of measurements that
% were actually accepted by that lane. Instantaneous packet-arrival flags
% are not used because VIO, LiDAR, range and barometer are asynchronous.
% The home point is a landing site; the mission goal is an airborne hover
% point and is therefore checked as a centre-safe, reachable grid cell.

estimateFinite=all(isfinite(est.p))&&all(isfinite(est.v))&& ...
    all(isfinite(est.q))&&all(isfinite(est.P(:)));

horizontalAidAge_s=get_field(est,'horizontalAidAge_s',inf);
verticalAidAge_s=get_field(est,'verticalAidAge_s',inf);
attitudeAidAge_s=get_field(est,'attitudeAidAge_s',inf);
activeLaneEligible=logical(get_field(est,'activeLaneEligible',false));
xyCovariance=get_field(est,'xyCovariance',inf);
zVariance=get_field(est,'zVariance',inf);
acceptedUpdates=get_field(est,'acceptedMeasurementCount',0);

horizontalAidsOK=horizontalAidAge_s<=cfg.horizontalAidTimeout;
verticalAidOK=~cfg.preflightRequireVerticalAid|| ...
    verticalAidAge_s<=cfg.verticalAidTimeout;
attitudeAidOK=attitudeAidAge_s<=cfg.attitudeAidTimeout;
covarianceOK=xyCovariance<=cfg.preflightMaxXYCovariance&& ...
    zVariance<=cfg.preflightMaxZVariance;
updateCountOK=acceptedUpdates>=cfg.preflightMinAcceptedUpdates;
laneOK=activeLaneEligible&&~get_field(est,'degraded',true)&& ...
    ~get_field(est,'rtlRequested',true);

homeClear=landing_zone_clear_S2_2( ...
    grid,scenario.home,cfg.landingZoneExtraMargin_m);
goalCellClear=~segment_occupied_grid_S2_2( ...
    grid,scenario.goal,scenario.goal);
startCellClear=~segment_occupied_grid_S2_2( ...
    grid,est.p(1:2).',est.p(1:2).');

goalReachable=false;
preflightPath=zeros(0,2);
astarExpanded=0;
if estimateFinite&&startCellClear&&goalCellClear
    [preflightPath,info]=astar_grid_S2_2( ...
        grid,est.p(1:2).',scenario.goal);
    goalReachable=~isempty(preflightPath);
    astarExpanded=info.expanded;
end

forced=logical(scenario.forcePreflightReject);
check=struct();
check.horizontalAidsOK=horizontalAidsOK;
check.verticalAidOK=verticalAidOK;
check.attitudeAidOK=attitudeAidOK;
check.activeLaneEligible=activeLaneEligible;
check.laneOK=laneOK;
check.covarianceOK=covarianceOK;
check.updateCountOK=updateCountOK;
check.horizontalAidAge_s=horizontalAidAge_s;
check.verticalAidAge_s=verticalAidAge_s;
check.attitudeAidAge_s=attitudeAidAge_s;
check.xyCovariance=xyCovariance;
check.zVariance=zVariance;
check.acceptedUpdates=acceptedUpdates;
check.homeClear=homeClear;
check.startCellClear=startCellClear;
check.goalCellClear=goalCellClear;
check.goalReachable=goalReachable;
check.goalFree=goalCellClear; % backward-compatible field
check.estimateFinite=estimateFinite;
check.forcedReject=forced;
check.astarExpanded=astarExpanded;
check.preflightPath=preflightPath;
check.pass=horizontalAidsOK&&verticalAidOK&&attitudeAidOK&& ...
    laneOK&&covarianceOK&&updateCountOK&&homeClear&&startCellClear&& ...
    goalCellClear&&goalReachable&&estimateFinite&&~forced;
end

function value=get_field(s,name,defaultValue)
if isfield(s,name)&&~isempty(s.(name))
    value=s.(name);
else
    value=defaultValue;
end
end

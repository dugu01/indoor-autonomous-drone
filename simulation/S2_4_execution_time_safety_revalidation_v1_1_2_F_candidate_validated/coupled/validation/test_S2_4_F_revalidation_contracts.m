function out = test_S2_4_F_revalidation_contracts()
% TEST_S2_4_F_REVALIDATION_CONTRACTS Deterministic helper-level contracts.
cfg=init_S2_4_F_config();c=cfg.activeExploration;g=makeGrid();
r=makeRequest(g,c);v=[0;0];t=0.5;
[s1,~]=revalidate_active_exploration_request_S2_4_F(cfg,c,g,r,[1 1],v,t);
assert(s1.valid&&s1.leaseRenewed,'F1 unchanged request must remain valid and renew its current lease.');

g2=g;g2.occ(11,25)=true;g2.knownFree(11,25)=false;g2.mapVersion=uint32(2);
[s2,~]=revalidate_active_exploration_request_S2_4_F(cfg,c,g2,r,[1 1],v,t);
assert(~s2.valid&&~s2.forwardSafe,'F2 occupied future route not detected.');

g3=g;g3.unknown(11,25)=true;g3.knownFree(11,25)=false;g3.occ(11,25)=true;g3.mapVersion=uint32(3);
[s3,~]=revalidate_active_exploration_request_S2_4_F(cfg,c,g3,r,[1 1],v,t);
assert(~s3.valid&&~s3.forwardSafe,'F3 unknown future route not detected.');

g4=g;g4.occ(11,31)=true;g4.knownFree(11,31)=false;
[s4,~]=revalidate_active_exploration_request_S2_4_F(cfg,c,g4,r,[1 1],v,t);
assert(~s4.valid&&~s4.viewpointSafe,'F4 invalid viewpoint not detected.');

g5=g;g5.occ(12,31)=true;g5.knownFree(12,31)=false;
[s5,~]=revalidate_active_exploration_request_S2_4_F(cfg,c,g5,r,[1 1],v,t);
assert(~s5.valid&&~s5.holdSupportSafe,'F5 invalid hold support not detected.');

r6=r;r6.retreatRouteXY=[nan nan;nan nan];
[s6,r6b]=revalidate_active_exploration_request_S2_4_F(cfg,c,g,r6,[2 1],v,t);
assert(s6.valid&&s6.retreatRefreshed&&all(isfinite(r6b.retreatRouteXY(:))), ...
    'F6 invalid stored retreat was not refreshed from current known-free grid.');

g7=g;g7.mapVersion=uint32(9);
[s7,~]=revalidate_active_exploration_request_S2_4_F(cfg,c,g7,r,[1 1],v,t);
assert(s7.valid&&s7.mapChangedSinceRequest,'F7 unrelated map-version change must revalidate and continue.');

r8=r;r8.validUntil_s=0.25;
[s8,r8b]=revalidate_active_exploration_request_S2_4_F(cfg,c,g,r8,[1 1],v,t);
assert(~s8.valid&&~s8.leaseRenewed&&r8b.validUntil_s==r8.validUntil_s&& ...
    any(strcmp(s8.reasons,'REQUEST_EXPIRED')),'F8 expiry was revived or not detected.');

% F1 runtime/planning-gate regression. A cell 0.30 m lateral to the viewpoint
% lies outside the 3x3 hold support and does not affect the accepted route, but
% it lies inside the frozen S2.3 terminal landing-footprint check. Re-running
% that planning gate every execution cycle would therefore false-abort a safe
% unchanged route. The runtime gate must instead use remaining known-free stop
% reserve on the already-inflated execution grid.
gLegacy=g;gLegacy.occ(14,31)=true;gLegacy.knownFree(14,31)=false;
legacyPath=[1.2 1.0;1.5 1.0;2.0 1.0;2.5 1.0;3.0 1.0];
assert(~validate_known_free_stop_S2_3(cfg,gLegacy,legacyPath,[0.20;0]), ...
    'Regression fixture must reproduce the old planning-gate false abort.');
[sRuntime,~]=revalidate_active_exploration_request_S2_4_F(cfg,c,gLegacy,r,[1.2 1.0],[0.20;0],t);
assert(sRuntime.valid&&sRuntime.routeStopReserveSafe&&sRuntime.stoppingSupportSafe, ...
    'F1 runtime stop reserve falsely reused the planning-time terminal gate.');

% Near the endpoint, when route arc length is shorter than d_stop, current
% authority may continue only if the shortfall is covered by a known-free
% terminal overrun region on the already-inflated grid.
[sTerm,~]=revalidate_active_exploration_request_S2_4_F(cfg,c,g,r,[2.95 1.0],zeros(2,1),t);
assert(sTerm.valid&&sTerm.terminalOverrunReserveSafe,'F1 terminal stop-overrun reserve failed.');

% Do not weaken the stop gate: a short remaining route at high speed must fail
% if the required terminal overrun region is not known free.
gStop=g;gStop.occ(11,33)=true;gStop.knownFree(11,33)=false;
[sStop,~]=revalidate_active_exploration_request_S2_4_F(cfg,c,gStop,r,[2.90 1.0],[0.32;0],t);
assert(~sStop.valid&&~sStop.stoppingSupportSafe&&any(strcmp(sStop.reasons,'KNOWN_FREE_STOP_INVALID')), ...
    'Insufficient current stopping support was incorrectly accepted.');

% F11: a change behind current progress must not invalidate the remaining
% forward route. The retreat may be refreshed by current A* authority.
g11=g;g11.occ(11,16)=true;g11.knownFree(11,16)=false;g11.mapVersion=uint32(11);
[s11,~]=revalidate_active_exploration_request_S2_4_F(cfg,c,g11,r,[2 1],v,t);
assert(s11.forwardSafe&&s11.valid,'F11 behind-only change falsely aborted current authority.');

[s12a,r12a]=revalidate_active_exploration_request_S2_4_F(cfg,c,g,r,[1 1],v,t);
[s12b,r12b]=revalidate_active_exploration_request_S2_4_F(cfg,c,g,r,[1 1],v,t);
assert(isequaln(s12a,s12b)&&isequaln(r12a,r12b),'F12 repeated revalidation is not deterministic.');

out=struct('pass',true,'f15DynamicPredictionSupported',cfg.executionSafety.dynamicPredictionLiveSupported);
fprintf('S2.4-F deterministic revalidation contracts: PASS\n');
fprintf('F15 predictive moving-obstacle contract: NOT APPLICABLE (live predictor not connected)\n');
end
function g=makeGrid()
res=0.1;nx=51;ny=31;g=struct();g.resolution=res;g.nx=nx;g.ny=ny;g.xs=(0:nx-1)*res;g.ys=(0:ny-1)*res;
g.occ=false(ny,nx);g.knownFree=true(ny,nx);g.unknown=false(ny,nx);g.staticOccupied=false(ny,nx);g.dynamicOccupied=false(ny,nx);
g.mapVersion=uint32(1);g.timestamp=0;g.lastObservedXY=zeros(ny,nx);
end
function r=makeRequest(g,c)
xy=[1.0 1.0;1.5 1.0;2.0 1.0;2.5 1.0;3.0 1.0];
cells=[round(xy(:,2)/g.resolution)+1 round(xy(:,1)/g.resolution)+1];
r=struct('schema','S2_4_EXPLORATION_REQUEST_V1','valid',true,'action','MOVE_TO_VIEWPOINT', ...
    'requestId',uint64(1),'frontierTrackId',uint64(1),'candidateId',uint64(1), ...
    'positionXY',[3.0 1.0],'yaw',0,'startXY',[1.0 1.0],'pathCells',cells, ...
    'knownFreeRouteXY',xy,'retreatRouteXY',flipud(xy),'mapVersion',uint32(1), ...
    'createdTime_s',0,'validUntil_s',c.requestValidity_s,'utility',1,'tier',uint8(1), ...
    'informationGain',1,'targetRelevance',1,'rejectionReasons',{{}},'statusReason','VALID','commandIssued',false);
end

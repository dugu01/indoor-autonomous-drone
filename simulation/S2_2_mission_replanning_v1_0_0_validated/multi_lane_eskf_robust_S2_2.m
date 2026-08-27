function varargout = multi_lane_eskf_robust_S2_2(action,varargin)
% MULTI_LANE_ESKF_ROBUST_S2_2 v0.5.3 fault-aware four-lane navigation bridge.
% Lanes:
%   1 primary IMU + VIO + LiDAR + range + baro
%   2 backup  IMU + VIO + LiDAR + range + baro
%   3 primary IMU + VIO + range + baro
%   4 backup  IMU + LiDAR + range + baro
switch lower(action)
    case 'init'
        cfg=varargin{1};packet=varargin{2};t=varargin{3};
        nav=init_nav(cfg,packet,t);out=make_output(nav,cfg,packet,t);
        varargout={nav,out};
    case 'step'
        nav=varargin{1};cfg=varargin{2};packet=varargin{3};t=varargin{4};dt=varargin{5};
        [nav,out]=step_nav(nav,cfg,packet,t,dt);varargout={nav,out};
    otherwise
        error('S2_2:UnknownESKFAction','Unknown ESKF action: %s',action);
end
end

function nav=init_nav(cfg,packet,t)
if packet.hasVio
    p0=packet.vioP;v0=packet.vioV;q0=qnormalize_S2_2(packet.vioQ);
else
    p0=[3;0.8;cfg.altitudeNominal_m];v0=zeros(3,1);q0=[1 0 0 0];
end
names={'Primary IMU + all aids','Backup IMU + all aids','Primary IMU + VIO','Backup IMU + LiDAR'};
imuIds=[0 1 0 1];useVio=[true true true false];useLidar=[true true false true];
s=[repmat(0.05,1,3),repmat(0.12,1,3),deg2rad([3 3 4]),repmat(0.06,1,3),deg2rad([0.4 0.4 0.4]),0.20];
P0=diag(s.^2);
prototype=struct('id',0,'name','','imuId',0,'useVio',false,'useLidar',false, ...
    'useRange',true,'useBaro',true,'p',zeros(3,1),'v',zeros(3,1),'q',[1 0 0 0], ...
    'ba',zeros(3,1),'bg',zeros(3,1),'bbaro',0.12,'P',P0,'nisNormalized',zeros(0,1), ...
    'nisType',{{}},'acceptedCount',0,'rejectedCount',0,'lastHorizontalAidTime',t, ...
    'lastVerticalAidTime',t,'lastAttitudeAidTime',t,'score',inf,'eligible',false,'reason','initialising');
lanes=repmat(prototype,1,4);
for i=1:4
    lanes(i).id=i;lanes(i).name=names{i};lanes(i).imuId=imuIds(i);lanes(i).useVio=useVio(i);lanes(i).useLidar=useLidar(i);
    lanes(i).p=p0;lanes(i).v=v0;lanes(i).q=q0;lanes(i).P=P0;
end
selector=struct('activeLane',1,'candidateLane',0,'candidateSince',-inf,'lastSwitchTime',-inf, ...
    'switchCount',0,'offsetP',zeros(3,1),'offsetV',zeros(3,1),'offsetQ',[1 0 0 0], ...
    'blendStartTime',-inf,'blendDuration_s',cfg.outputBlendTime, ...
    'blendFaultAware',false,'maxSwitchJump',0);
nav=struct('lanes',lanes,'selector',selector,'imuDisagreementCount',0,'imuSuspect',-1, ...
    'imuFaultDetectedTime',nan,'imuFaultSuspectId',-1, ...
    'degradedStart',nan,'rtlRequested',false,'output',struct('p',p0,'v',v0,'q',q0), ...
    'switchLog',struct('time',{},'from',{},'to',{},'oldScore',{},'newScore',{}, ...
    'reason',{},'faultAware',{},'blendDuration_s',{},'faultDetectedTime',{}));
end

function [nav,out]=step_nav(nav,cfg,packet,t,dt)
for i=1:numel(nav.lanes)
    nav.lanes(i)=propagate_lane(nav.lanes(i),packet,dt,cfg);
end
if packet.hasVio
    Rvio=diag([repmat(cfg.vioPosSigma^2,1,3),repmat(cfg.vioVelSigma^2,1,3),repmat(cfg.vioAttSigma^2,1,3)]);
    for i=1:numel(nav.lanes)
        if nav.lanes(i).useVio
            r=[packet.vioP-nav.lanes(i).p;packet.vioV-nav.lanes(i).v; ...
                qlog_S2_2(qmul_S2_2(qconj_S2_2(nav.lanes(i).q),packet.vioQ)).'];
            H=zeros(9,16);H(1:3,1:3)=eye(3);H(4:6,4:6)=eye(3);H(7:9,7:9)=eye(3);
            [nav.lanes(i),ok,n]=filter_update(nav.lanes(i),r,H,Rvio,cfg.gateVIO9);
            nav.lanes(i)=record_nis(nav.lanes(i),n,cfg.gateVIO9,'vio',ok);
            if ok,nav.lanes(i).lastHorizontalAidTime=t;nav.lanes(i).lastVerticalAidTime=t;nav.lanes(i).lastAttitudeAidTime=t;end
        end
    end
end
if packet.hasLidar
    Rlid=diag([cfg.lidarSigmaXY^2,cfg.lidarSigmaXY^2,cfg.lidarSigmaYaw^2]);
    for i=1:numel(nav.lanes)
        if nav.lanes(i).useLidar
            yaw=q2rpy_S2_2(nav.lanes(i).q);r=[packet.lidarXY-nav.lanes(i).p(1:2);wrap_pi_S2_2(packet.lidarYaw-yaw(3))];
            H=zeros(3,16);H(1,1)=1;H(2,2)=1;H(3,9)=1;
            [nav.lanes(i),ok,n]=filter_update(nav.lanes(i),r,H,Rlid,cfg.gateLidar3);
            nav.lanes(i)=record_nis(nav.lanes(i),n,cfg.gateLidar3,'lidar',ok);
            if ok,nav.lanes(i).lastHorizontalAidTime=t;nav.lanes(i).lastAttitudeAidTime=t;end
        end
    end
end
if packet.hasRange
    for i=1:numel(nav.lanes)
        H=zeros(1,16);H(3)=1;r=packet.range-nav.lanes(i).p(3);
        [nav.lanes(i),ok,n]=filter_update(nav.lanes(i),r,H,cfg.rangeSigma^2,cfg.gate1);
        nav.lanes(i)=record_nis(nav.lanes(i),n,cfg.gate1,'range',ok);
        if ok,nav.lanes(i).lastVerticalAidTime=t;end
    end
end
if packet.hasBaro
    for i=1:numel(nav.lanes)
        H=zeros(1,16);H(3)=1;H(16)=1;r=packet.baro-(nav.lanes(i).p(3)+nav.lanes(i).bbaro);
        [nav.lanes(i),ok,n]=filter_update(nav.lanes(i),r,H,cfg.baroSigma^2,cfg.gate1);
        nav.lanes(i)=record_nis(nav.lanes(i),n,cfg.gate1,'baro',ok);
        if ok,nav.lanes(i).lastVerticalAidTime=t;end
    end
end

da=norm(packet.acc0-packet.acc1);dg=norm(packet.gyro0-packet.gyro1);
if da>cfg.imuDisagreementAccel||dg>cfg.imuDisagreementGyro
    nav.imuDisagreementCount=nav.imuDisagreementCount+1;
else
    nav.imuDisagreementCount=max(0,nav.imuDisagreementCount-1);
end
previousSuspect=nav.imuSuspect;
if nav.imuDisagreementCount>=cfg.imuDisagreementSamples
    % Attribute the disagreement from the two all-aid lanes using recent
    % normalized innovations. A short recent window reacts to a new IMU
    % fault without waiting for the long health-score median to turn over.
    scorePrimary=imu_attribution_score(nav.lanes(1),t,cfg);
    scoreBackup=imu_attribution_score(nav.lanes(2),t,cfg);
    if scorePrimary>scoreBackup+cfg.imuGroupScoreMargin
        nav.imuSuspect=0;
    elseif scoreBackup>scorePrimary+cfg.imuGroupScoreMargin
        nav.imuSuspect=1;
    end
elseif nav.imuDisagreementCount==0
    nav.imuSuspect=-1;
end
if nav.imuSuspect>=0&&nav.imuSuspect~=previousSuspect
    nav.imuFaultDetectedTime=t;
    nav.imuFaultSuspectId=nav.imuSuspect;
end

old=nav.output;
[nav.selector,scores,eligible,degraded,nav.lanes,sw]=choose_lane(nav.lanes,nav.selector,t,cfg,old,nav.imuSuspect,nav.imuFaultDetectedTime);
if ~isempty(sw),nav.switchLog(end+1)=sw;end %#ok<AGROW>
if degraded
    if isnan(nav.degradedStart),nav.degradedStart=t;end
else
    nav.degradedStart=nan;
end
nav.rtlRequested=~isnan(nav.degradedStart)&&(t-nav.degradedStart)>=cfg.degradedRTLDelay;
out=make_output(nav,cfg,packet,t);
out.scores=scores;out.eligible=eligible;out.degraded=degraded;out.rtlRequested=nav.rtlRequested;
nav.output=struct('p',out.p,'v',out.v,'q',out.q);
end

function lane=propagate_lane(lane,packet,dt,cfg)
if lane.imuId==1,acc=packet.acc1;gyro=packet.gyro1;else,acc=packet.acc0;gyro=packet.gyro0;end
fb=acc-lane.ba;w=gyro-lane.bg;R=q2R_S2_2(lane.q);aw=R*fb+cfg.gW;
lane.p=lane.p+lane.v*dt+0.5*aw*dt^2;lane.v=lane.v+aw*dt;lane.q=qmul_S2_2(lane.q,qexp_S2_2((w*dt).'));
F=zeros(16);F(1:3,4:6)=eye(3);F(4:6,7:9)=-R*skew3_S2_2(fb);F(4:6,10:12)=-R;F(7:9,7:9)=-skew3_S2_2(w);F(7:9,13:15)=-eye(3);
Phi=eye(16)+F*dt;G=zeros(16,13);G(4:6,1:3)=-R;G(7:9,4:6)=-eye(3);G(10:12,7:9)=eye(3);G(13:15,10:12)=eye(3);G(16,13)=1;
qc=[repmat(cfg.accelND^2,1,3),repmat(cfg.gyroND^2,1,3),repmat(cfg.accelBiasRW^2,1,3),repmat(cfg.gyroBiasRW^2,1,3),cfg.baroBiasRW^2];
lane.P=Phi*lane.P*Phi'+G*diag(qc)*G'*dt;lane.P=(lane.P+lane.P')/2;
end

function [lane,ok,nis]=filter_update(lane,r,H,Rm,gate)
r=r(:);S=H*lane.P*H'+Rm;
if any(~isfinite(S(:)))||rcond(S)<1e-12,ok=false;nis=inf;return;end
nis=r'*(S\r);
if ~isfinite(nis)||nis>gate,ok=false;return;end
K=(lane.P*H')/S;dx=K*r;lane.p=lane.p+dx(1:3);lane.v=lane.v+dx(4:6);lane.q=qmul_S2_2(lane.q,qexp_S2_2(dx(7:9).'));
lane.ba=lane.ba+dx(10:12);lane.bg=lane.bg+dx(13:15);lane.bbaro=lane.bbaro+dx(16);
I=eye(16);J=I-K*H;lane.P=J*lane.P*J'+K*Rm*K';G=eye(16);G(7:9,7:9)=eye(3)-0.5*skew3_S2_2(dx(7:9));lane.P=G*lane.P*G';lane.P=(lane.P+lane.P')/2;ok=true;
end

function lane=record_nis(lane,nis,gate,typeName,accepted)
if isfinite(nis)&&gate>0,lane.nisNormalized(end+1,1)=nis/gate;lane.nisType{end+1,1}=typeName;end
if accepted,lane.acceptedCount=lane.acceptedCount+1;else,lane.rejectedCount=lane.rejectedCount+1;end
end

function [selector,scores,eligible,degraded,lanes,sw]=choose_lane(lanes,selector,t,cfg,old,imuSuspect,faultDetectedTime)
n=numel(lanes);scores=inf(n,1);eligible=false(n,1);sw=[];
for i=1:n
    [scores(i),eligible(i),lanes(i).reason]=lane_health(lanes(i),t,cfg);
    if imuSuspect>=0&&lanes(i).imuId==imuSuspect,scores(i)=scores(i)+50;end
    lanes(i).score=scores(i);lanes(i).eligible=eligible(i);
end
degraded=~any(eligible);if degraded,selector.candidateLane=0;return;end
active=selector.activeLane;tmp=scores;tmp(~eligible)=inf;[bestScore,best]=min(tmp);improvement=scores(active)-bestScore;activeBad=~eligible(active);
fast=(imuSuspect>=0&&lanes(active).imuId==imuSuspect);if fast,margin=cfg.laneFastSwitchMargin;confirm=cfg.laneFastConfirmTime;else,margin=cfg.laneSwitchMargin;confirm=cfg.laneConfirmTime;end
if best==active||(~activeBad&&improvement<margin),selector.candidateLane=0;return;end
if ~activeBad
    if norm(lanes(best).p-lanes(active).p)>cfg.maxLanePositionJump||norm(qlog_S2_2(qmul_S2_2(qconj_S2_2(lanes(active).q),lanes(best).q)))>cfg.maxLaneAttitudeJump
        selector.candidateLane=0;return;
    end
end
if t-selector.lastSwitchTime<cfg.laneMinDwellTime,return;end
if selector.candidateLane~=best,selector.candidateLane=best;selector.candidateSince=t;return;end
if t-selector.candidateSince<confirm,return;end
oldId=active;selector.activeLane=best;selector.lastSwitchTime=t;selector.switchCount=selector.switchCount+1;selector.candidateLane=0;
selector.offsetP=old.p-lanes(best).p;selector.offsetV=old.v-lanes(best).v;selector.offsetQ=qmul_S2_2(qconj_S2_2(lanes(best).q),old.q);selector.blendStartTime=t;
selector.blendFaultAware=logical(fast);
if fast,selector.blendDuration_s=cfg.outputBlendTimeFault_s;else,selector.blendDuration_s=cfg.outputBlendTime;end
selector.maxSwitchJump=max(selector.maxSwitchJump,norm(selector.offsetP));
sw=struct('time',t,'from',oldId,'to',best,'oldScore',scores(oldId),'newScore',scores(best), ...
    'reason',lanes(oldId).reason,'faultAware',logical(fast), ...
    'blendDuration_s',selector.blendDuration_s,'faultDetectedTime',faultDetectedTime);
end

function score=imu_attribution_score(lane,t,cfg)
% Fast, causal score used only after persistent cross-IMU disagreement.
[base,~,~]=lane_health(lane,t,cfg);
n=numel(lane.nisNormalized);
first=max(1,n-cfg.imuAttributionWindow+1);
recent=lane.nisNormalized(first:n);
recent=recent(isfinite(recent));
if isempty(recent)
    recentScore=base;
else
    recentScore=max(min(recent,8));
end
w=min(1,max(0,cfg.imuAttributionRecentWeight));
score=(1-w)*base+w*recentScore;
end

function [score,eligible,reason]=lane_health(lane,t,cfg)
xyAge=t-lane.lastHorizontalAidTime;zAge=t-lane.lastVerticalAidTime;pxy=trace(lane.P(1:2,1:2));pz=lane.P(3,3);
finiteState=all(isfinite([lane.p;lane.v;lane.q(:);diag(lane.P)]));eligible=finiteState&&xyAge<=cfg.horizontalAidTimeout&&zAge<=cfg.verticalAidTimeout&&pxy<=cfg.maxXYCovariance&&pz<=cfg.maxZCovariance;
nNis=numel(lane.nisNormalized);first=max(1,nNis-cfg.laneNisWindow+1);vals=lane.nisNormalized(first:nNis);vals=vals(isfinite(vals));if isempty(vals),score=1;else,score=median(min(vals,8));end
score=score+cfg.lanePriorityPenalty(lane.id)+3*min(pxy/cfg.maxXYCovariance,10)+min(pz/cfg.maxZCovariance,10)+2*max(0,xyAge-0.25);
reasons={};if ~finiteState,reasons{end+1}='nonfinite';end;if xyAge>cfg.horizontalAidTimeout,reasons{end+1}='XY stale';end;if zAge>cfg.verticalAidTimeout,reasons{end+1}='Z stale';end;if pxy>cfg.maxXYCovariance,reasons{end+1}='Pxy high';end;if pz>cfg.maxZCovariance,reasons{end+1}='Pz high';end
if ~eligible,score=score+100;end;if isempty(reasons),reason='healthy';else,reason=strjoin(reasons,', ');end
end

function out=make_output(nav,cfg,packet,t)
L=nav.lanes(nav.selector.activeLane);p=L.p;v=L.v;q=L.q;
if isfinite(nav.selector.blendStartTime)
    elapsed=max(0,t-nav.selector.blendStartTime);
    blendDuration=max(nav.selector.blendDuration_s,eps);
    if elapsed<blendDuration
        rem=1-elapsed/blendDuration;p=p+rem*nav.selector.offsetP;v=v+rem*nav.selector.offsetV;q=qmul_S2_2(q,qexp_S2_2(rem*qlog_S2_2(nav.selector.offsetQ)));
    end
end
if L.imuId==1,omega=packet.gyro1-L.bg;else,omega=packet.gyro0-L.bg;end
Pxy=(L.P(1:2,1:2)+L.P(1:2,1:2)')/2;e=eig(Pxy);xySigma=sqrt(max(max(e),0));
out=struct('p',p,'v',v,'q',q,'omega',omega,'activeLane',nav.selector.activeLane, ...
    'laneSwitches',nav.selector.switchCount,'P',L.P,'xySigma_m',xySigma,'scores',[],'eligible',[], ...
    'degraded',false,'rtlRequested',nav.rtlRequested,'imuSuspect',nav.imuSuspect, ...
    'imuFaultDetectedTime',nav.imuFaultDetectedTime,'faultAwareBlend',nav.selector.blendFaultAware, ...
    'blendDuration_s',nav.selector.blendDuration_s);
end

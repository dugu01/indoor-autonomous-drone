function [packet,model] = simulate_sensor_packet_S2_5(cfg,scenario,truth,model,t,k)
% SIMULATE_SENSOR_PACKET_S2_5 S2.2 sensor model plus S2.5 qualification faults.
% The nominal equations/noise values are copied from the frozen S2.2 model.
% S2.5 faults modify only the synthetic autonomy-visible packet.
model=init_stats(model);
if t>=scenario.primaryImuBiasStepTime
    model.ba0=scenario.primaryAccelBiasStep(:);
    model.bg0=scenario.primaryGyroBiasStep(:);
end
if t>=scenario.backupImuBiasStepTime
    model.ba1=scenario.backupAccelBiasStep(:);
    model.bg1=scenario.backupGyroBiasStep(:);
end
R=q2R_S2_2(truth.q);
specificForce=R'*(truth.a-cfg.gW);
noiseScale=scenario.measurementNoiseScale;
packet=struct();
packet.acc0=specificForce+model.ba0+noiseScale*cfg.imuAccelNoiseSigma*randn(3,1);
packet.gyro0=truth.omega+model.bg0+noiseScale*cfg.imuGyroNoiseSigma*randn(3,1);
packet.acc1=specificForce+model.ba1+1.15*noiseScale*cfg.imuAccelNoiseSigma*randn(3,1);
packet.gyro1=truth.omega+model.bg1+1.15*noiseScale*cfg.imuGyroNoiseSigma*randn(3,1);
packet.hasVio=mod(k-1,cfg.vioPeriodSteps)==0 && ~inside_windows_local(t,scenario.vioOutageWindows);
packet.hasLidar=mod(k-1,cfg.lidarPeriodSteps)==0 && ~inside_windows_local(t,scenario.lidarOutageWindows);
packet.hasRange=mod(k-1,cfg.rangePeriodSteps)==0 && ~inside_windows_local(t,scenario.rangeOutageWindows);
packet.hasBaro=mod(k-1,cfg.baroPeriodSteps)==0 && ~inside_windows_local(t,scenario.baroOutageWindows);
packet.groundContact=logical(isfield(truth,'onGround')&&truth.onGround);
if packet.hasVio
    packet.vioP=truth.p+noiseScale*cfg.vioPosSigma*randn(3,1);
    packet.vioV=truth.v+noiseScale*cfg.vioVelSigma*randn(3,1);
    packet.vioQ=qmul_S2_2(truth.q,qexp_S2_2((noiseScale*cfg.vioAttSigma*randn(3,1)).'));
else
    packet.vioP=[];packet.vioV=[];packet.vioQ=[];
end
if packet.hasLidar
    packet.lidarXY=truth.p(1:2)+noiseScale*cfg.lidarSigmaXY*randn(2,1);
    rpy=q2rpy_S2_2(truth.q);
    packet.lidarYaw=wrap_pi_S2_2(rpy(3)+noiseScale*cfg.lidarSigmaYaw*randn);
else
    packet.lidarXY=[];packet.lidarYaw=[];
end
if packet.hasRange,packet.range=truth.p(3)+noiseScale*cfg.rangeSigma*randn;else,packet.range=[];end
if packet.hasBaro,packet.baro=truth.p(3)+model.baroBias+noiseScale*cfg.baroSigma*randn;else,packet.baro=[];end

fault=field_or_local(scenario,'s25NavigationFault',struct('name','NONE','start_s',inf,'end_s',-inf));
active=in_window(t,field_or_local(fault,'start_s',inf),field_or_local(fault,'end_s',-inf));
applied=false;
name=upper(char(field_or_local(fault,'name','NONE')));
if active
    switch name
        case 'VIO_DROPOUT'
            if packet.hasVio,applied=true;end
            packet.hasVio=false;packet.vioP=[];packet.vioV=[];packet.vioQ=[];
        case 'LIDAR_AID_DROPOUT'
            if packet.hasLidar,applied=true;end
            packet.hasLidar=false;packet.lidarXY=[];packet.lidarYaw=[];
        case 'VIO_OUTLIER_BURST'
            if packet.hasVio
                packet.vioP=packet.vioP+field_or_local(fault,'vioPositionBias_m',zeros(3,1));
                packet.vioV=packet.vioV+field_or_local(fault,'vioVelocityBias_mps',zeros(3,1));
                packet.vioQ=qmul_S2_2(packet.vioQ,qexp_S2_2(field_or_local(fault,'vioAttitudeBias_rad',zeros(3,1))));
                applied=true;
            end
        case 'LIDAR_OUTLIER_BURST'
            if packet.hasLidar
                packet.lidarXY=packet.lidarXY+field_or_local(fault,'lidarXYBias_m',zeros(2,1));
                packet.lidarYaw=wrap_pi_S2_2(packet.lidarYaw+field_or_local(fault,'lidarYawBias_rad',0));
                applied=true;
            end
        case 'RANGE_OUTLIER_BURST'
            if packet.hasRange,packet.range=packet.range+field_or_local(fault,'rangeBias_m',0);applied=true;end
        case 'BARO_OUTLIER_BURST'
            if packet.hasBaro,packet.baro=packet.baro+field_or_local(fault,'baroBias_m',0);applied=true;end
        case 'NONE'
            % no-op
        otherwise
            error('S2_5:UnsupportedNavigationFault','Unsupported S2.5 navigation fault: %s',name);
    end
end
packet.s25FaultName=name;packet.s25FaultActive=active;packet.s25FaultApplied=applied;
if applied
    model.s25FaultApplicationCount=model.s25FaultApplicationCount+1;
    if ~isfinite(model.s25FirstFaultTime_s),model.s25FirstFaultTime_s=t;end
    model.s25LastFaultTime_s=t;
end
end

function model=init_stats(model)
if ~isfield(model,'s25FaultApplicationCount'),model.s25FaultApplicationCount=0;end
if ~isfield(model,'s25FirstFaultTime_s'),model.s25FirstFaultTime_s=nan;end
if ~isfield(model,'s25LastFaultTime_s'),model.s25LastFaultTime_s=nan;end
end
function tf=inside_windows_local(t,w)
tf=false;for i=1:size(w,1),if t>=w(i,1)&&t<=w(i,2),tf=true;return;end,end
end
function tf=in_window(t,t0,t1),tf=isfinite(t0)&&t>=t0&&t<=t1;end
function v=field_or_local(s,n,d),if isstruct(s)&&isfield(s,n),v=s.(n);else,v=d;end,end

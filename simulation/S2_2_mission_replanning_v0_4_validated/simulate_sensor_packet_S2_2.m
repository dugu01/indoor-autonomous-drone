function [packet,model] = simulate_sensor_packet_S2_2(cfg,scenario,truth,model,t,k)
% SIMULATE_SENSOR_PACKET_S2_2 Synthetic onboard sensor packet.
% LiDAR is represented by the accepted local pose output of the S2.1 front
% end; raw scan matching remains in the frozen S2.1 package.

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
end
function tf=inside_windows_local(t,w)
tf=false;
for i=1:size(w,1),if t>=w(i,1)&&t<=w(i,2),tf=true;return;end,end
end

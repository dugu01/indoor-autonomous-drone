function results = run_S2_2_mission_replanning(seed,scenarioName,makePlots,makeAnimation)
% RUN_S2_2_MISSION_REPLANNING Stage S2.2 v0.5.3.2 multi-seed robustness correction.
if nargin<1||isempty(seed),seed=0;end
if nargin<2||isempty(scenarioName),scenarioName='full_mission_nominal';end
if nargin<3||isempty(makePlots),makePlots=true;end
if nargin<4||isempty(makeAnimation),makeAnimation=false;end
rng(seed,'twister');scriptDir=fileparts(mfilename('fullpath'));addpath(scriptDir,'-begin');cfg=init_S2_2_config();cfg.seed=seed;
versionFolder=lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_'));cfg.resultsRoot=fullfile(fileparts(scriptDir),'results','S2_2_mission_replanning',versionFolder);
if ~exist(cfg.resultsRoot,'dir'),mkdir(cfg.resultsRoot);end
scenario=scenario_S2_2(scenarioName);label=lower(regexprep(scenario.name,'[^A-Za-z0-9]+','_'));
resultsDir=fullfile(cfg.resultsRoot,label,sprintf('seed_%03d',seed));if ~exist(resultsDir,'dir'),mkdir(resultsDir);end
fprintf('\n============================================================\n');
fprintf(' STAGE S2.2 %s AUTONOMOUS MISSION + 6-DOF REPLANNING\n',cfg.version);
fprintf(' seed=%d | scenario=%s | plots=%d | animation=%d\n',seed,scenario.name,makePlots,makeAnimation);
fprintf(' Results: %s\n',resultsDir);fprintf('============================================================\n');
[log,summary,maps]=mission_manager_S2_2(cfg,scenario);summary.seed=seed;summary.scenario=scenario.name;summary.outputDir=resultsDir;

fprintf('Goal / failsafe / RTL  : %d / %d / %d\n',summary.goalReached,summary.failsafeTriggered,summary.rtlRequested);
if isfield(summary,'lifecycleEnabled')&&summary.lifecycleEnabled
    fprintf('Arm/takeoff/RTL/land   : %d / %d / %d / %d\n',summary.armedEver,summary.takeoffCompleted,summary.rtlExecuted,summary.landed);
    fprintf('Disarm/preflight/emerg : %d / %d / %d\n',summary.disarmed,summary.preflightRejected,summary.emergencyLanding);
    fprintf('Landing alternate/xy   : %d / [%.2f %.2f]\n',summary.alternateLandingUsed,summary.selectedLandingXY(1),summary.selectedLandingXY(2));
    fprintf('RTL midcourse replans  : %d\n',summary.rtlMidcourseReplanCount);
    fprintf('Lifecycle transitions  : %d | timeout %d\n',summary.stateTransitionCount,summary.stateTimeoutTriggered);
    if isfield(summary,'preflightCheck')
        pf=summary.preflightCheck;
        fprintf(['Preflight H/V/A/L/C/U/home/goal/route: ' ...
            '%d / %d / %d / %d / %d / %d / %d / %d / %d\n'], ...
            pf.horizontalAidsOK,pf.verticalAidOK,pf.attitudeAidOK, ...
            pf.laneOK,pf.covarianceOK,pf.updateCountOK,pf.homeClear, ...
            pf.goalCellClear,pf.goalReachable);
        fprintf(['Preflight age XY/Z/att : %.3f / %.3f / %.3f s | ' ...
            'accepted %d | Pxy/Pz %.3e / %.3e\n'], ...
            pf.horizontalAidAge_s,pf.verticalAidAge_s,pf.attitudeAidAge_s, ...
            pf.acceptedUpdates,pf.xyCovariance,pf.zVariance);
    end
    fprintf('Truth validation G/T/L : %d / %d / %d | complete %d\n', ...
        summary.truthGoalReached,summary.truthTakeoffReached, ...
        summary.truthLanded,summary.missionComplete);
end
fprintf('Replans/promotions/infl: %d / %d / %d\n',summary.replanCount,summary.promotionCount,summary.inflationReplanCount);
fprintf('Dynamic avoid/holds    : %d / %d\n',summary.dynamicAvoidSteps,summary.dynamicHoldCount);
fprintf('Obstacle no-data holds : %d\n',summary.obstacleNoDataHoldCount);
fprintf('Replan brake / retries : %d / %d\n',summary.replanBrakeCount,summary.replanRetryCount);
if isfield(summary,'progressRecoveryCount')
    fprintf('Progress/grid recovery : %d / %d\n', ...
        summary.progressRecoveryCount,field_or(summary,'gridFallbackCount',0));
end
fprintf('Lane final / switches  : %d / %d\n',summary.activeLaneFinal,summary.laneSwitches);
if isfield(summary,'faultSwitchTime_s')
    fprintf('IMU detect/switch/blend: %.3f / %.3f / %.3f s\n', ...
        field_or(summary,'imuFaultDetectedTime_s',nan), ...
        summary.faultSwitchTime_s,field_or(summary,'faultBlendDuration_s',nan));
end
fprintf('D* repair / A* scratch : %d / %d expanded nodes\n',summary.dstarRepairExpanded,summary.astarScratchExpanded);
fprintf('Clearance obs/wall/dyn : %.3f / %.3f / %.3f m\n',summary.minObstacleClearance_m,summary.minWallClearance_m,summary.minDynamicClearance_m);
fprintf('Reference XY v/a/j     : %.3f / %.3f / %.3f\n',summary.maxReferenceSpeed_mps,summary.maxReferenceAccel_mps2,summary.maxReferenceJerk_mps3);
if isfield(summary,'maxVerticalReferenceSpeed_mps')
    fprintf('Reference Z v/a/j      : %.3f / %.3f / %.3f\n',summary.maxVerticalReferenceSpeed_mps,summary.maxVerticalReferenceAccel_mps2,summary.maxVerticalReferenceJerk_mps3);
end
fprintf('Executed XY v/a/j      : %.3f / %.3f / %.3f\n',summary.maxExecutedSpeed_mps,summary.maxExecutedAccel_mps2,summary.maxExecutedJerk_mps3);
if isfield(summary,'maxExecutedVerticalSpeed_mps')
    fprintf('Executed Z v/a/j       : %.3f / %.3f / %.3f\n',summary.maxExecutedVerticalSpeed_mps,summary.maxExecutedVerticalAccel_mps2,summary.maxExecutedVerticalJerk_mps3);
end
fprintf('Estimator pos/att max  : %.3f m / %.3f deg\n',summary.maxEstimatorPositionError_m,summary.maxEstimatorAttitudeError_deg);
if isfield(summary,'lifecycleEnabled')&&summary.lifecycleEnabled
    fprintf('Estimator obs/trig/post: %.3f / %.3f / %.3f m\n', ...
        summary.maxEstimatorPositionErrorObservable_m, ...
        summary.estimatorPositionErrorAtFailsafeTrigger_m, ...
        summary.maxEstimatorPositionErrorPostLoss_m);
    fprintf('Degraded holds / drift : %d / %.3f m\n', ...
        summary.navigationDegradedHoldCount,summary.maxEmergencyHorizontalDrift_m);
    fprintf('XY loss/brake end      : %.3f / %.3f s | |a| %.3f m/s^2\n', ...
        field_or(summary,'xyLossDetectionTime_s',nan), ...
        field_or(summary,'xyLossBrakeEndTime_s',nan), ...
        norm(field_or(summary,'xyLossBrakeAccel_mps2',[nan nan])));
    fprintf('Brake release t/speed  : %.3f s / %.3f m/s\n', ...
        field_or(summary,'xyLossBrakeReleaseTime_s',nan), ...
        field_or(summary,'xyLossBrakeReleaseSpeed_mps',nan));
end
fprintf('Track/override/alt     : %.3f / %.3f / %.3f m\n',summary.maxTrackingError_m,summary.maxSafetyOverrideDeviation_m,summary.maxAltitudeError_m);
fprintf('Tilt / inflation max   : %.3f deg / %.3f m\n',summary.maxTilt_deg,summary.maxInflationRadius_m);
fprintf('Trajectories/fallbacks : %d / %d | max scale %.2f\n',summary.trajectoryGenerationCount,summary.trajectoryFallbackCount,summary.maxTrajectoryTimeScale);
fprintf('Core S/D/RK/EK/C/E/U   : %d / %d / %d / %d / %d / %d / %d\n', ...
    summary.staticPass,summary.dynamicPass,summary.referenceKinematicPass,summary.executedKinematicPass, ...
    summary.controllerPass,summary.estimatorPositionPass&&summary.estimatorAttitudePass,summary.uncertaintyPass);
fprintf('Mission/failsafe/event : %d / %d / %d\n',summary.missionOutcomePass,summary.failsafeExpectationPass,summary.eventPass);
if isfield(summary,'finalState')
    fprintf('Final state / distance : %s / %.3f m\n', ...
        summary.finalState,field_or(summary,'finalDistanceToGoal_m',nan));
end
fprintf('Collision / geofence   : %d / %d\n',summary.collisionCount,summary.geofenceViolationCount);
fprintf('RESULT                 : %s\n\n',ternary(summary.pass,'PASS','FAIL'));

plotFiles={};if makePlots,plotFiles=plot_S2_2_dashboard(cfg,scenario,log,summary,maps,resultsDir);end
animationFile='';if makeAnimation,animationFile=animate_S2_2_flight(cfg,scenario,log,summary,maps,resultsDir);end
matName=sprintf('S2_2_%s_trial_data.mat',versionFolder);save(fullfile(resultsDir,matName),'cfg','scenario','log','summary','maps','-v7.3');
write_summary(summary,resultsDir,cfg);
results=struct('summary',summary,'log',log,'maps',maps,'plotFiles',{plotFiles},'animationFile',animationFile,'outputDir',resultsDir);
end

function write_summary(s,d,cfg)
versionFolder=lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_'));
f=fopen(fullfile(d,sprintf('summary_S2_2_%s.txt',versionFolder)),'w');if f<0,return;end;c=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'Stage S2.2 %s autonomous mission and 6-DOF replanning\nScenario: %s | seed %d\n',cfg.version,s.scenario,s.seed);
fprintf(f,'Goal %d | failsafe %d | RTL request %d | PASS %d\n',s.goalReached,s.failsafeTriggered,s.rtlRequested,s.pass);
if isfield(s,'lifecycleEnabled')&&s.lifecycleEnabled
    fprintf(f,'Arm %d | takeoff %d | RTL execute %d | landed %d | disarmed %d\n',s.armedEver,s.takeoffCompleted,s.rtlExecuted,s.landed,s.disarmed);
    fprintf(f,'Preflight reject %d | emergency land %d | alternate %d | site %.6f %.6f\n',s.preflightRejected,s.emergencyLanding,s.alternateLandingUsed,s.selectedLandingXY(1),s.selectedLandingXY(2));
    fprintf(f,'Truth goal/takeoff/land %d %d %d | complete %d\n',s.truthGoalReached,s.truthTakeoffReached,s.truthLanded,s.missionComplete);
    if isfield(s,'preflightCheck')
        pf=s.preflightCheck;
        fprintf(f,'Preflight H/V/A/L/C/U/home/goal/route %d %d %d %d %d %d %d %d %d\n', ...
            pf.horizontalAidsOK,pf.verticalAidOK,pf.attitudeAidOK,pf.laneOK, ...
            pf.covarianceOK,pf.updateCountOK,pf.homeClear,pf.goalCellClear,pf.goalReachable);
        fprintf(f,'Preflight ages XY/Z/att %.8f %.8f %.8f | accepted %d | Pxy/Pz %.8e %.8e\n', ...
            pf.horizontalAidAge_s,pf.verticalAidAge_s,pf.attitudeAidAge_s, ...
            pf.acceptedUpdates,pf.xyCovariance,pf.zVariance);
    end
end
fprintf(f,'Replans %d | inflation replans %d | brake %d | retries %d\n',s.replanCount,s.inflationReplanCount,s.replanBrakeCount,s.replanRetryCount);
if isfield(s,'progressRecoveryCount')
    fprintf(f,'Progress recoveries %d | grid fallback %d | max rejoin %.8f\n', ...
        s.progressRecoveryCount,field_or(s,'gridFallbackCount',0), ...
        field_or(s,'maxRejoinDuration_s',nan));
end
fprintf(f,'Lane final %d | switches %d\n',s.activeLaneFinal,s.laneSwitches);
if isfield(s,'faultSwitchTime_s')
    fprintf(f,'IMU fault detect/switch/blend %.8f %.8f %.8f\n', ...
        field_or(s,'imuFaultDetectedTime_s',nan),s.faultSwitchTime_s, ...
        field_or(s,'faultBlendDuration_s',nan));
end
fprintf(f,'Estimator max position %.8f | attitude deg %.8f\n',s.maxEstimatorPositionError_m,s.maxEstimatorAttitudeError_deg);

% These diagnostics exist only for the v0.5.3.2 lifecycle manager.  The seven
% preserved v0.4 regression scenarios intentionally return the frozen v0.4
% summary schema, so never access lifecycle-only fields for those trials.
isLifecycle = isfield(s,'lifecycleEnabled') && s.lifecycleEnabled;
if isLifecycle
    fprintf(f,'Estimator observable/trigger/postloss %.8f %.8f %.8f | metric %.8f\n', ...
        s.maxEstimatorPositionErrorObservable_m, ...
        s.estimatorPositionErrorAtFailsafeTrigger_m, ...
        s.maxEstimatorPositionErrorPostLoss_m, ...
        s.estimatorFailsafeMetric_m);
    fprintf(f,'Navigation degraded holds %d | emergency horizontal drift %.8f\n', ...
        s.navigationDegradedHoldCount,s.maxEmergencyHorizontalDrift_m);
    fprintf(f,'XY loss time %.8f | brake end %.8f | brake accel %.8f %.8f\n', ...
        field_or(s,'xyLossDetectionTime_s',nan),field_or(s,'xyLossBrakeEndTime_s',nan), ...
        field_or(s,'xyLossBrakeAccel_mps2',[nan nan]));
    fprintf(f,'Brake release time/speed %.8f %.8f\n', ...
        field_or(s,'xyLossBrakeReleaseTime_s',nan), ...
        field_or(s,'xyLossBrakeReleaseSpeed_mps',nan));
    fprintf(f,'Grid fallback count %d\n',field_or(s,'gridFallbackCount',0));
end
fprintf(f,'Reference XY v/a/j %.8f %.8f %.8f\n',s.maxReferenceSpeed_mps,s.maxReferenceAccel_mps2,s.maxReferenceJerk_mps3);
fprintf(f,'Executed XY v/a/j %.8f %.8f %.8f\n',s.maxExecutedSpeed_mps,s.maxExecutedAccel_mps2,s.maxExecutedJerk_mps3);
if isfield(s,'maxExecutedVerticalSpeed_mps')
    fprintf(f,'Reference Z v/a/j %.8f %.8f %.8f\n',s.maxVerticalReferenceSpeed_mps,s.maxVerticalReferenceAccel_mps2,s.maxVerticalReferenceJerk_mps3);
    fprintf(f,'Executed Z v/a/j %.8f %.8f %.8f\n',s.maxExecutedVerticalSpeed_mps,s.maxExecutedVerticalAccel_mps2,s.maxExecutedVerticalJerk_mps3);
end
fprintf(f,'Track %.8f | override %.8f | altitude %.8f | tilt %.8f deg\n',s.maxTrackingError_m,s.maxSafetyOverrideDeviation_m,s.maxAltitudeError_m,s.maxTilt_deg);
fprintf(f,'Clearance obstacle/wall/dynamic %.8f %.8f %.8f\n',s.minObstacleClearance_m,s.minWallClearance_m,s.minDynamicClearance_m);
if isfield(s,'finalState')
    fprintf(f,'Final state %s | distance goal %.8f\n',s.finalState,field_or(s,'finalDistanceToGoal_m',nan));
end
end
function v=field_or(s,name,default)
if isfield(s,name),v=s.(name);else,v=default;end
end
function o=ternary(c,a,b),if c,o=a;else,o=b;end,end

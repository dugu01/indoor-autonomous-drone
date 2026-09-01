function report = validate_S2_2(makePlots)
% VALIDATE_S2_2 Complete v0.4 regression + v0.5.3.3 lifecycle matrix.
if nargin<1,makePlots=false;end
cfg=init_S2_2_config();versionLabel=cfg.version;versionFolder=lower(regexprep(versionLabel,'[^A-Za-z0-9]+','_'));
scenarios={ ...
    'nominal_6dof','incremental_static_estimated','dynamic_crossing_6dof', ...
    'dynamic_blocker_becomes_static_6dof','obstacle_sensor_dropout_recover_6dof', ...
    'primary_imu_fault_vio_outage','xy_aid_loss_failsafe', ...
    'full_mission_nominal','rtl_obstacle_replan','alternate_landing_zone', ...
    'preflight_reject_unsafe_home','xy_loss_emergency_land'};
report=struct('scenario',{},'pass',{},'summary',{},'error',{});
for i=1:numel(scenarios)
    fprintf('\n[VALIDATE S2.2 %s] %s\n',versionLabel,scenarios{i});
    report(i).scenario=scenarios{i};report(i).pass=false;report(i).summary=[];report(i).error='';
    try
        r=run_S2_2_mission_replanning(0,scenarios{i},makePlots,false);
        report(i).pass=r.summary.pass;report(i).summary=r.summary;
        if ~r.summary.pass,report(i).error='Scenario completed but one or more validation gates failed.';end
    catch ME
        report(i).pass=false;report(i).error=sprintf('%s: %s',ME.identifier,ME.message);
        fprintf(2,'[VALIDATE] Runtime error in %s: %s\n',scenarios{i},report(i).error);
    end
end
scriptDir=fileparts(mfilename('fullpath'));
validationDir=fullfile(fileparts(scriptDir),'results','S2_2_mission_replanning',versionFolder,'validation');if ~exist(validationDir,'dir'),mkdir(validationDir);end
save(fullfile(validationDir,sprintf('validation_report_S2_2_%s.mat',versionFolder)),'report');
write_validation_text(report,fullfile(validationDir,sprintf('validation_report_S2_2_%s.txt',versionFolder)),versionLabel);
failed=find(~[report.pass]);
if isempty(failed)
    fprintf('\n[VALIDATE S2.2 %s] All %d scenarios passed.\n',versionLabel,numel(scenarios));
    fprintf('[VALIDATE S2.2 %s] Report saved: %s\n',versionLabel,validationDir);
else
    names=strjoin({report(failed).scenario},', ');
    fprintf(2,'\n[VALIDATE S2.2 %s] %d/%d scenarios failed: %s\n',versionLabel,numel(failed),numel(scenarios),names);
    fprintf('[VALIDATE S2.2 %s] Complete report saved: %s\n',versionLabel,validationDir);
    error('S2_2:ValidationFailed','S2.2 %s failed scenarios: %s',versionLabel,names);
end
end

function write_validation_text(report,filePath,versionLabel)
f=fopen(filePath,'w');if f<0,return;end;c=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'Stage S2.2 %s validation report\nGenerated: %s\n\n',versionLabel,datestr(now,31));
for i=1:numel(report)
    if isempty(report(i).summary)
        fprintf(f,'%s | PASS 0 | ERROR %s\n',report(i).scenario,report(i).error);continue;
    end
    s=report(i).summary;
    if isfield(s,'lifecycleEnabled')&&s.lifecycleEnabled
        pf=s.preflightCheck;
        fprintf(f,['%s | PASS %d | goal %d | fail %d | arm %d | takeoff %d | RTL %d | land %d | disarm %d | ' ...
            'emergency %d | alternate %d | complete %d | truth G/T/L %d/%d/%d | replans %d | ' ...
            'preflight H/V/A/L/C/U/home/goal/route %d/%d/%d/%d/%d/%d/%d/%d/%d | ' ...
            'est total/metric %.4f/%.4f | drift %.4f | clear %.4f | Zexec %.3f/%.3f/%.3f\n'], ...
            report(i).scenario,report(i).pass,s.goalReached,s.failsafeTriggered,s.armedEver,s.takeoffCompleted, ...
            s.rtlExecuted,s.landed,s.disarmed,s.emergencyLanding,s.alternateLandingUsed,s.missionComplete, ...
            s.truthGoalReached,s.truthTakeoffReached,s.truthLanded,s.replanCount, ...
            pf.horizontalAidsOK,pf.verticalAidOK,pf.attitudeAidOK,pf.laneOK,pf.covarianceOK, ...
            pf.updateCountOK,pf.homeClear,pf.goalCellClear,pf.goalReachable, ...
            s.maxEstimatorPositionError_m,s.estimatorFailsafeMetric_m, ...
            s.maxEmergencyHorizontalDrift_m,min(s.minObstacleClearance_m,s.minWallClearance_m), ...
            s.maxExecutedVerticalSpeed_mps,s.maxExecutedVerticalAccel_mps2,s.maxExecutedVerticalJerk_mps3);
    else
        fprintf(f,['%s | PASS %d | goal %d | failsafe %d | RTL %d | replans %d | lane %d/%d | ' ...
            'est %.4f/%.3fdeg | track %.4f | clear %.4f\n'], ...
            report(i).scenario,report(i).pass,s.goalReached,s.failsafeTriggered,s.rtlRequested,s.replanCount, ...
            s.activeLaneFinal,s.laneSwitches,s.maxEstimatorPositionError_m,s.maxEstimatorAttitudeError_deg, ...
            s.maxTrackingError_m,min(s.minObstacleClearance_m,s.minWallClearance_m));
    end
end
fprintf(f,'\nOverall: %d/%d PASS\n',sum([report.pass]),numel(report));
end

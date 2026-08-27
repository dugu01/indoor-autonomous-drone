function report = validate_S2_2(makePlots)
% VALIDATE_S2_2 Complete S2.2 v0.4 estimator/6-DOF validation matrix.
if nargin<1,makePlots=false;end
scenarios={'nominal_6dof','incremental_static_estimated','dynamic_crossing_6dof', ...
    'dynamic_blocker_becomes_static_6dof','obstacle_sensor_dropout_recover_6dof', ...
    'primary_imu_fault_vio_outage','xy_aid_loss_failsafe'};
report=struct('scenario',{},'pass',{},'summary',{},'error',{});
for i=1:numel(scenarios)
    fprintf('\n[VALIDATE S2.2 v0.4] %s\n',scenarios{i});report(i).scenario=scenarios{i};report(i).pass=false;report(i).summary=[];report(i).error='';
    try
        r=run_S2_2_mission_replanning(0,scenarios{i},makePlots,false);report(i).pass=r.summary.pass;report(i).summary=r.summary;
        if ~r.summary.pass,report(i).error='Scenario completed but one or more validation gates failed.';end
    catch ME
        report(i).pass=false;report(i).error=sprintf('%s: %s',ME.identifier,ME.message);fprintf(2,'[VALIDATE] Runtime error in %s: %s\n',scenarios{i},report(i).error);
    end
end
scriptDir=fileparts(mfilename('fullpath'));cfg=init_S2_2_config();versionFolder=lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_'));
validationDir=fullfile(fileparts(scriptDir),'results','S2_2_mission_replanning',versionFolder,'validation');if ~exist(validationDir,'dir'),mkdir(validationDir);end
save(fullfile(validationDir,'validation_report_S2_2_v0_4.mat'),'report');write_validation_text(report,fullfile(validationDir,'validation_report_S2_2_v0_4.txt'));
failed=find(~[report.pass]);
if isempty(failed)
    fprintf('\n[VALIDATE S2.2 v0.4] All %d scenarios passed.\n',numel(scenarios));fprintf('[VALIDATE S2.2 v0.4] Report saved: %s\n',validationDir);
else
    names=strjoin({report(failed).scenario},', ');fprintf(2,'\n[VALIDATE S2.2 v0.4] %d/%d scenarios failed: %s\n',numel(failed),numel(scenarios),names);
    fprintf('[VALIDATE S2.2 v0.4] Complete report saved: %s\n',validationDir);error('S2_2:ValidationFailed','S2.2 v0.4 failed scenarios: %s',names);
end
end
function write_validation_text(report,filePath)
f=fopen(filePath,'w');if f<0,return;end;c=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'Stage S2.2 v0.4 validation report\nGenerated: %s\n\n',datestr(now,31));
for i=1:numel(report)
    if isempty(report(i).summary),fprintf(f,'%s | PASS 0 | ERROR %s\n',report(i).scenario,report(i).error);continue;end
    s=report(i).summary;fprintf(f,['%s | PASS %d | goal %d | failsafe %d | RTL %d | replans %d | lane %d/%d | ' ...
        'est %.4f/%.3fdeg | track %.4f | clear %.4f | core %d%d%d%d%d%d%d | events %d%d%d%d%d%d\n'], ...
        report(i).scenario,report(i).pass,s.goalReached,s.failsafeTriggered,s.rtlRequested,s.replanCount,s.activeLaneFinal,s.laneSwitches, ...
        s.maxEstimatorPositionError_m,s.maxEstimatorAttitudeError_deg,s.maxTrackingError_m,min(s.minObstacleClearance_m,s.minWallClearance_m), ...
        s.staticPass,s.dynamicPass,s.referenceKinematicPass,s.executedKinematicPass,s.controllerPass, ...
        s.estimatorPositionPass&&s.estimatorAttitudePass,s.uncertaintyPass,s.replanEventPass,s.dynamicEventPass, ...
        s.promotionEventPass,s.noDataEventPass,s.laneSwitchEventPass,s.rtlEventPass);
end
fprintf(f,'\nOverall: %d/%d PASS\n',sum([report.pass]),numel(report));
end

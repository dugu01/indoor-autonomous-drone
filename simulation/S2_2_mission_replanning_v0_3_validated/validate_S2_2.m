function report = validate_S2_2(makePlots)
% VALIDATE_S2_2 Run the complete S2.2 v0.3 matrix and report all failures.
% This validator completes every scenario before raising one consolidated
% error, so one run reveals the entire remaining failure set.
if nargin<1,makePlots=false;end
scenarios={'incremental_static_insert','dynamic_crossing_yield','dynamic_blocker_becomes_static', ...
    'sensor_dropout_recover','sensor_dropout_failsafe','two_dynamic_crossings','trajectory_time_rescale'};
report=struct('scenario',{},'pass',{},'summary',{},'error',{});
for i=1:numel(scenarios)
    fprintf('\n[VALIDATE S2.2 v0.3] %s\n',scenarios{i});
    report(i).scenario=scenarios{i};report(i).pass=false;report(i).summary=[];report(i).error='';
    try
        r=run_S2_2_mission_replanning(0,scenarios{i},makePlots,false);
        report(i).pass=r.summary.pass;report(i).summary=r.summary;
        if ~r.summary.pass
            report(i).error='Scenario completed but one or more validation gates failed.';
        end
    catch ME
        report(i).pass=false;report(i).error=sprintf('%s: %s',ME.identifier,ME.message);
        fprintf(2,'[VALIDATE] Runtime error in %s: %s\n',scenarios{i},report(i).error);
    end
end
scriptDir=fileparts(mfilename('fullpath'));cfg=init_S2_2_config();
versionFolder=lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_'));
validationDir=fullfile(fileparts(scriptDir),'results','S2_2_mission_replanning',versionFolder,'validation');
if ~exist(validationDir,'dir'),mkdir(validationDir);end
save(fullfile(validationDir,'validation_report_S2_2_v0_3.mat'),'report');
write_validation_text(report,fullfile(validationDir,'validation_report_S2_2_v0_3.txt'));
failed=find(~[report.pass]);
if isempty(failed)
    fprintf('\n[VALIDATE S2.2 v0.3] All %d scenarios passed.\n',numel(scenarios));
    fprintf('[VALIDATE S2.2 v0.3] Report saved: %s\n',validationDir);
else
    names=strjoin({report(failed).scenario},', ');
    fprintf(2,'\n[VALIDATE S2.2 v0.3] %d/%d scenarios failed: %s\n',numel(failed),numel(scenarios),names);
    fprintf('[VALIDATE S2.2 v0.3] Complete report saved: %s\n',validationDir);
    error('S2_2:ValidationFailed','S2.2 v0.3 failed scenarios: %s',names);
end
end

function write_validation_text(report,filePath)
f=fopen(filePath,'w');if f<0,return;end;c=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'Stage S2.2 v0.3 validation report\nGenerated: %s\n\n',datestr(now,31));
for i=1:numel(report)
    if isempty(report(i).summary)
        fprintf(f,'%s | PASS 0 | ERROR %s\n',report(i).scenario,report(i).error);
    else
        s=report(i).summary;
        fprintf(f,['%s | PASS %d | goal %d | failsafe %d | replan %d | dynamic %d | ' ...
            'vmax %.4f | amax %.4f | jmax %.4f | track %.4f | core %d%d%d%d%d%d | ' ...
            'mission %d%d%d | events %d%d%d%d%d%d\n'], ...
            report(i).scenario,report(i).pass,s.goalReached,s.failsafeTriggered,s.replanCount, ...
            s.dynamicAvoidSteps,s.maxSpeed_mps,s.maxAccel_mps2,s.maxJerk_mps3,s.maxTrackingError_m, ...
            s.staticPass,s.dynamicPass,s.kinematicPass,s.referenceContinuityPass, ...
            s.replanContinuityPass,s.trackingPass,s.missionOutcomePass, ...
            s.failsafeExpectationPass,s.eventPass,s.incrementalEventPass,s.dynamicEventPass, ...
            s.noDataEventPass,s.promotionEventPass,s.timeRescalePass,s.searchEfficiencyPass);
    end
end
fprintf(f,'\nOverall: %d/%d PASS\n',sum([report.pass]),numel(report));
end

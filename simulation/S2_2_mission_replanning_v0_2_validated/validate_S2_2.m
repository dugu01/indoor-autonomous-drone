function report = validate_S2_2(makePlots)
% VALIDATE_S2_2  Run Stage S2.2 v0.2 scenario matrix.
% With makePlots=true, each scenario opens as a tab in one docked Figures window.
if nargin<1,makePlots=false;end
scenarios={'incremental_static_insert','dynamic_crossing_yield', ...
    'dynamic_blocker_becomes_static','sensor_dropout_recover', ...
    'sensor_dropout_failsafe','two_dynamic_crossings'};
report=struct('scenario',{},'pass',{},'summary',{});
for i=1:numel(scenarios)
    fprintf('\n[VALIDATE S2.2 v0.2] %s\n',scenarios{i});
    r=run_S2_2_mission_replanning(0,scenarios{i},makePlots,false);
    report(i).scenario=scenarios{i};report(i).pass=r.summary.pass;report(i).summary=r.summary;
    assert(r.summary.pass,'S2.2 v0.2 scenario failed: %s',scenarios{i});
end

% Save an aggregate validation record inside the same versioned result tree.
scriptDir=fileparts(mfilename('fullpath'));
cfg=init_S2_2_config();
versionFolder=lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_'));
validationDir=fullfile(fileparts(scriptDir),'results','S2_2_mission_replanning',versionFolder,'validation');
if ~exist(validationDir,'dir'),mkdir(validationDir);end
save(fullfile(validationDir,'validation_report_S2_2_v0_2.mat'),'report');
write_validation_text(report,fullfile(validationDir,'validation_report_S2_2_v0_2.txt'));

fprintf('\n[VALIDATE S2.2 v0.2] All %d scenarios passed.\n',numel(scenarios));
fprintf('[VALIDATE S2.2 v0.2] Report saved: %s\n',validationDir);
end

function write_validation_text(report,filePath)
f=fopen(filePath,'w');if f<0,return;end;c=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'Stage S2.2 v0.2 validation report\n');
fprintf(f,'Generated: %s\n\n',datestr(now,31));
for i=1:numel(report)
    s=report(i).summary;
    fprintf(f,'%s | PASS %d | goal %d | failsafe %d | replans %d | promotions %d | collisions %d | geofence %d\n', ...
        report(i).scenario,report(i).pass,s.goalReached,s.failsafeTriggered, ...
        s.replanCount,s.promotionCount,s.collisionCount,s.geofenceViolationCount);
end
fprintf(f,'\nOverall: %d/%d PASS\n',sum([report.pass]),numel(report));
end

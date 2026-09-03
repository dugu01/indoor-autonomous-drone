function report = validate_S2_2_v0_5_3_focus()
% VALIDATE_S2_2_V0_5_3_FOCUS
% Re-runs the exact twelve seed/scenario combinations that failed the
% Stage S2.2 v0.5.2 multi-seed sweep. Every run is completed and recorded;
% an error is raised only after the full focused matrix finishes.

cases={ ...
    'dynamic_crossing_6dof',                 1; ...
    'dynamic_crossing_6dof',                 7; ...
    'primary_imu_fault_vio_outage',          1; ...
    'primary_imu_fault_vio_outage',          2; ...
    'primary_imu_fault_vio_outage',          6; ...
    'primary_imu_fault_vio_outage',          9; ...
    'rtl_obstacle_replan',                   2; ...
    'xy_loss_emergency_land',                2; ...
    'xy_loss_emergency_land',                3; ...
    'xy_loss_emergency_land',                6; ...
    'xy_loss_emergency_land',                7; ...
    'xy_loss_emergency_land',                9};

cfg=init_S2_2_config();
scriptDir=fileparts(mfilename('fullpath'));
versionFolder=lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_'));
validationDir=fullfile(fileparts(scriptDir),'results','S2_2_mission_replanning', ...
    versionFolder,'validation');
if ~exist(validationDir,'dir'),mkdir(validationDir);end

n=size(cases,1);passed=false(n,1);errors=cell(n,1);summaries=cell(n,1);
for i=1:n
    scenario=cases{i,1};seed=cases{i,2};
    fprintf('\n[FOCUSED %02d/%02d] %s seed %d\n',i,n,upper(scenario),seed);
    try
        r=run_S2_2_mission_replanning(seed,scenario,false,false);
        passed(i)=logical(r.summary.pass);summaries{i}=r.summary;
        if ~passed(i),errors{i}='summary.pass was false';end
    catch ME
        passed(i)=false;errors{i}=sprintf('%s: %s',ME.identifier,ME.message);
    end
end

report=struct('version',cfg.version,'cases',{cases},'passed',passed, ...
    'errors',{errors},'summaries',{summaries},'totalPass',nnz(passed), ...
    'totalRuns',n,'allPassed',all(passed));
save(fullfile(validationDir,sprintf('focused_failed_seeds_%s.mat',versionFolder)), ...
    'report','-v7.3');

textPath=fullfile(validationDir,sprintf('focused_failed_seeds_%s.txt',versionFolder));
f=fopen(textPath,'w');
if f>=0
    c=onCleanup(@()fclose(f)); %#ok<NASGU>
    fprintf(f,'Stage S2.2 %s focused former-failure regression\n',cfg.version);
    fprintf(f,'Total %d/%d PASS\n\n',report.totalPass,report.totalRuns);
    for i=1:n
        fprintf(f,'%s seed %d: %s',cases{i,1},cases{i,2}, ...
            ternary(passed(i),'PASS','FAIL'));
        if ~passed(i),fprintf(f,' — %s',errors{i});end
        fprintf(f,'\n');
    end
end

fprintf('\n============================================================\n');
fprintf(' S2.2 %s FOCUSED FORMER FAILURES: %d/%d PASS\n', ...
    cfg.version,report.totalPass,report.totalRuns);
fprintf(' Report: %s\n',validationDir);
fprintf('============================================================\n');

if ~report.allPassed
    failed={};
    for i=1:n
        if ~passed(i)
            failed{end+1}=sprintf('%s seed %d',cases{i,1},cases{i,2}); %#ok<AGROW>
        end
    end
    error('S2_2:FocusedFailure','Focused failures: %s',strjoin(failed,'; '));
end
end

function out=ternary(condition,a,b)
if condition,out=a;else,out=b;end
end

function report = validate_S2_2_multiseed_robustness(seeds)
% VALIDATE_S2_2_MULTISEED_ROBUSTNESS
% Runs the six highest-risk S2.2 v0.5.3 scenarios over a deterministic seed
% set. The function completes every run, records failures, and saves a
% versioned MATLAB and text report. It never hides a failed seed by averaging.

if nargin<1||isempty(seeds),seeds=0:9;end
validateattributes(seeds,{'numeric'},{'vector','integer','nonnegative','finite'});
seeds=double(seeds(:).');

scenarios={ ...
    'dynamic_crossing_6dof', ...
    'dynamic_blocker_becomes_static_6dof', ...
    'primary_imu_fault_vio_outage', ...
    'rtl_obstacle_replan', ...
    'alternate_landing_zone', ...
    'xy_loss_emergency_land'};

cfg=init_S2_2_config();
scriptDir=fileparts(mfilename('fullpath'));
versionFolder=lower(regexprep(cfg.version,'[^A-Za-z0-9]+','_'));
validationDir=fullfile(fileparts(scriptDir),'results','S2_2_mission_replanning', ...
    versionFolder,'validation');
if ~exist(validationDir,'dir'),mkdir(validationDir);end

nScenario=numel(scenarios);nSeed=numel(seeds);
passed=false(nScenario,nSeed);errors=cell(nScenario,nSeed);summaries=cell(nScenario,nSeed);

for i=1:nScenario
    name=scenarios{i};
    for j=1:nSeed
        seed=seeds(j);
        fprintf('\n[MULTISEED %s] seed %d\n',upper(name),seed);
        try
            r=run_S2_2_mission_replanning(seed,name,false,false);
            passed(i,j)=logical(r.summary.pass);
            summaries{i,j}=r.summary;
            if ~passed(i,j),errors{i,j}='summary.pass was false';end
        catch ME
            passed(i,j)=false;errors{i,j}=sprintf('%s: %s',ME.identifier,ME.message);
        end
    end
    fprintf('\n%s: %d/%d PASS\n',name,nnz(passed(i,:)),nSeed);
end

scenarioPassCounts=sum(passed,2);
totalPass=nnz(passed);totalRuns=numel(passed);
report=struct('version',cfg.version,'scenarios',{scenarios},'seeds',seeds, ...
    'passed',passed,'scenarioPassCounts',scenarioPassCounts, ...
    'totalPass',totalPass,'totalRuns',totalRuns,'allPassed',all(passed(:)), ...
    'errors',{errors},'summaries',{summaries});

save(fullfile(validationDir,sprintf('multiseed_robustness_%s.mat',versionFolder)), ...
    'report','-v7.3');
write_text_report(report,validationDir,versionFolder);

fprintf('\n============================================================\n');
fprintf(' S2.2 %s MULTI-SEED ROBUSTNESS: %d/%d PASS\n', ...
    cfg.version,totalPass,totalRuns);
fprintf(' Report: %s\n',validationDir);
fprintf('============================================================\n');

if ~report.allPassed
    failed={};
    for i=1:nScenario
        badSeeds=seeds(~passed(i,:));
        if ~isempty(badSeeds)
            failed{end+1}=sprintf('%s seeds [%s]',scenarios{i},num2str(badSeeds)); %#ok<AGROW>
        end
    end
    error('S2_2:MultiSeedFailure','Multi-seed failures: %s',strjoin(failed,'; '));
end
end

function write_text_report(report,validationDir,versionFolder)
path=fullfile(validationDir,sprintf('multiseed_robustness_%s.txt',versionFolder));
f=fopen(path,'w');if f<0,return;end
c=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'Stage S2.2 %s multi-seed robustness\n',report.version);
fprintf(f,'Total %d/%d PASS\n\n',report.totalPass,report.totalRuns);
for i=1:numel(report.scenarios)
    fprintf(f,'%s: %d/%d PASS\n',report.scenarios{i}, ...
        report.scenarioPassCounts(i),numel(report.seeds));
    for j=1:numel(report.seeds)
        if ~report.passed(i,j)
            fprintf(f,'  seed %d FAIL: %s\n',report.seeds(j),report.errors{i,j});
        end
    end
end
end

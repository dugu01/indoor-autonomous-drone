function report = validate_S2_2_monte_carlo(seeds,makePlots)
% VALIDATE_S2_2_MONTE_CARLO Multi-seed lifecycle regression.
if nargin<1||isempty(seeds),seeds=0:4;end
if nargin<2,makePlots=false;end
cfg=init_S2_2_config();versionLabel=cfg.version;versionFolder=lower(regexprep(versionLabel,'[^A-Za-z0-9]+','_'));
scenarios={'full_mission_nominal','rtl_obstacle_replan','alternate_landing_zone', ...
    'preflight_reject_unsafe_home','xy_loss_emergency_land'};
report=struct('scenario',{},'seed',{},'pass',{},'summary',{},'error',{});n=0;
for i=1:numel(scenarios)
    for j=1:numel(seeds)
        n=n+1;report(n).scenario=scenarios{i};report(n).seed=seeds(j);report(n).pass=false;report(n).summary=[];report(n).error='';
        fprintf('\n[MONTE CARLO S2.2 %s] %s seed %d\n',versionLabel,scenarios{i},seeds(j));
        try
            r=run_S2_2_mission_replanning(seeds(j),scenarios{i},makePlots,false);
            report(n).pass=r.summary.pass;report(n).summary=r.summary;
        catch ME
            report(n).error=sprintf('%s: %s',ME.identifier,ME.message);
        end
    end
end
scriptDir=fileparts(mfilename('fullpath'));
outDir=fullfile(fileparts(scriptDir),'results','S2_2_mission_replanning',versionFolder,'monte_carlo');if ~exist(outDir,'dir'),mkdir(outDir);end
save(fullfile(outDir,sprintf('monte_carlo_report_S2_2_%s.mat',versionFolder)),'report','seeds','scenarios');
failed=find(~[report.pass]);
if isempty(failed)
    fprintf('\n[MONTE CARLO S2.2 %s] All %d runs passed.\n',versionLabel,numel(report));
else
    error('S2_2:MonteCarloFailed','%d/%d Monte Carlo runs failed.',numel(failed),numel(report));
end
end

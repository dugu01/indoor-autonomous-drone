function report = validate_S2_3_release_focus(makePlots)
% VALIDATE_S2_3_RELEASE_FOCUS Re-run only the five release-closing cases.
if nargin<1,makePlots=false;end
scenarios={'hidden_obstacle_replan','occluded_obstacle', ...
    'unknown_narrow_passage','goal_requires_scan','unreachable_goal'};
report=struct('scenario',{},'pass',{},'summary',{},'error',{});
for i=1:numel(scenarios)
    fprintf('\n[S2.3 RELEASE FOCUS %02d/%02d] %s\n',i,numel(scenarios),scenarios{i});
    report(i).scenario=scenarios{i};report(i).pass=false;report(i).summary=[];report(i).error='';
    try
        r=run_S2_3_online_mapping(0,scenarios{i},makePlots,false);
        report(i).summary=r.summary;report(i).pass=r.summary.pass;
    catch ME
        report(i).error=getReport(ME,'extended','hyperlinks','off');
        fprintf(2,'%s\n',report(i).error);
    end
end
cfg=init_S2_3_config();d=fullfile(cfg.resultsRoot,'validation');if ~exist(d,'dir'),mkdir(d);end
save(fullfile(d,'release_focus_report_S2_3_candidate.mat'),'report');
nPass=sum([report.pass]);
fprintf('\nS2.3 RELEASE FOCUS: %d / %d PASS\n',nPass,numel(report));
if ~all([report.pass]),error('S2_3:ReleaseFocusFailed','S2.3 release focus failed.');end
end

function report = validate_S2_3(makePlots)
% VALIDATE_S2_3 Deterministic full-stage scenario catalogue.
if nargin<1,makePlots=false;end
scenarios={'unknown_room_nominal','hidden_obstacle_replan','occluded_obstacle', ...
    'unknown_narrow_passage','dead_end_recovery','goal_requires_scan', ...
    'unreachable_goal','depth_dropout_lidar','lidar_dropout_depth', ...
    'perception_dropout_recover','primary_imu_fault_mapping','dynamic_to_static_mapping'};
report=struct('scenario',{},'pass',{},'summary',{},'error',{});
for i=1:numel(scenarios)
    fprintf('\n[VALIDATE S2.3 %02d/%02d] %s\n',i,numel(scenarios),scenarios{i});
    report(i).scenario=scenarios{i};report(i).pass=false;report(i).summary=[];report(i).error='';
    try
        r=run_S2_3_online_mapping(0,scenarios{i},makePlots,false);
        report(i).summary=r.summary;report(i).pass=r.summary.pass;
    catch ME
        report(i).error=getReport(ME,'extended','hyperlinks','off');
        fprintf(2,'%s\n',report(i).error);
    end
end
cfg=init_S2_3_config();validationDir=fullfile(cfg.resultsRoot,'validation');if ~exist(validationDir,'dir'),mkdir(validationDir);end
save(fullfile(validationDir,'validation_report_S2_3_candidate.mat'),'report');
if ~all([report.pass]),error('S2_3:ValidationFailed','S2.3 deterministic validation failed.');end
end

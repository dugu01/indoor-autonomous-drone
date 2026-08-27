function report = validate_S2_3_multiseed(seeds)
% VALIDATE_S2_3_MULTISEED Six critical scenarios x ten deterministic seeds.
if nargin<1||isempty(seeds),seeds=0:9;end
scenarios={'hidden_obstacle_replan','dead_end_recovery','primary_imu_fault_mapping', ...
    'depth_dropout_lidar','perception_dropout_recover','dynamic_to_static_mapping'};
passed=false(numel(scenarios),numel(seeds));summaries=cell(size(passed));errors=cell(size(passed));
for i=1:numel(scenarios)
 for j=1:numel(seeds)
  fprintf('\n[S2.3 ROBUST %s seed %d]\n',scenarios{i},seeds(j));
  try,r=run_S2_3_online_mapping(seeds(j),scenarios{i},false,false);passed(i,j)=r.summary.pass;summaries{i,j}=r.summary;
  catch ME,errors{i,j}=getReport(ME,'extended','hyperlinks','off');end
 end
end
report=struct('scenarios',{scenarios},'seeds',seeds,'passed',passed,'summaries',{summaries},'errors',{errors}, ...
    'passCount',nnz(passed),'trialCount',numel(passed),'pass',all(passed(:)));
cfg=init_S2_3_config();d=fullfile(cfg.resultsRoot,'validation');if ~exist(d,'dir'),mkdir(d);end
save(fullfile(d,'multiseed_robustness_S2_3_candidate.mat'),'report','-v7.3');
if ~report.pass,error('S2_3:RobustnessFailed','S2.3 robustness %d/%d PASS.',report.passCount,report.trialCount);end
end

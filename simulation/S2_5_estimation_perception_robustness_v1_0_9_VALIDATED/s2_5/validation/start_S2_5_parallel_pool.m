function info = start_S2_5_parallel_pool()
% START_S2_5_PARALLEL_POOL Require/use a bounded local MATLAB worker pool.
% Default is 4 process workers for memory-heavy coupled missions. Set
% S2_5_WORKERS=2..8 to override. If an existing pool has a different worker
% count from an explicit override, it is restarted so the request is honored.
if ~license('test','Distrib_Computing_Toolbox') || isempty(ver('parallel'))
    error('S2_5:ParallelUnavailable', ...
        ['Parallel Computing Toolbox is required by this parallel qualification harness. ' ...
         'Install/enable it or use the original serial validator.']);
end
requested=str2double(getenv('S2_5_WORKERS'));
explicit=isfinite(requested)&&requested>=2;
if explicit,requested=min(8,max(2,floor(requested)));else,requested=NaN;end
try
    c=parcluster('Processes');
catch
    c=parcluster('local'); % compatibility with older MATLAB releases
end
if explicit,n=min(requested,c.NumWorkers);else,n=min(4,c.NumWorkers);end
if n<2,error('S2_5:InsufficientWorkers','At least 2 local workers are required; cluster reports %d.',c.NumWorkers);end
p=gcp('nocreate');
if ~isempty(p) && p.NumWorkers~=n
    fprintf('S2.5 PARALLEL POOL: restarting existing %d-worker pool as %d workers.\n',p.NumWorkers,n);
    delete(p);p=[];
end
if isempty(p),p=parpool(c,n);end
if n>4
    fprintf(['S2.5 PARALLEL WARNING: %d process workers requested. Full coupled missions are memory-heavy; ' ...
        'if wall time regresses, use S2_5_WORKERS=4.\n'],n);
end
info=struct('workers',p.NumWorkers,'profile',p.Cluster.Profile,'pool',p);
fprintf('S2.5 PARALLEL POOL: PASS | workers=%d | profile=%s\n',p.NumWorkers,p.Cluster.Profile);
end

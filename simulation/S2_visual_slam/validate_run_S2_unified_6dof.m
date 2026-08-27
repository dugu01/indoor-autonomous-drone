function validation = validate_run_S2_unified_6dof(runStress)
%VALIDATE_RUN_S2_UNIFIED_6DOF Static and runtime checks for the fixed file.
%
%   validation = validate_run_S2_unified_6dof(false)
%   validation = validate_run_S2_unified_6dof(true)
%
% Put this file and run_S2_unified_6dof.m in the same folder before running.

if nargin < 1 || isempty(runStress)
    runStress = false;
end
runStress = logical(runStress);

thisFile = mfilename('fullpath');
thisDir = fileparts(thisFile);
addpath(thisDir);
rehash;

fprintf('\n=== MATLAB PATH CHECK ===\n');
allMatches = which('run_S2_unified_6dof','-all');
disp(allMatches);
resolved = which('run_S2_unified_6dof');
expected = fullfile(thisDir,'run_S2_unified_6dof.m');
if ~strcmpi(resolved,expected)
    error('S2:PathConflict', ...
        'MATLAB resolves %s instead of %s. Remove or reorder duplicate paths.', ...
        resolved,expected);
end

fprintf('\n=== CHECKCODE ===\n');
issues = checkcode(expected,'-id');
if isempty(issues)
    fprintf('No Code Analyzer issues reported.\n');
else
    for k = 1:numel(issues)
        fprintf('Line %d, column %d, %s: %s\n', ...
            issues(k).line,issues(k).column,issues(k).id,issues(k).message);
    end
end

fprintf('\n=== NOMINAL RUNTIME CHECK ===\n');
results = run_S2_unified_6dof(0,runStress,false,false);

assert(isfield(results,'nominal'),'Missing results.nominal.');
assert(isfield(results.nominal,'summary'),'Missing nominal summary.');
assert(isfield(results.nominal,'animationFile'),'Missing animationFile field.');
assert(all(isfinite(results.nominal.est.p),'all'),'Non-finite nominal position state.');
assert(all(isfinite(results.nominal.est.q),'all'),'Non-finite nominal attitude state.');
assert(all(vecnorm(results.nominal.est.q,2,2) > 0.999 & ...
           vecnorm(results.nominal.est.q,2,2) < 1.001), ...
       'Quaternion norm left the expected unit interval.');

s = results.nominal.summary;
fprintf('\nNominal max after 5 s : %.3f cm\n',100*s.fusedMaxAfter5_m);
fprintf('Nominal RMSE after 5 s: %.3f cm\n',100*s.fusedRMSEAfter5_m);
fprintf('Attitude max          : %.3f deg\n',s.attMaxAfter5_deg);
fprintf('Lane switches         : %d\n',s.counts.laneSwitches);
fprintf('LiDAR accepted        : %.2f %%\n',100*s.lidarHealthyFraction);
fprintf('Nominal pass flag     : %d\n',s.pass);

validation = struct();
validation.resolvedFile = resolved;
validation.codeAnalyzerIssues = issues;
validation.results = results;
validation.nominalPass = s.pass;
validation.positionRequirementMet = s.fusedMaxAfter5_m < 0.10;
validation.attitudeRequirementMet = s.attMaxAfter5_deg < 2.0;

if ~validation.positionRequirementMet
    warning('S2:PositionRequirement', ...
        'Nominal maximum error %.3f m exceeds the 0.10 m requirement.', ...
        s.fusedMaxAfter5_m);
end
if ~validation.attitudeRequirementMet
    warning('S2:AttitudeRequirement', ...
        'Nominal maximum attitude error %.3f deg exceeds 2 deg.', ...
        s.attMaxAfter5_deg);
end
end

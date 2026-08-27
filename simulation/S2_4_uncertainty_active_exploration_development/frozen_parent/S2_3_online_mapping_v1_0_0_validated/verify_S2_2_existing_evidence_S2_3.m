function evidence = verify_S2_2_existing_evidence_S2_3()
% VERIFY_S2_2_EXISTING_EVIDENCE_S2_3 Reuse completed frozen S2.2 evidence.
% This is valid only because audit_S2_3_candidate.py verifies that every
% inherited S2.2 source file remains byte-identical to the frozen manifest.
scriptDir=fileparts(mfilename('fullpath'));
simulationDir=fileparts(scriptDir);
validationDir=fullfile(simulationDir,'results','S2_2_mission_replanning', ...
    'v0_5_3_3','validation');
paths=struct( ...
    'deterministic',fullfile(validationDir,'validation_report_S2_2_v0_5_3_3.mat'), ...
    'focused',fullfile(validationDir,'focused_failed_seeds_v0_5_3_3.mat'), ...
    'robustness',fullfile(validationDir,'multiseed_robustness_v0_5_3_3.mat'));

cmd=sprintf('python3 "%s"',fullfile(scriptDir,'audit_S2_3_candidate.py'));
[status,auditText]=system(cmd);
sourceAuditPass=status==0&&contains(auditText,'inherited_files_unchanged             : PASS');

filesPresent=exist(paths.deterministic,'file')==2&&exist(paths.focused,'file')==2&& ...
    exist(paths.robustness,'file')==2;
deterministicPass=false;focusedPass=false;robustnessPass=false;
if filesPresent
    a=load(paths.deterministic,'report');
    b=load(paths.focused,'report');
    c=load(paths.robustness,'report');
    deterministicPass=isfield(a,'report')&&numel(a.report)==12&&all([a.report.pass]);
    focusedPass=isfield(b,'report')&&isfield(b.report,'allPassed')&&logical(b.report.allPassed);
    robustnessPass=isfield(c,'report')&&isfield(c.report,'allPassed')&&logical(c.report.allPassed)&& ...
        isfield(c.report,'totalPass')&&c.report.totalPass==60;
end
pass=sourceAuditPass&&filesPresent&&deterministicPass&&focusedPass&&robustnessPass;
evidence=struct('pass',pass,'sourceAuditPass',sourceAuditPass,'filesPresent',filesPresent, ...
    'deterministicPass',deterministicPass,'focusedPass',focusedPass, ...
    'robustnessPass',robustnessPass,'paths',paths,'auditText',auditText);

fprintf('\n============================================================\n');
fprintf(' FROZEN S2.2 EXISTING-EVIDENCE VERIFICATION\n');
fprintf(' Source byte identity     : %d\n',sourceAuditPass);
fprintf(' Deterministic 12/12      : %d\n',deterministicPass);
fprintf(' Focused 12/12            : %d\n',focusedPass);
fprintf(' Robustness 60/60         : %d\n',robustnessPass);
fprintf(' EVIDENCE VERIFIED        : %d\n',pass);
fprintf('============================================================\n');
if ~pass,error('S2_3:LegacyEvidenceInvalid','Frozen S2.2 evidence could not be verified.');end
end

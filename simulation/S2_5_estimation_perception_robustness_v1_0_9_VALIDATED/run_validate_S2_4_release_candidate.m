function report = run_validate_S2_4_release_candidate()
%RUN_VALIDATE_S2_4_RELEASE_CANDIDATE Final MATLAB-only release-candidate check.
projectRoot = fileparts(mfilename('fullpath'));
cd(projectRoot);
% Keep the frozen parent byte-identical even if nested MATLAB validation
% invokes Python helpers.
setenv('PYTHONDONTWRITEBYTECODE','1');

fprintf('\n============================================================\n');
fprintf(' S2.4 v1.0.0 CLEAN RELEASE-CANDIDATE FINAL VERIFICATION\n');
fprintf(' Root: %s\n', projectRoot);
fprintf('============================================================\n\n');

restoredefaultpath;
setup_S2_4_E_path(projectRoot);
clear functions;
rehash;

requestReport = test_S2_4_E_request_contracts();
assert(requestReport.pass,'S2.4-E request contracts failed.');

policy = test_S2_4_E_competing_decision_contract();
assert(policy.pass,'Controlled adversarial policy contract failed.');

geometry = test_S2_4_E_literal_corridor_geometry_contract();
assert(geometry.pass,'Literal corridor geometry contract failed.');

milestone1 = validate_S2_4_E_milestone_1();
assert(milestone1.pass,'S2.4-E Milestone 1 failed.');

milestone2 = validate_S2_4_E_milestone_2();
assert(milestone2.pass,'S2.4-E layered Milestone 2 failed.');

multi = validate_S2_4_E_competing_corridors_multiseed(0:9);
assert(multi.pass,'S2.4-E layered 10-seed validation failed.');

referenceParity = multi.cleanDecoyActivationCount == 4 && ...
    isequal(reshape(double(multi.cleanDecoyActivationSeeds),1,[]), [0 3 4 7]) && ...
    multi.adversarialPolicyContractPass && ...
    multi.literalGeometryPass && ...
    multi.benchmarkActivationPass && ...
    multi.allSeedDecisionPass && ...
    multi.conditionalPriorityPass && ...
    multi.hardSafetyPass && ...
    multi.missionCompletionPass;

report = struct();
report.requestContracts = requestReport.pass;
report.policyContract = policy.pass;
report.geometryContract = geometry.pass;
report.milestone1 = milestone1.pass;
report.milestone2 = milestone2.pass;
report.multiseed = multi.pass;
report.referenceParity = referenceParity;
report.pass = all(struct2array(report));

outDir = fullfile(projectRoot,'results','S2_4_release_candidate_verification');
if ~exist(outDir,'dir'), mkdir(outDir); end
save(fullfile(outDir,'S2_4_v1_0_0_CLEAN_FINAL_VERIFICATION.mat'),'report','multi','policy','geometry','milestone1','milestone2','requestReport');

fid=fopen(fullfile(outDir,'S2_4_v1_0_0_CLEAN_FINAL_VERIFICATION.txt'),'w');
fprintf(fid,'S2.4 v1.0.0 CLEAN RELEASE-CANDIDATE FINAL VERIFICATION\n');
fprintf(fid,'request contracts: %d\n',report.requestContracts);
fprintf(fid,'policy contract: %d\n',report.policyContract);
fprintf(fid,'geometry contract: %d\n',report.geometryContract);
fprintf(fid,'milestone 1: %d\n',report.milestone1);
fprintf(fid,'milestone 2: %d\n',report.milestone2);
fprintf(fid,'multiseed 0:9: %d\n',report.multiseed);
fprintf(fid,'reference parity: %d\n',report.referenceParity);
fprintf(fid,'PASS: %d\n',report.pass);
fclose(fid);

fprintf('\nS2.4 v1.0.0 CLEAN FINAL VERIFICATION: %s\n', passText(report.pass));
fprintf('reference parity: %s\n', passText(referenceParity));
end

function s=passText(tf)
if tf, s='PASS'; else, s='FAIL'; end
end

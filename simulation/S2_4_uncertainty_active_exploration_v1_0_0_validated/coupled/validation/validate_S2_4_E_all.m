function gate = validate_S2_4_E_all(runCoupledMissions)
% VALIDATE_S2_4_E_ALL Contracts plus coupled milestones 1 and 2.
if nargin<1,runCoupledMissions=true;end
requestContracts=test_S2_4_E_request_contracts();
decisionContract=test_S2_4_E_competing_decision_contract();
literalGeometryContract=test_S2_4_E_literal_corridor_geometry_contract();
if runCoupledMissions
    milestone1=validate_S2_4_E_milestone_1();
    milestone2=validate_S2_4_E_milestone_2();
else
    milestone1=struct('pass',true,'skipped',true);
    milestone2=struct('pass',true,'skipped',true);
end
gate=struct( ...
    'requestContracts',requestContracts, ...
    'competingDecisionContract',decisionContract, ...
    'literalCorridorGeometryContract',literalGeometryContract, ...
    'milestone1',milestone1, ...
    'milestone2',milestone2, ...
    'pass',requestContracts.pass&&decisionContract.pass&& ...
        literalGeometryContract.pass&&milestone1.pass&&milestone2.pass);
fprintf('\nS2.4-E COMBINED MATLAB GATE — MILESTONES 1+2: %s\n', ...
    localTernary(gate.pass,'PASS','FAIL'));
end

function out=localTernary(condition,a,b)
if condition,out=a;else,out=b;end
end

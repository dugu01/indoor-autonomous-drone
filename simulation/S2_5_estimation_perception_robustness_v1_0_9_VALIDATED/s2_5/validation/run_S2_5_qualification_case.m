function out = run_S2_5_qualification_case(seed,caseName)
% RUN_S2_5_QUALIFICATION_CASE Full coupled mission with compact return.
% Executes the same mission lifecycle as run_S2_5_coupled but suppresses
% console output and heavyweight per-case MAT persistence. Only the summary is
% transferred from a parallel worker to the client. Runtime/autonomy logic is
% unchanged.
out=struct('ok',false,'seed',seed,'caseName',caseName,'summary',struct(), ...
    'errorIdentifier','','errorMessage','','elapsedWall_s',nan);
tWall=tic;
try
    r=run_S2_5_coupled(seed,caseName,false,false,false,false);
    out.ok=true;
    out.summary=r.summary;
catch ME
    out.errorIdentifier=ME.identifier;
    out.errorMessage=ME.message;
end
out.elapsedWall_s=toc(tWall);
end

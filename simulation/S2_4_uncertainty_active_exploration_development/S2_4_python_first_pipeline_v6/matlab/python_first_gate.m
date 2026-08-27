function out = python_first_gate(projectRoot,seed,outJson,requireBenchmarkActivation)
% PYTHON_FIRST_GATE v6 layered physical integration gate invoked by Python.
% S2_4_E_LAYERED_V6
%
% Benchmark seed: requires a clean, physically feasible Tier-3 decoy to be
% present at the same decision at which the Tier-1 target is selected.
% Robustness seeds: all mission/safety/target-selection requirements remain
% mandatory, but stochastic non-appearance of the clean decoy is not itself
% a failure. Whenever such a decoy appears, target-priority is checked.
if nargin<4 || isempty(requireBenchmarkActivation), requireBenchmarkActivation=false; end
if nargin<3, outJson=''; end
if nargin<2 || isempty(seed), seed=0; end
old=pwd; c=onCleanup(@()cd(old)); %#ok<NASGU>
cd(projectRoot);restoredefaultpath;setup_S2_4_E_path(projectRoot);rehash;
geom=test_S2_4_E_literal_corridor_geometry_contract();
r=run_S2_4_coupled(seed,'active_competing_corridors',false,false);s=r.summary;
[e,cleanFound,cleanDecoy]=findEvidence(r.maps.explorationDecisionHistory);

out=struct();out.seed=seed;out.requireBenchmarkActivation=logical(requireBenchmarkActivation);
out.geometryPass=logical(geom.pass);out.missionPass=logical(s.pass);
out.goalReached=logical(s.goalReached);out.rtlAndLanding=logical(s.rtlExecuted==1&&s.landed==1);
out.zeroCollision=s.collisionCount==0;out.zeroGeofence=s.geofenceViolationCount==0;
out.zeroUnknownCommitment=s.unknownCommitmentCount==0;out.zeroUnsafeExecution=s.unsafeViewpointExecutionCount==0;
out.truthIsolation=logical(s.truthIsolationPass==1&&r.maps.uncertaintySidecar.truthAccessCount==0);
out.requestGenerated=s.explorationRequestCount>=1;out.viewpointExecuted=s.explorationExecutedCount>=1;
out.noIrrelevantSelection=s.irrelevantSelectionCount==0;
out.cleanDecoyFound=logical(cleanFound);out.targetSelected=false;
out.selectedInformationGain=nan;out.selectedTargetRelevance=nan;out.selectedTier=nan;
out.decoyInformationGain=nan;out.decoyTier=nan;out.decoyFrontierTrackId=uint64(0);
out.decoyCandidateId=uint64(0);out.decoyRejectionReasons={};out.informationGainRatio=nan;
if ~isempty(e)
    out.targetSelected=logical(e.selected&&e.selectedAccepted&&e.selectedTier==1&&e.selectedTargetRelevance>0&&s.targetRelevantSelectionCount>=1);
    out.selectedInformationGain=double(e.selectedInformationGain);out.selectedTargetRelevance=double(e.selectedTargetRelevance);out.selectedTier=double(e.selectedTier);
end
if cleanFound
    out.decoyInformationGain=double(cleanDecoy.informationGain);out.decoyTier=double(cleanDecoy.tier);
    out.decoyFrontierTrackId=uint64(cleanDecoy.frontierTrackId);out.decoyCandidateId=uint64(cleanDecoy.candidateId);
    out.decoyRejectionReasons=cleanDecoy.rejectionReasons;
    if out.selectedInformationGain>0,out.informationGainRatio=out.decoyInformationGain/out.selectedInformationGain;end
end
out.decoyMoreInformative=out.cleanDecoyFound&&out.targetSelected&&out.decoyInformationGain>out.selectedInformationGain;
out.physicalDecoyMoreInformativeObserved=out.decoyMoreInformative;
out.benchmarkActivationPass=out.cleanDecoyFound&&out.targetSelected&&out.noIrrelevantSelection;
out.conditionalPriorityPass=(~out.cleanDecoyFound)||(out.targetSelected&&out.noIrrelevantSelection);
commonPass=out.geometryPass&&out.missionPass&&out.goalReached&&out.rtlAndLanding&&out.zeroCollision&&out.zeroGeofence&& ...
    out.zeroUnknownCommitment&&out.zeroUnsafeExecution&&out.truthIsolation&&out.requestGenerated&&out.viewpointExecuted&& ...
    out.targetSelected&&out.noIrrelevantSelection&&out.conditionalPriorityPass;
out.pass=commonPass&&((~requireBenchmarkActivation)||out.benchmarkActivationPass);

fprintf('\nPYTHON-FIRST MATLAB PHYSICAL GATE v6 | seed=%d | benchmark=%d | %s\n',seed,requireBenchmarkActivation,tf(out.pass));
fprintf('selected IG / target relevance / tier : %.6f / %.6f / %.0f\n',out.selectedInformationGain,out.selectedTargetRelevance,out.selectedTier);
fprintf('clean Tier-3 decoy present              : %d\n',out.cleanDecoyFound);
if out.cleanDecoyFound
    fprintf('clean decoy IG / observed ratio          : %.6f / %.6f\n',out.decoyInformationGain,out.informationGainRatio);
end
fprintf('benchmark activation / conditional priority: %d / %d\n',out.benchmarkActivationPass,out.conditionalPriorityPass);
if ~isempty(outJson),fid=fopen(outJson,'w');assert(fid>=0,'S2_4:PythonFirstJsonOpen','Could not open %s',outJson);cc=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(out),'char');end
end

function [e,cleanFound,cleanDecoy]=findEvidence(history)
e=[];cleanFound=false;cleanDecoy=struct();fallback=[];
for i=1:numel(history)
    item=history{i};if ~isstruct(item)||~isfield(item,'evidence')||isempty(item.evidence),continue,end
    q=item.evidence;
    if ~(isfield(q,'selected')&&q.selected&&isfield(q,'selectedTargetRelevant')&&q.selectedTargetRelevant&& ...
            isfield(q,'selectedAccepted')&&q.selectedAccepted&&isfield(q,'selectedTier')&&q.selectedTier==1),continue,end
    if isempty(fallback),fallback=q;end
    [found,d]=cleanTier3Decoy(q);
    if found,e=q;cleanFound=true;cleanDecoy=d;return,end
end
if ~isempty(fallback),e=fallback;end
end

function [found,best]=cleanTier3Decoy(e)
found=false;best=struct();if ~isfield(e,'candidateDiagnostics')||isempty(e.candidateDiagnostics),return,end
D=e.candidateDiagnostics;selectedFrontier=uint64(e.selectedFrontierTrackId);bestIG=-inf;
for i=1:numel(D),q=D(i);if uint64(q.frontierTrackId)==selectedFrontier,continue,end
if double(q.informationGain)<=0||double(q.targetRelevance)>0||double(q.tier)~=3,continue,end
rr=normaliseReasons(q.rejectionReasons);if numel(rr)~=1||rr(1)~="IRRELEVANT_EXPLORATION",continue,end
if double(q.informationGain)>bestIG,bestIG=double(q.informationGain);best=q;found=true;end,end
end
function rr=normaliseReasons(x)
if isempty(x),rr=strings(0,1);return,end
if ischar(x),rr=string({x});elseif isstring(x),rr=x(:);elseif iscell(x),rr=string(x(:));else,rr=string.empty(0,1);end
rr=rr(strlength(rr)>0);
end
function s=tf(x)
if x,s='PASS';else,s='FAIL';end
end

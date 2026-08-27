function report = test_S2_4_E_request_contracts()
% TEST_S2_4_E_REQUEST_CONTRACTS Fast deterministic request validation tests.
c=struct('requestValidity_s',1.0);
g=baseGrid();
q=baseRequest();
results=false(6,1);names={ ...
    'valid_known_free_request'; ...
    'blocked_route_rejected'; ...
    'unknown_viewpoint_rejected'; ...
    'local_hold_support_rejected'; ...
    'expired_request_rejected'; ...
    'unrelated_map_version_change_revalidated'};

s=validate_exploration_request_S2_4(c,g,q,0.2);
results(1)=s.valid&&~s.mapChangedSinceRequest;

g2=g;g2.occ(5,5)=true;
s=validate_exploration_request_S2_4(c,g2,q,0.2);
results(2)=~s.valid&&any(strcmp(s.reasons,'ROUTE_BLOCKED'));

g3=g;g3.knownFree(6,6)=false;g3.unknown(6,6)=true;
s=validate_exploration_request_S2_4(c,g3,q,0.2);
results(3)=~s.valid&&any(strcmp(s.reasons,'VIEWPOINT_NOT_EXECUTABLE'));

g4=g;g4.knownFree(5,6)=false;g4.occ(5,6)=true;
s=validate_exploration_request_S2_4(c,g4,q,0.2);
results(4)=~s.valid&&any(strcmp(s.reasons,'LOCAL_HOLD_SUPPORT_INVALID'));

s=validate_exploration_request_S2_4(c,g,q,2.0);
results(5)=~s.valid&&any(strcmp(s.reasons,'REQUEST_EXPIRED'));

g5=g;g5.mapVersion=uint32(2);
s=validate_exploration_request_S2_4(c,g5,q,0.2);
results(6)=s.valid&&s.mapChangedSinceRequest;

for k=1:numel(results)
    fprintf('%02d %-48s %s\n',k,names{k},ternary(results(k),'PASS','FAIL'));
end
report=struct('names',{names},'results',results,'pass',all(results), ...
    'commandIssued',false);
fprintf('S2.4-E REQUEST CONTRACTS: %d/%d %s\n',nnz(results),numel(results), ...
    ternary(report.pass,'PASS','FAIL'));
end

function g=baseGrid()
n=10;g=struct();g.knownFree=true(n);g.unknown=false(n);g.occ=false(n);g.nx=n;g.ny=n;
g.resolution=0.1;g.xs=0:0.1:0.9;g.ys=0:0.1:0.9;g.mapVersion=uint32(1);g.timestamp=0.2;
end
function q=baseRequest()
path=[2 2;3 3;4 4;5 5;6 6];
q=struct('schema','S2_4_EXPLORATION_REQUEST_V1','valid',true, ...
    'action','MOVE_TO_VIEWPOINT','requestId',uint64(1), ...
    'frontierTrackId',uint64(1),'candidateId',uint64(1), ...
    'positionXY',[0.5 0.5],'yaw',0,'startXY',[0.1 0.1], ...
    'pathCells',path,'knownFreeRouteXY',(path-1)*0.1, ...
    'retreatRouteXY',flipud((path-1)*0.1),'mapVersion',uint32(1), ...
    'createdTime_s',0,'validUntil_s',1,'utility',1,'tier',uint8(1), ...
    'informationGain',1,'targetRelevance',1,'rejectionReasons',{{}}, ...
    'statusReason','VALID','commandIssued',false);
end
function o=ternary(c,a,b),if c,o=a;else,o=b;end,end

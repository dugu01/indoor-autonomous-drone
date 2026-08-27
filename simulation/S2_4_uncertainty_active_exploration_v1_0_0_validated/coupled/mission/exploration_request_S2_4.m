function request = exploration_request_S2_4(c,grid,candidate,startXY,tNow)
% EXPLORATION_REQUEST_S2_4 Convert an accepted viewpoint into a mission request.
%
% This is not a controller command. It contains a validated position, yaw,
% known-free route and retreat route for the mission manager to accept or
% reject.
arguments
    c (1,1) struct
    grid (1,1) struct
    candidate
    startXY (1,2) double
    tNow (1,1) double
end
request = emptyRequest();
request.createdTime_s = tNow;
request.validUntil_s = tNow + c.requestValidity_s;
request.startXY = startXY;
request.mapVersion = uint32(grid.mapVersion);
if isempty(candidate)
    request.statusReason = 'NO_SELECTED_CANDIDATE';
    return
end
request.requestId = uint64(candidate.candidateId);
request.frontierTrackId = uint64(candidate.frontierTrackId);
request.candidateId = uint64(candidate.candidateId);
request.utility = double(candidate.utility);
request.tier = uint8(candidate.tier);
request.informationGain = double(candidate.informationGain);
request.targetRelevance = double(candidate.targetRelevance);
request.rejectionReasons = candidate.rejectionReasons;
if ~candidate.accepted || ~isempty(candidate.rejectionReasons)
    request.statusReason = 'CANDIDATE_NOT_ACCEPTED';
    return
end
if size(candidate.path,2)~=2 || isempty(candidate.path)
    request.statusReason = 'EMPTY_KNOWN_FREE_ROUTE';
    return
end
if any(candidate.cell<1) || candidate.cell(1)>grid.ny || candidate.cell(2)>grid.nx
    request.statusReason = 'VIEWPOINT_OUTSIDE_MAP';
    return
end
request.positionXY = [double(grid.xs(candidate.cell(2))) double(grid.ys(candidate.cell(1)))];
request.yaw = double(candidate.yaw);
request.pathCells = double(candidate.path);
request.knownFreeRouteXY = cellsToXY(grid,request.pathCells);
request.retreatRouteXY = flipud(request.knownFreeRouteXY);
request.action = 'MOVE_TO_VIEWPOINT';
request.valid = true;
request.statusReason = 'VALID';
end

function xy=cellsToXY(grid,cells)
x=double(grid.xs(cells(:,2)));y=double(grid.ys(cells(:,1)));
xy=[x(:) y(:)];
end

function r=emptyRequest()
r=struct( ...
    'schema','S2_4_EXPLORATION_REQUEST_V1', ...
    'valid',false, ...
    'action','NONE', ...
    'requestId',uint64(0), ...
    'frontierTrackId',uint64(0), ...
    'candidateId',uint64(0), ...
    'positionXY',[nan nan], ...
    'yaw',nan, ...
    'startXY',[nan nan], ...
    'pathCells',zeros(0,2), ...
    'knownFreeRouteXY',zeros(0,2), ...
    'retreatRouteXY',zeros(0,2), ...
    'mapVersion',uint32(0), ...
    'createdTime_s',nan, ...
    'validUntil_s',nan, ...
    'utility',-inf, ...
    'tier',uint8(3), ...
    'informationGain',0, ...
    'targetRelevance',0, ...
    'rejectionReasons',{{}}, ...
    'statusReason','UNINITIALIZED', ...
    'commandIssued',false);
end

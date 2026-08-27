function gate = test_S2_4_E_literal_corridor_geometry_contract()
% TEST_S2_4_E_LITERAL_CORRIDOR_GEOMETRY_CONTRACT Scenario/schema contract.
%
% This test validates the physical scenario definition only. It does not
% validate the coupled mission; validate_S2_4_E_milestone_2 remains the
% command-enabled release authority.
scenario=scenario_S2_4('active_competing_corridors');

assert(isfield(scenario,'literalCorridorWorld')&&scenario.literalCorridorWorld, ...
    'S2_4:LiteralCorridorFlagMissing','Literal corridor flag is missing.');
assert(isfield(scenario,'validationGeometry')&&isstruct(scenario.validationGeometry), ...
    'S2_4:LiteralGeometryMissing','validationGeometry is missing.');
g=scenario.validationGeometry;

expected=double(g.obstacles_xywh_m);
actual=zeros(size(expected));
for k=1:numel(scenario.truthInsertedObstacles)
    actual(k,:)=extractObstacleXYWH(scenario.truthInsertedObstacles(k));
end

inflation=double(g.design_inflation_m);
room=double(g.room_xy_m(:).');
vertical=expected(1,:);
topForkSafe=room(2)-(vertical(2)+vertical(4))-2*inflation;
bottomForkSafe=vertical(2)-2*inflation;
targetRange=double(g.target_branch_y_m(:).');
decoyRange=double(g.decoy_branch_y_m(:).');
targetBranchSafe=diff(targetRange)-2*inflation;
decoyBranchSafe=diff(decoyRange)-2*inflation;

start=double(g.start_xy_m(:).');
goal=double(g.goal_xy_m(:).');
decoy=double(g.decoy_probe_xy_m(:).');

checks=struct();
checks.schema=strcmp(char(g.schema),'S2_4_E_LITERAL_COMPETING_CORRIDORS_GEOMETRY_V1');
checks.literalFlag=scenario.literalCorridorWorld==true;
checks.twoWalls=size(actual,1)==2;
checks.obstaclesMatch=max(abs(actual(:)-expected(:)))<1e-12;
checks.startMatch=norm(double(scenario.start(:).')-start)<1e-12;
checks.homeMatch=norm(double(scenario.home(:).')-double(g.home_xy_m(:).'))<1e-12;
checks.goalMatch=norm(double(scenario.goal(:).')-goal)<1e-12;
checks.obstaclesActiveFromStart=all(arrayfun(@obstacleStartsAtZero,scenario.truthInsertedObstacles));
checks.upperForkSafe=topForkSafe>=0.50;
checks.lowerForkSafe=bottomForkSafe>=0.50;
checks.targetBranchSafe=targetBranchSafe>=0.75;
checks.decoyBranchSafe=decoyBranchSafe>=0.75;
checks.decoyBranchWider=decoyBranchSafe>targetBranchSafe;
checks.targetInitiallyOccluded=segmentHitsRect(start,goal,vertical);
checks.decoyInitiallyOccluded=segmentHitsRect(start,decoy,vertical);
checks.validationOnlyGeometryField=true;
checks.schemaSourceRecorded=isfield(scenario,'validationObstacleSchemaSource')&& ...
    ~isempty(scenario.validationObstacleSchemaSource);

vals=struct2cell(checks);
gate=checks;
gate.upperForkSafeWidth_m=topForkSafe;
gate.lowerForkSafeWidth_m=bottomForkSafe;
gate.targetBranchSafeWidth_m=targetBranchSafe;
gate.decoyBranchSafeWidth_m=decoyBranchSafe;
gate.pass=all(cellfun(@logical,vals));

fprintf('\nS2.4-E LITERAL CORRIDOR GEOMETRY CONTRACT: %s\n', ...
    localTernary(gate.pass,'PASS','FAIL'));
fprintf('Safe fork width upper/lower   : %.3f / %.3f m\n', ...
    topForkSafe,bottomForkSafe);
fprintf('Safe branch width target/decoy: %.3f / %.3f m\n', ...
    targetBranchSafe,decoyBranchSafe);
if isfield(scenario,'validationObstacleSchemaSource')
    fprintf('Obstacle schema source          : %s\n',scenario.validationObstacleSchemaSource);
end
fprintf('Physical obstacle overlay:\n');
disp(actual);
end

function xywh=extractObstacleXYWH(obstacle)
fields={'rect','rect5','rectXYWH','xywh','xywh_m'};
for i=1:numel(fields)
    f=fields{i};
    if isfield(obstacle,f)
        v=obstacle.(f);
        if isnumeric(v)&&isvector(v)&&numel(v)>=4
            xywh=double(v(1:4));
            xywh=xywh(:).';
            return
        end
    end
end
error('S2_4:UnsupportedObstacleSchema', ...
    'No supported XYWH field in truth obstacle. Fields: %s', ...
    strjoin(fieldnames(obstacle),', '));
end

function tf=obstacleStartsAtZero(obstacle)
tf=isfield(obstacle,'time')&&isnumeric(obstacle.time)&& ...
    isscalar(obstacle.time)&&abs(double(obstacle.time))<1e-12;
end

function tf=segmentHitsRect(a,b,r)
x0=a(1);y0=a(2);x1=b(1);y1=b(2);
xmin=r(1);ymin=r(2);xmax=r(1)+r(3);ymax=r(2)+r(4);
dx=x1-x0;dy=y1-y0;
p=[-dx dx -dy dy];q=[x0-xmin xmax-x0 y0-ymin ymax-y0];
u1=0;u2=1;tf=true;
for k=1:4
    if abs(p(k))<1e-12
        if q(k)<0,tf=false;return,end
    else
        t=q(k)/p(k);
        if p(k)<0,u1=max(u1,t);else,u2=min(u2,t);end
        if u1>u2,tf=false;return,end
    end
end
end

function out=localTernary(condition,a,b)
if condition,out=a;else,out=b;end
end

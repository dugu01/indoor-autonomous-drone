function scenario = make_literal_competing_corridors_S2_4(scenario)
% MAKE_LITERAL_COMPETING_CORRIDORS_S2_4 Build the literal two-corridor world.
%
% Scenario-only truth overlay. The frozen S2.3 parent and the S2.4
% decision/planning/control implementation are not modified.
%
% Important schema rule:
%   truthInsertedObstacles is an event list and may legitimately be empty in
%   GOAL_REQUIRES_SCAN. Therefore this function never assumes that the base
%   scenario contains a prototype obstacle. It resolves the exact frozen-S2.3
%   inserted-obstacle struct schema from an inherited scenario that actually
%   contains an insertion event, then instantiates the two t=0 corridor walls
%   using that unmodified schema.
%
% Corridor semantics are stored only in validationGeometry. The S2.4 static
% audit forbids autonomy decision modules from reading validationGeometry.

geometryFile=fullfile(fileparts(mfilename('fullpath')), ...
    'literal_competing_corridors_geometry.json');
assert(isfile(geometryFile),'S2_4:LiteralGeometryMissing', ...
    'Missing literal corridor geometry file: %s',geometryFile);
geom=jsondecode(fileread(geometryFile));

[prototype,schemaSource]=resolveObstaclePrototype(scenario);
rects=double(geom.obstacles_xywh_m);
obstacles=repmat(prototype,1,size(rects,1));
for k=1:size(rects,1)
    obstacles(k)=setObstacleGeometry(obstacles(k),rects(k,:), ...
        double(geom.obstacle_height_m));
end
scenario.truthInsertedObstacles=obstacles;

scenario.start=double(geom.start_xy_m(:).');
scenario.home=double(geom.home_xy_m(:).');
scenario.goal=double(geom.goal_xy_m(:).');
if isfield(scenario,'alternateLandingZones')
    scenario.alternateLandingZones=double(geom.alternate_landing_zones_xy_m);
end

% Validation-only labels/geometry. No autonomy module may consume these.
scenario.validationGeometry=geom;
scenario.validationObstacleSchemaSource=schemaSource;
scenario.literalCorridorWorld=true;
end

function [prototype,sourceName]=resolveObstaclePrototype(baseScenario)
% Obtain the actual frozen-parent inserted-obstacle struct schema without
% guessing it. GOAL_REQUIRES_SCAN may have an empty insertion event list.
if isfield(baseScenario,'truthInsertedObstacles') && ...
        ~isempty(baseScenario.truthInsertedObstacles)
    prototype=baseScenario.truthInsertedObstacles(1);
    sourceName='base:goal_requires_scan';
    validatePrototype(prototype,sourceName);
    return
end

% These are validated S2.3 scenarios. Some exercise obstacle insertion or
% map-change events. Invalid/unsupported names are harmlessly skipped so the
% resolver remains compatible with the frozen parent release.
candidates={ ...
    'hidden_obstacle_replan', ...
    'occluded_obstacle', ...
    'dead_end_recovery', ...
    'dynamic_to_static_mapping', ...
    'unreachable_goal', ...
    'unknown_narrow_passage'};

seen={};
for i=1:numel(candidates)
    name=candidates{i};
    try
        s=scenario_S2_3(name);
    catch
        continue
    end
    seen{end+1}=name; %#ok<AGROW>
    if isfield(s,'truthInsertedObstacles') && ~isempty(s.truthInsertedObstacles)
        prototype=s.truthInsertedObstacles(1);
        sourceName=['inherited:' name];
        validatePrototype(prototype,sourceName);
        return
    end
end

if isempty(seen)
    searched='none resolved';
else
    searched=strjoin(seen,', ');
end
error('S2_4:ObstacleSchemaUnavailable', ...
    ['Could not resolve a nonempty frozen-S2.3 truthInsertedObstacles ' ...
     'prototype. GOAL_REQUIRES_SCAN is allowed to have an empty event list. ' ...
     'Inherited scenarios successfully inspected: %s.'],searched);
end

function validatePrototype(obstacle,sourceName)
assert(isstruct(obstacle)&&isscalar(obstacle), ...
    'S2_4:ObstaclePrototypeInvalid', ...
    'Obstacle prototype from %s must be a scalar struct.',sourceName);
assert(isfield(obstacle,'time')&&isnumeric(obstacle.time)&&isscalar(obstacle.time), ...
    'S2_4:ObstacleTimeSchemaUnavailable', ...
    'Obstacle prototype from %s is missing required scalar time.',sourceName);

fields={'rect','rect5','rectXYWH','xywh','xywh_m'};
hasGeometry=false;
for i=1:numel(fields)
    f=fields{i};
    if isfield(obstacle,f)
        v=obstacle.(f);
        if isnumeric(v)&&isvector(v)&&numel(v)>=4
            hasGeometry=true;
            break
        end
    end
end
assert(hasGeometry,'S2_4:UnsupportedObstacleSchema', ...
    ['Obstacle prototype from %s has no supported XYWH geometry field. ' ...
     'Available fields: %s'],sourceName,strjoin(fieldnames(obstacle),', '));
end

function obstacle=setObstacleGeometry(obstacle,xywh,height_m)
% Modify only geometry/time fields already present in the frozen prototype.
rectFields={'rect','rect5','rectXYWH','xywh','xywh_m'};
matched='';
for i=1:numel(rectFields)
    f=rectFields{i};
    if isfield(obstacle,f)
        v=obstacle.(f);
        if isnumeric(v)&&isvector(v)&&numel(v)>=4
            v=double(v(:).');
            v(1:4)=xywh;
            if numel(v)>=5
                v(5)=height_m;
            end
            obstacle.(f)=v;
            matched=f;
            break
        end
    end
end
if isempty(matched)
    error('S2_4:UnsupportedObstacleSchema', ...
        ['Could not find a supported XYWH field in the inherited obstacle ' ...
         'prototype. Available fields: %s'],strjoin(fieldnames(obstacle),', '));
end

% mission_lifecycle_manager_S2_4 reads truthInsertedObstacles(i).time.
obstacle.time=0;
if isfield(obstacle,'enabled') && ...
        (islogical(obstacle.enabled)||isnumeric(obstacle.enabled))
    obstacle.enabled=true;
end

% Preserve vertical semantics if the frozen schema exposes them.
heightFields={'height','height_m','zMax','zMax_m','z_max_m'};
for i=1:numel(heightFields)
    f=heightFields{i};
    if isfield(obstacle,f)&&isnumeric(obstacle.(f))&&isscalar(obstacle.(f))
        obstacle.(f)=height_m;
    end
end
end

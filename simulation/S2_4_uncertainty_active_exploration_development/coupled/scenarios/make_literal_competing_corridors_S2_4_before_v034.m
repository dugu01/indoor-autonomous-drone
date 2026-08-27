function scenario = make_literal_competing_corridors_S2_4(scenario)
% MAKE_LITERAL_COMPETING_CORRIDORS_S2_4 Build the literal two-corridor world.
%
% This is a scenario-only truth overlay. It deliberately does not modify the
% frozen S2.3 parent or any S2.4 decision/planning/control implementation.
% The inherited GOAL_REQUIRES_SCAN obstacle struct is used as a schema
% prototype so the patch remains compatible with the frozen sensor simulator.
%
% Corridor semantics are stored under scenario.validationGeometry only for
% post-run validation. The autonomy decision modules are forbidden from
% reading that field by the S2.4 static audit.

geometryFile=fullfile(fileparts(mfilename('fullpath')), ...
    'literal_competing_corridors_geometry.json');
assert(isfile(geometryFile),'S2_4:LiteralGeometryMissing', ...
    'Missing literal corridor geometry file: %s',geometryFile);
geom=jsondecode(fileread(geometryFile));

assert(isfield(scenario,'truthInsertedObstacles')&& ...
    ~isempty(scenario.truthInsertedObstacles), ...
    'S2_4:ObstacleSchemaUnavailable', ...
    ['The inherited goal_requires_scan scenario must contain at least one ' ...
     'truthInsertedObstacles element so its frozen S2.3 obstacle schema can ' ...
     'be reused without guessing field names.']);

rects=double(geom.obstacles_xywh_m);
prototype=scenario.truthInsertedObstacles(1);
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
scenario.literalCorridorWorld=true;
end

function obstacle=setObstacleGeometry(obstacle,xywh,height_m)
% Adapt the known frozen-parent obstacle prototype without inventing a new
% struct schema. Supported rectangle field names are intentionally narrow;
% an unfamiliar parent schema fails loudly instead of being guessed.
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

% mission_lifecycle_manager_S2_4 reads truthInsertedObstacles(i).time
% explicitly, so this field is part of the known frozen-parent contract.
assert(isfield(obstacle,'time')&&isnumeric(obstacle.time)&&isscalar(obstacle.time), ...
    'S2_4:ObstacleTimeSchemaUnavailable', ...
    'Inherited truth obstacle is missing the required scalar time field.');
obstacle.time=0;
if isfield(obstacle,'enabled')&&(islogical(obstacle.enabled)||isnumeric(obstacle.enabled))
    obstacle.enabled=true;
end

% Preserve the prototype's vertical semantics where possible, while ensuring
% a full-height indoor wall for the nominal flight altitude.
heightFields={'height','height_m','zMax','zMax_m','z_max_m'};
for i=1:numel(heightFields)
    f=heightFields{i};
    if isfield(obstacle,f)&&isnumeric(obstacle.(f))&&isscalar(obstacle.(f))
        obstacle.(f)=height_m;
    end
end
end

function [planner,path,stats] = dstar_lite_S2_2(action,varargin)
% DSTAR_LITE_S2_2  Incremental 8-connected grid replanning.
%
% Usage:
%   [planner,path,stats] = dstar_lite_S2_2('init',grid,startXY,goalXY)
%   [planner,path,stats] = dstar_lite_S2_2('repair',planner,newGrid,newStartXY,changedMask)
%   [planner,path,stats] = dstar_lite_S2_2('refresh',planner,newGrid,newStartXY)
%
% The implementation follows the D* Lite update structure of Koenig and
% Likhachev. OPEN uses lazy deletion for compatibility with base MATLAB.

switch lower(action)
    case 'init'
        grid = varargin{1}; startXY = varargin{2}; goalXY = varargin{3};
        planner = initialize_planner(grid,startXY,goalXY);
        [planner,expanded] = compute_shortest_path(planner);
        path = extract_path(planner);
        stats = struct('expanded',expanded,'totalExpanded',planner.totalExpanded,'success',~isempty(path));

    case 'repair'
        planner = varargin{1}; grid = varargin{2}; startXY = varargin{3}; changedMask = varargin{4};
        % Use the updated occupancy grid before relocating the start.
        % Otherwise nearest_free() can select a cell that was free only in
        % the old map after an obstacle insertion/promotion.
        planner.grid = grid;
        planner = move_start(planner,startXY);
        planner = update_changed_cells(planner,changedMask);
        [planner,expanded] = compute_shortest_path(planner);
        path = extract_path(planner);
        stats = struct('expanded',expanded,'totalExpanded',planner.totalExpanded,'success',~isempty(path));

    case 'refresh'
        planner = varargin{1}; grid = varargin{2}; startXY = varargin{3};
        % Refresh against the current map, not the stale map.
        planner.grid = grid;
        planner = move_start(planner,startXY);
        [planner,expanded] = compute_shortest_path(planner);
        path = extract_path(planner);
        stats = struct('expanded',expanded,'totalExpanded',planner.totalExpanded,'success',~isempty(path));

    otherwise
        error('S2_2:BadDStarAction','Unknown D* Lite action: %s',action);
end
end

function planner = initialize_planner(grid,startXY,goalXY)
[sx,sy] = point_to_index(grid,startXY);
[gx,gy] = point_to_index(grid,goalXY);
[sx,sy,ok] = nearest_free(grid,sx,sy,12);
if ~ok || ~in_grid(grid,gx,gy) || grid.occ(gy,gx)
    planner = empty_planner(grid,startXY,goalXY);
    planner.valid = false;
    return;
end
planner = empty_planner(grid,startXY,goalXY);
planner.start = [sx sy]; planner.last = [sx sy]; planner.goal = [gx gy];
planner.g = inf(grid.ny,grid.nx); planner.rhs = inf(grid.ny,grid.nx);
planner.rhs(gy,gx) = 0;
planner.openK1 = nan(grid.ny,grid.nx); planner.openK2 = nan(grid.ny,grid.nx);
planner.heap = zeros(0,4); planner.km = 0; planner.valid = true; planner.totalExpanded = 0;
key = calculate_key(planner,[gx gy]);
planner = open_insert(planner,[gx gy],key);
end

function p = empty_planner(grid,startXY,goalXY)
p = struct('grid',grid,'start',[1 1],'last',[1 1],'goal',[1 1], ...
    'g',[],'rhs',[],'openK1',[],'openK2',[],'heap',zeros(0,4), ...
    'km',0,'valid',false,'totalExpanded',0,'startXY',startXY,'goalXY',goalXY);
end

function planner = move_start(planner,newStartXY)
if ~planner.valid, return; end
[sx,sy] = point_to_index(planner.grid,newStartXY);
[sx,sy,ok] = nearest_free(planner.grid,sx,sy,12);
if ~ok
    planner.valid = false; return;
end
newStart = [sx sy];
if any(newStart ~= planner.start)
    planner.km = planner.km + octile(planner.last,newStart);
    planner.last = newStart;
    planner.start = newStart;
end
end

function planner = update_changed_cells(planner,changedMask)
if ~planner.valid || isempty(changedMask), return; end
[ys,xs] = find(changedMask);
mark = false(planner.grid.ny,planner.grid.nx);
for i=1:numel(xs)
    x=xs(i); y=ys(i); mark(y,x)=true;
    nbr = all_adjacent(planner.grid,[x y]);
    for j=1:size(nbr,1), mark(nbr(j,2),nbr(j,1))=true; end
end
[uy,ux] = find(mark);
for i=1:numel(ux)
    planner = update_vertex(planner,[ux(i) uy(i)]);
end
end

function [planner,count] = compute_shortest_path(planner)
count = 0;
if ~planner.valid, return; end
maxExpansions = 500000;
while key_less(top_key(planner),calculate_key(planner,planner.start)) || ...
        abs(planner.rhs(planner.start(2),planner.start(1)) - planner.g(planner.start(2),planner.start(1))) > 1e-12
    if isempty(planner.heap) || count >= maxExpansions, break; end
    [planner,kOld,u,ok] = open_pop(planner);
    if ~ok, break; end
    kNew = calculate_key(planner,u);
    if key_less(kOld,kNew)
        planner = open_insert(planner,u,kNew);
    elseif planner.g(u(2),u(1)) > planner.rhs(u(2),u(1))
        planner.g(u(2),u(1)) = planner.rhs(u(2),u(1));
        pred = all_adjacent(planner.grid,u);
        for j=1:size(pred,1), planner = update_vertex(planner,pred(j,:)); end
    else
        planner.g(u(2),u(1)) = inf;
        planner = update_vertex(planner,u);
        pred = all_adjacent(planner.grid,u);
        for j=1:size(pred,1), planner = update_vertex(planner,pred(j,:)); end
    end
    count = count + 1;
end
planner.totalExpanded = planner.totalExpanded + count;
end

function planner = update_vertex(planner,u)
if any(u ~= planner.goal)
    best = inf;
    if free_cell(planner.grid,u)
        nbr = reachable_neighbors(planner.grid,u);
        for j=1:size(nbr,1)
            s=nbr(j,:);
            best=min(best,edge_cost(u,s)+planner.g(s(2),s(1)));
        end
    end
    planner.rhs(u(2),u(1)) = best;
end
planner = open_remove(planner,u);
if abs(planner.g(u(2),u(1))-planner.rhs(u(2),u(1))) > 1e-12
    planner = open_insert(planner,u,calculate_key(planner,u));
end
end

function path = extract_path(planner)
path = zeros(0,2);
if ~planner.valid || ~free_cell(planner.grid,planner.start) || ...
        ~isfinite(planner.g(planner.start(2),planner.start(1)))
    return;
end
u=planner.start; cells=u; seen=false(planner.grid.ny,planner.grid.nx); seen(u(2),u(1))=true;
for k=1:10000
    if all(u==planner.goal), break; end
    nbr=reachable_neighbors(planner.grid,u);
    if isempty(nbr), path=zeros(0,2); return; end
    vals=inf(size(nbr,1),1); hvals=vals;
    for j=1:size(nbr,1)
        s=nbr(j,:); vals(j)=edge_cost(u,s)+planner.g(s(2),s(1)); hvals(j)=octile(s,planner.goal);
    end
    [~,ord]=sortrows([vals hvals],[1 2]);
    u=nbr(ord(1),:);
    if seen(u(2),u(1)), path=zeros(0,2); return; end
    seen(u(2),u(1))=true; cells(end+1,:)=u; %#ok<AGROW>
end
if ~all(cells(end,:)==planner.goal), return; end
path=zeros(size(cells,1),2);
for i=1:size(cells,1), path(i,:)=index_to_point(planner.grid,cells(i,1),cells(i,2)); end
end

function k = calculate_key(planner,u)
m=min(planner.g(u(2),u(1)),planner.rhs(u(2),u(1)));
k=[m+octile(planner.start,u)+planner.km,m];
end

function planner = open_insert(planner,u,key)
planner.openK1(u(2),u(1))=key(1); planner.openK2(u(2),u(1))=key(2);
planner.heap(end+1,:)=[key u]; %#ok<AGROW>
end

function planner = open_remove(planner,u)
planner.openK1(u(2),u(1))=nan; planner.openK2(u(2),u(1))=nan;
end

function key = top_key(planner)
key=[inf inf];
if isempty(planner.heap), return; end
valid=false(size(planner.heap,1),1);
for i=1:size(planner.heap,1)
    u=planner.heap(i,3:4); k=planner.heap(i,1:2);
    valid(i)=same_key([planner.openK1(u(2),u(1)) planner.openK2(u(2),u(1))],k);
end
if ~any(valid), return; end
rows=planner.heap(valid,:); [~,idx]=sortrows(rows(:,1:2),[1 2]); key=rows(idx(1),1:2);
end

function [planner,key,u,ok] = open_pop(planner)
key=[inf inf]; u=[0 0]; ok=false;
while ~isempty(planner.heap)
    [~,idx]=sortrows(planner.heap(:,1:2),[1 2]); row=planner.heap(idx(1),:); planner.heap(idx(1),:)=[];
    ku=[planner.openK1(row(4),row(3)) planner.openK2(row(4),row(3))];
    if same_key(ku,row(1:2))
        key=row(1:2); u=row(3:4); planner=open_remove(planner,u); ok=true; return;
    end
end
end

function tf = same_key(a,b)
tf=all(isfinite(a)) && abs(a(1)-b(1))<=1e-12 && abs(a(2)-b(2))<=1e-12;
end

function tf = key_less(a,b)
tf = a(1)<b(1)-1e-12 || (abs(a(1)-b(1))<=1e-12 && a(2)<b(2)-1e-12);
end

function n = reachable_neighbors(grid,u)
alln=all_adjacent(grid,u); keep=false(size(alln,1),1);
for i=1:size(alln,1)
    v=alln(i,:);
    if ~free_cell(grid,v), continue; end
    dx=v(1)-u(1);dy=v(2)-u(2);
    if dx~=0 && dy~=0
        if ~free_cell(grid,[u(1)+dx u(2)]) || ~free_cell(grid,[u(1) u(2)+dy]), continue; end
    end
    keep(i)=true;
end
n=alln(keep,:);
end

function n = all_adjacent(grid,u)
d=[1 0;-1 0;0 1;0 -1;1 1;1 -1;-1 1;-1 -1]; n=u+d;
keep=n(:,1)>=1 & n(:,1)<=grid.nx & n(:,2)>=1 & n(:,2)<=grid.ny; n=n(keep,:);
end

function tf = free_cell(grid,u)
tf=in_grid(grid,u(1),u(2)) && ~grid.occ(u(2),u(1));
end

function c = edge_cost(a,b)
c=hypot(a(1)-b(1),a(2)-b(2));
end

function h = octile(a,b)
dx=abs(a(1)-b(1));dy=abs(a(2)-b(2));h=max(dx,dy)+(sqrt(2)-1)*min(dx,dy);
end

function [ix,iy] = point_to_index(grid,p)
ix=round(p(1)/grid.resolution)+1;iy=round(p(2)/grid.resolution)+1;
end

function p = index_to_point(grid,ix,iy)
p=[(ix-1)*grid.resolution,(iy-1)*grid.resolution];
end

function tf = in_grid(grid,ix,iy)
tf=ix>=1 && iy>=1 && ix<=grid.nx && iy<=grid.ny;
end

function [bx,by,ok] = nearest_free(grid,ix0,iy0,maxCells)
ok=false;bx=ix0;by=iy0;
if in_grid(grid,ix0,iy0) && ~grid.occ(iy0,ix0),ok=true;return;end
best=inf;
for r=1:maxCells
    for iy=max(1,iy0-r):min(grid.ny,iy0+r)
        for ix=max(1,ix0-r):min(grid.nx,ix0+r)
            if grid.occ(iy,ix),continue;end
            d2=(ix-ix0)^2+(iy-iy0)^2;
            if d2<best,best=d2;bx=ix;by=iy;ok=true;end
        end
    end
    if ok,return;end
end
end

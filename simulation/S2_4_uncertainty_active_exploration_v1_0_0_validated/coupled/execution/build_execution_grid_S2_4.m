function g = build_execution_grid_S2_4(grid)
% BUILD_EXECUTION_GRID_S2_4 Add explicit raw/execution fields to S2.3 grid.
%
% Frontier extraction uses raw free/unknown classes. Candidate execution
% uses grid.occ, which already contains inherited obstacle and unknown-space
% inflation. No map value is changed.
arguments
    grid (1,1) struct
end
required = {'occ','knownFree','unknown','staticOccupied','dynamicOccupied', ...
    'xs','ys','nx','ny','resolution','mapVersion','timestamp','lastObservedXY'};
for k=1:numel(required)
    if ~isfield(grid,required{k})
        error('S2_4:GridSchema','Missing S2.3 grid field: %s',required{k});
    end
end
g = grid;
g.navigationBlocked = logical(grid.occ);
g.rawOccupied = logical(grid.staticOccupied) | logical(grid.dynamicOccupied);
g.staticOccupiedRaw = logical(grid.staticOccupied);
g.dynamicOccupiedRaw = logical(grid.dynamicOccupied);
g.knownFree = logical(grid.knownFree) & ~g.rawOccupied & ~logical(grid.unknown);
g.unknown = logical(grid.unknown) & ~g.rawOccupied;
end

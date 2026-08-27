function smooth = smooth_path_S2_2(grid, path)
% SMOOTH_PATH_S2_2  Line-of-sight shortcut smoothing over inflated grid.
if isempty(path)
    smooth = zeros(0,2);
    return;
end
smooth = path(1,:);
i = 1;
while i < size(path,1)
    j = size(path,1);
    while j > i+1 && segment_occupied_grid_S2_2(grid,path(i,:),path(j,:))
        j = j - 1;
    end
    smooth(end+1,:) = path(j,:); %#ok<AGROW>
    i = j;
end
end

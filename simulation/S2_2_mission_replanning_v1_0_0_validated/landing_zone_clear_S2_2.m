function clear = landing_zone_clear_S2_2(grid,xy,radius_m)
% LANDING_ZONE_CLEAR_S2_2 Check an inflated circular landing footprint.
xy=double(xy(:).');radius_m=max(0,double(radius_m));
if numel(xy)~=2||any(~isfinite(xy))
    clear=false;return;
end
res=grid.resolution;
ix0=round(xy(1)/res)+1;iy0=round(xy(2)/res)+1;
rCells=ceil(radius_m/res)+1;
clear=true;
for iy=max(1,iy0-rCells):min(grid.ny,iy0+rCells)
    for ix=max(1,ix0-rCells):min(grid.nx,ix0+rCells)
        cellXY=[grid.xs(ix) grid.ys(iy)];
        if norm(cellXY-xy)<=radius_m+0.5*sqrt(2)*res && grid.occ(iy,ix)
            clear=false;return;
        end
    end
end
% The full vehicle footprint must also remain inside the room grid.
if xy(1)-radius_m<grid.xs(1)||xy(1)+radius_m>grid.xs(end)|| ...
        xy(2)-radius_m<grid.ys(1)||xy(2)+radius_m>grid.ys(end)
    clear=false;
end
end

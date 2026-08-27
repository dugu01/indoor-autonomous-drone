function vmax = static_braking_speed_S2_2(cfg,pos,dirn,grid)
% STATIC_BRAKING_SPEED_S2_2  Speed cap from delay + braking distance.
% Uses the same conservative segment-grid traversal as path/trajectory
% validation, avoiding fixed-point sampling gaps near cell corners.
pos=pos(:).';dirn=dirn(:).';
if norm(dirn)<1e-12,vmax=0;return;end
dirn=dirn/norm(dirn);step=grid.resolution/4;maxRange=2.5;
lastFree=0;previous=pos;
for d=step:step:maxRange
    current=pos+dirn*d;
    if segment_occupied_grid_S2_2(grid,previous,current)
        break;
    end
    lastFree=d;
    previous=current;
end
clearance=max(0,lastFree);
a=cfg.maxDecelXY_mps2;t=cfg.sensorControlDelay_s;
vmax=max(0,-a*t+sqrt(max(0,(a*t)^2+2*a*clearance)));
vmax=min(cfg.maxSpeedXY_mps,vmax);
end

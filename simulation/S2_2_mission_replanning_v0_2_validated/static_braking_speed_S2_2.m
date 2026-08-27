function vmax = static_braking_speed_S2_2(cfg,pos,dirn,grid)
% STATIC_BRAKING_SPEED_S2_2  Speed cap from delay + braking distance.
pos=pos(:).';dirn=dirn(:).';
if norm(dirn)<1e-12,vmax=0;return;end
dirn=dirn/norm(dirn);step=grid.resolution/4;maxRange=2.5;d=0;
while d<=maxRange
    p=pos+dirn*d;
    ix=round(p(1)/grid.resolution)+1;iy=round(p(2)/grid.resolution)+1;
    if ix<1||iy<1||ix>grid.nx||iy>grid.ny||grid.occ(iy,ix),break;end
    d=d+step;
end
clearance=max(0,d-grid.resolution);
a=cfg.maxDecelXY_mps2;t=cfg.sensorControlDelay_s;
vmax=max(0,-a*t+sqrt(max(0,(a*t)^2+2*a*clearance)));
vmax=min(cfg.maxSpeedXY_mps,vmax);
end

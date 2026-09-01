function pass = validate_known_free_stop_S2_3(cfg,grid,path,currentVelocity)
% VALIDATE_KNOWN_FREE_STOP_S2_3 Verify terminal stopping reserve.
pass=false;if isempty(path)||size(path,1)<2,return;end
speed=norm(currentVelocity(:));
dStop=speed^2/(2*max(cfg.maxDecelXY_mps2,eps))+ ...
    speed*cfg.sensorControlDelay_s+cfg.mapStopExtraMargin_m;
lengthAvailable=0;
for i=size(path,1)-1:-1:1
    lengthAvailable=lengthAvailable+norm(path(i+1,:)-path(i,:));
    if segment_occupied_grid_S2_2(grid,path(i,:),path(i+1,:)),return;end
    if lengthAvailable>=dStop,break;end
end
if lengthAvailable<dStop,return;end
endPt=path(end,:);radius=cfg.collisionRadius+cfg.controlMargin;
pass=landing_zone_clear_S2_2(grid,endPt,radius);
end

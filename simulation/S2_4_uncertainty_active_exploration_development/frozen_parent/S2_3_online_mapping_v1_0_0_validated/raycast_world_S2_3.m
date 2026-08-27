function [range,hit] = raycast_world_S2_3(cfg,world,origin,direction,minRange,maxRange)
% RAYCAST_WORLD_S2_3 Conservative ray marching against room and obstacles.
direction=direction(:);n=norm(direction);
if n<1e-12,range=maxRange;hit=false;return;end
direction=direction/n;range=maxRange;hit=false;
for s=minRange:cfg.raycastStep_m:maxRange
    p=origin(:)+s*direction;
    if p(1)<=0||p(1)>=cfg.room(1)||p(2)<=0||p(2)>=cfg.room(2)||p(3)<=0||p(3)>=cfg.room(3)
        range=s;hit=true;return;
    end
    for j=1:size(world.staticRects5,1)
        r=world.staticRects5(j,:);
        if p(1)>=r(1)&&p(1)<=r(1)+r(3)&&p(2)>=r(2)&&p(2)<=r(2)+r(4)&&p(3)<=r(5)
            range=s;hit=true;return;
        end
    end
    for j=1:numel(world.dynamic)
        d=world.dynamic(j);
        if norm(p(1:2).'-d.p)<=d.radius&&p(3)<=1.9
            range=s;hit=true;return;
        end
    end
end
end

function [cmd,modified,minMargin,minTTC] = velocity_obstacle_filter_S2_2(cfg,pos,desired,dynStates,grid)
% VELOCITY_OBSTACLE_FILTER_S2_2  Finite-horizon VO candidate filter.
% Every returned command is first checked against the inflated static grid.
% If no dynamically safe static-feasible candidate exists, the vehicle
% holds instead of selecting an escape command that crosses a static cell.
pos=pos(:).';desired=desired(:).';
if isempty(dynStates)
    cmd=desired(:);modified=false;minMargin=inf;minTTC=inf;return;
end
speed=norm(desired);
if speed<1e-9,base=[1 0];else,base=desired/speed;end
angles=deg2rad([0 20 -20 40 -40 65 -65 90 -90 180]);
levels=unique([0,0.35*cfg.maxSpeedXY_mps,0.65*cfg.maxSpeedXY_mps, ...
    max(speed,0.75*cfg.maxSpeedXY_mps),cfg.maxSpeedXY_mps]);
candidates=[0 0];
for i=2:numel(levels)
    for j=1:numel(angles)
        c=cos(angles(j));s=sin(angles(j));
        d=[c*base(1)-s*base(2),s*base(1)+c*base(2)];
        candidates(end+1,:)=levels(i)*d; %#ok<AGROW>
    end
end
bestScore=inf;bestEscape=-inf;cmd=[0 0];minMargin=-inf;minTTC=inf;
foundSafe=false;foundStaticEscape=false;
for i=1:size(candidates,1)
    cand=candidates(i,:);
    preview=pos+cand*min(1.0,cfg.predictionHorizon_s);
    staticOK=~segment_occupied_grid_S2_2(grid,pos,preview);
    if ~staticOK
        continue;
    end
    [margin,ttc]=predicted_margin(cfg,pos,cand,dynStates);
    escapeScore=margin-0.15*norm(cand-desired);
    if escapeScore>bestEscape
        bestEscape=escapeScore;escapeCand=cand;escapeMargin=margin;escapeTTC=ttc;
        foundStaticEscape=true;
    end
    if margin>=0
        score=norm(cand-desired)^2-0.08*min(margin,2.0)-0.03*norm(cand);
        if score<bestScore
            bestScore=score;cmd=cand;minMargin=margin;minTTC=ttc;foundSafe=true;
        end
    end
end
if ~foundSafe
    if foundStaticEscape && escapeMargin>=0
        cmd=escapeCand;minMargin=escapeMargin;minTTC=escapeTTC;
    else
        % No collision-free velocity exists over the prediction horizon.
        % Holding is safer than an escape velocity that is predicted to
        % collide dynamically or that crosses an inflated static cell.
        cmd=[0 0];
        [minMargin,minTTC]=predicted_margin(cfg,pos,cmd,dynStates);
    end
end
modified=norm(cmd-desired)>1e-6;
cmd=cmd(:);
end

function [minMargin,minTTC]=predicted_margin(cfg,pos,cmd,dynStates)
minMargin=inf;minTTC=inf;
for k=1:numel(dynStates)
    rp=dynStates(k).p-pos;rv=dynStates(k).v-cmd;
    vv=dot(rv,rv);
    if vv<1e-10,tcpa=0;else,tcpa=max(0,min(cfg.predictionHorizon_s,-dot(rp,rv)/vv));end
    d=norm(rp+rv*tcpa);
    safe=cfg.collisionRadius+dynStates(k).radius+cfg.dynamicBuffer_m;
    margin=d-safe;
    if margin<minMargin,minMargin=margin;minTTC=tcpa;end
end
end

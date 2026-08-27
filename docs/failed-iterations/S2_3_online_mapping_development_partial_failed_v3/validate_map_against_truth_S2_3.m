function metrics = validate_map_against_truth_S2_3(cfg,map,world)
% VALIDATE_MAP_AGAINST_TRUTH_S2_3 Independent final-map validation.
% Truth is used only here, after the autonomy run. Floor contact is validated
% by the plant and touchdown model rather than volumetric occupancy truth. Occupied recall is scored
% on truth-occupied voxels that received the configured repeated physical
% hit endpoints; ordinary free-ray traversal is not evidence that obstacle
% interior was observable.
truthOcc=false(map.ny,map.nx,map.nz);
for iy=1:map.ny
 for ix=1:map.nx
  for iz=1:map.nz
   p=[map.xs(ix) map.ys(iy) map.zs(iz)];
   if p(1)<=0||p(1)>=cfg.room(1)||p(2)<=0||p(2)>=cfg.room(2)||p(3)>=cfg.room(3),truthOcc(iy,ix,iz)=true;continue;end
   for j=1:size(world.staticRects5,1)
    r=world.staticRects5(j,:);
    if p(1)>=r(1)&&p(1)<=r(1)+r(3)&&p(2)>=r(2)&&p(2)<=r(2)+r(4)&&p(3)<=r(5),truthOcc(iy,ix,iz)=true;break;end
   end
  end
 end
end
loOcc=log(cfg.mapOccupiedProbability/(1-cfg.mapOccupiedProbability));
loFree=log(cfg.mapFreeProbability/(1-cfg.mapFreeProbability));
if isfield(map,'staticOccupied')
    mapOcc=map.staticOccupied;
else
    mapOcc=map.logOdds>=loOcc&map.hitCount>=cfg.mapMinOccupiedObservations;
end
mapFree=map.logOdds<=loFree&map.observationCount>=cfg.mapMinFreeObservations&~mapOcc;
observable=map.observationCount>0;
if isfield(map,'rawHitCount')
    occupiedObservable=truthOcc&map.rawHitCount>=cfg.mapMinOccupiedObservations;
else
    occupiedObservable=truthOcc&map.hitCount>=cfg.mapMinOccupiedObservations;
end
falseFree=nnz(mapFree&truthOcc);freeDeclared=max(1,nnz(mapFree));
trueOccupied=max(1,nnz(occupiedObservable));
recall=nnz(mapOcc&occupiedObservable)/trueOccupied;
metrics=struct('falseFreeRate',falseFree/freeDeclared,'occupiedRecall',recall, ...
    'observedFraction',nnz(observable)/numel(observable),'knownFreeCount',nnz(mapFree), ...
    'occupiedCount',nnz(mapOcc),'falseFreeCount',falseFree, ...
    'occupiedHitObservableCount',nnz(occupiedObservable), ...
    'pass',falseFree/freeDeclared<=cfg.mapMaxFalseFreeRate&&recall>=cfg.mapMinOccupiedRecall);
end

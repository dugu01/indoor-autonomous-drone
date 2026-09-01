function [state,frontiers,diag] = extract_frontiers_incremental_S2_4(c,state,grid)
% EXTRACT_FRONTIERS_INCREMENTAL_S2_4 Incremental, clustered frontier manager.
% A frontier is inherited known-free adjacent (4-neighbour) to inherited unknown.
arguments
    c (1,1) struct
    state struct
    grid (1,1) struct
end
% Frontier classification uses the authoritative raw S2.3 classes.  The
% execution mask (grid.occ) also contains unknown-space inflation and must not
% be applied here, otherwise every known-free cell adjacent to unknown would
% be removed before it could become a frontier.
known=logical(grid.knownFree)&~logical(grid.rawOccupied);unknown=logical(grid.unknown)&~logical(grid.rawOccupied);full=frontierPredicate(known,unknown);
if isempty(state)||isempty(fieldnames(state))
    state=struct('mask',full,'known',known,'unknown',unknown,'rawOccupied',logical(grid.rawOccupied), ...
        'tracks',struct([]),'nextTrackId',uint64(1),'updateIndex',uint32(0));dirty=true(size(full));inc=full;
else
    changed=state.known~=known|state.unknown~=unknown|state.rawOccupied~=logical(grid.rawOccupied);dirty=conv2(double(changed),ones(3),'same')>0;inc=state.mask;inc(dirty)=full(dirty);
end
if ~isequal(inc,full),error('S2_4:IncrementalFrontierMismatch','Incremental frontier mask differs from full extraction.');end
clusters=clusterAndSplit(inc,c.minFrontierCells,c.maxFrontierExtentCells);state.updateIndex=state.updateIndex+uint32(1);[tracks,frontiers,nextId]=associateTracks(state.tracks,clusters,state.nextTrackId,state.updateIndex);state.mask=inc;state.known=known;state.unknown=unknown;state.rawOccupied=logical(grid.rawOccupied);state.tracks=tracks;state.nextTrackId=nextId;diag=struct('dirtyCount',nnz(dirty),'frontierCellCount',nnz(full),'clusterCount',numel(clusters),'incrementalEqualsFull',true,'mapVersion',grid.mapVersion);
end

function f=frontierPredicate(known,unknown)
nb=conv2(double(unknown),[0 1 0;1 0 1;0 1 0],'same')>0;f=known&nb;
end
function clusters=clusterAndSplit(mask,minCells,maxExtent)
[ny,nx]=size(mask);seen=false(ny,nx);clusters={};nb=[-1 -1;-1 0;-1 1;0 -1;0 1;1 -1;1 0;1 1];
for y=1:ny
 for x=1:nx
  if ~mask(y,x)||seen(y,x),continue,end
  q=zeros(nnz(mask),2);head=1;tail=1;q(1,:)=[y x];seen(y,x)=true;cells=zeros(nnz(mask),2);n=0;
  while head<=tail
   p=q(head,:);head=head+1;n=n+1;cells(n,:)=p;
   for j=1:8,yy=p(1)+nb(j,1);xx=p(2)+nb(j,2);if yy>=1&&yy<=ny&&xx>=1&&xx<=nx&&mask(yy,xx)&&~seen(yy,xx),tail=tail+1;q(tail,:)=[yy xx];seen(yy,xx)=true;end,end
  end
  cells=cells(1:n,:);parts=splitRec(cells,maxExtent);for j=1:numel(parts),if size(parts{j},1)>=minCells,clusters{end+1}=sortrows(parts{j},[1 2]);end,end %#ok<AGROW>
 end
end
if ~isempty(clusters),keys=cellfun(@(z)sub2ind([ny nx],z(1,1),z(1,2)),clusters);[~,o]=sort(keys);clusters=clusters(o);end
end
function parts=splitRec(cells,maxExtent)
if isempty(cells)||max(max(cells,[],1)-min(cells,[],1))<=maxExtent,parts={cells};return,end
A=cells-mean(cells,1);[V,D]=eig(A.'*A);[~,k]=max(diag(D));axis=V(:,k);j=find(abs(axis)>1e-12,1);if ~isempty(j)&&axis(j)<0,axis=-axis;end;proj=A*axis;T=[proj cells];T=sortrows(T,[1 2 3]);m=floor(size(T,1)/2);if m<1||m>=size(T,1),parts={cells};else,parts=[splitRec(T(1:m,2:3),maxExtent),splitRec(T(m+1:end,2:3),maxExtent)];end
end
function [tracks,active,nextId]=associateTracks(old,clusters,nextId,updateIndex)
tracks=old;used=false(1,numel(old));active=repmat(emptyFrontier(),0,1);
for i=1:numel(clusters)
 cells=clusters{i};cent=mean(cells,1);best=0;bestKey=[inf inf inf];
 for j=1:numel(old)
  if used(j),continue,end;overlap=size(intersect(cells,old(j).cells,'rows'),1);dist=norm(cent-old(j).centroid);key=[-overlap dist double(old(j).trackId)];if lexless(key,bestKey),bestKey=key;best=j;end
 end
 if best>0&&(bestKey(1)<0||bestKey(2)<=2.5),f=old(best);used(best)=true;else,f=emptyFrontier();f.trackId=nextId;nextId=nextId+uint64(1);f.createdUpdate=updateIndex;end
 f.cells=cells;f.centroid=cent;f.geometryHash=hashCells(cells);f.updatedUpdate=updateIndex;if strcmp(f.state,'RESOLVED'),f.state='ACTIVE';end;tracks=replaceTrack(tracks,f);active(end+1)=f; %#ok<AGROW>
end
for j=1:numel(old),if ~used(j),k=find([tracks.trackId]==old(j).trackId,1);if ~isempty(k),tracks(k).state='RESOLVED';end,end,end
end
function out=replaceTrack(in,f),if isempty(in),out=f;return,end;k=find([in.trackId]==f.trackId,1);if isempty(k),out=[in f];else,in(k)=f;out=in;end,end
function f=emptyFrontier(),f=struct('trackId',uint64(0),'cells',zeros(0,2),'centroid',[nan nan],'geometryHash','','state','ACTIVE','failures',uint16(0),'blockedUntilUpdate',uint32(0),'createdUpdate',uint32(0),'updatedUpdate',uint32(0));end
function h=hashCells(c),v=uint64(c(:,1))*uint64(4294967291)+uint64(c(:,2));h=sprintf('%016x',mod(sum(v.*uint64((1:numel(v)).')),intmax('uint64')));end
function tf=lexless(a,b),k=find(a~=b,1);tf=~isempty(k)&&a(k)<b(k);end

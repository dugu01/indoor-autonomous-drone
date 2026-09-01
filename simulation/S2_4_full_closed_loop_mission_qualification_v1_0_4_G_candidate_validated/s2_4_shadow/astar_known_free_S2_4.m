function path = astar_known_free_S2_4(knownFree,startCell,goalCell)
% ASTAR_KNOWN_FREE_S2_4 Deterministic 8-connected A* on known-free cells only.
arguments
    knownFree (:,:) logical
    startCell (1,2) double
    goalCell (1,2) double
end
[ny,nx]=size(knownFree);startCell=round(startCell);goalCell=round(goalCell);path=zeros(0,2);
if any(startCell<1)||startCell(1)>ny||startCell(2)>nx||any(goalCell<1)||goalCell(1)>ny||goalCell(2)>nx||~knownFree(startCell(1),startCell(2))||~knownFree(goalCell(1),goalCell(2)),return,end
N=ny*nx;g=inf(N,1);parent=zeros(N,1,'uint32');closed=false(N,1);s=sub2ind([ny nx],startCell(1),startCell(2));goal=sub2ind([ny nx],goalCell(1),goalCell(2));g(s)=0;open=uint32(s);fval=h(startCell,goalCell);
nb=[-1 -1;-1 0;-1 1;0 -1;0 1;1 -1;1 0;1 1];
while ~isempty(open)
    [~,ord]=sortrows([fval(:),double(open(:))],[1 2]);k=ord(1);cur=double(open(k));open(k)=[];fval(k)=[];if closed(cur),continue,end;closed(cur)=true;if cur==goal,break,end
    [cy,cx]=ind2sub([ny nx],cur);
    for j=1:8
        yy=cy+nb(j,1);xx=cx+nb(j,2);if yy<1||yy>ny||xx<1||xx>nx||~knownFree(yy,xx),continue,end
        ni=sub2ind([ny nx],yy,xx);if closed(ni),continue,end;step=1+(nb(j,1)~=0&&nb(j,2)~=0)*(sqrt(2)-1);ng=g(cur)+step;
        if ng+1e-12<g(ni),g(ni)=ng;parent(ni)=uint32(cur);open(end+1)=uint32(ni);fval(end+1)=ng+h([yy xx],goalCell);end %#ok<AGROW>
    end
end
if ~isfinite(g(goal)),return,end
idx=goal;rev=zeros(N,2);n=0;while idx~=0,n=n+1;[yy,xx]=ind2sub([ny nx],idx);rev(n,:)=[yy xx];if idx==s,break,end;idx=double(parent(idx));end
path=flipud(rev(1:n,:));
end
function d=h(a,b),d=hypot(a(1)-b(1),a(2)-b(2));end

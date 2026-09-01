function [candidates,selected,diag] = generate_safe_viewpoints_S2_4(c,grid,frontiers,startXY,targetXY,uncertainty)
% GENERATE_SAFE_VIEWPOINTS_S2_4 Deterministic known-free shadow viewpoints.
% No output from this function is connected to the S2.3 mission manager.
arguments
    c (1,1) struct
    grid (1,1) struct
    frontiers struct
    startXY (1,2) double
    targetXY (1,2) double
    uncertainty struct = struct()
end
res=grid.resolution;start=xy2cell(grid,startXY);target=xy2cell(grid,targetXY);corridor=diagnosticCorridor(grid,start,target,c);candidates=repmat(emptyCandidate(),0,1);candidateId=uint64(1);
for i=1:numel(frontiers)
    f=frontiers(i);if ~strcmp(f.state,'ACTIVE'),continue,end
    proposed=zeros(0,2);
    for r=c.candidateRadii_m
        rc=r/res;for k=0:c.candidateAngles-1,a=2*pi*k/c.candidateAngles;proposed(end+1,:)=[round(f.centroid(1)+rc*sin(a)) round(f.centroid(2)+rc*cos(a))];end %#ok<AGROW>
    end
    stride=max(1,floor(size(f.cells,1)/8));proposed=[proposed;f.cells(1:stride:end,:)];proposed=unique(proposed,'rows','sorted');
    for j=1:size(proposed,1)
        q=emptyCandidate();q.candidateId=candidateId;q.frontierTrackId=f.trackId;q.cell=proposed(j,:);candidateId=candidateId+uint64(1);reasons={};
        if ~inside(grid,q.cell),reasons{end+1}='OUTSIDE_MAP';elseif grid.occ(q.cell(1),q.cell(2)),reasons{end+1}='POSITION_OCCUPIED_OR_UNKNOWN_INFLATED';elseif ~grid.knownFree(q.cell(1),q.cell(2)),reasons{end+1}='POSITION_UNKNOWN';end
        if isempty(reasons)
            q.path=astar_known_free_S2_4(logical(grid.knownFree)&~logical(grid.occ),start,q.cell);if isempty(q.path),reasons{end+1}='UNREACHABLE_KNOWN_FREE';end
            if ~isempty(q.path)&&isfield(grid,'lastObservedXY')&&isfinite(grid.timestamp),age=grid.timestamp-double(grid.lastObservedXY);idx=sub2ind(size(age),q.path(:,1),q.path(:,2));if any(age(idx)>c.staleFreeAge_s),reasons{end+1}='STALE_ROUTE_REQUIRES_RESCAN';end,end
            if ~holdSupport(grid,q.cell),reasons{end+1}='STOPPING_SUPPORT_INVALID';end
            if isempty(q.path)||size(q.path,1)<2,reasons{end+1}='RETREAT_ROUTE_INVALID';end
        end
        q.yaw=atan2(target(1)-q.cell(1),target(2)-q.cell(2));if isempty(reasons)||all(~ismember(reasons,{'OUTSIDE_MAP','POSITION_OCCUPIED_OR_UNKNOWN_INFLATED','POSITION_UNKNOWN'}))
            q.visibleUnknown=visibleUnknown(grid,q.cell,q.yaw,c);if size(q.visibleUnknown,1)<c.minVisibleUnknownCells,reasons{end+1}='INSUFFICIENT_VISIBLE_UNKNOWN';end
        end
        if ~isempty(q.path)&&isfield(grid,'dynamicOccupied'),idx=sub2ind(size(grid.dynamicOccupied),q.path(:,1),q.path(:,2));q.dynamicRisk=double(any(grid.dynamicOccupied(idx)));if q.dynamicRisk>=0.8,reasons{end+1}='DYNAMIC_ROUTE_CROSSING';end,end
        [q.informationGain,q.targetRelevance]=gainTerms(q.visibleUnknown,corridor,uncertainty,grid);[q.tier,progress]=tierForFrontier(f,start,target,q.targetRelevance,q.informationGain);if q.tier==3,reasons{end+1}='IRRELEVANT_EXPLORATION';end
        q.travelCost=pathLength(q.path,res);q.staticRisk=double(~holdSupport(grid,q.cell));q.uncertaintyRisk=pathUncertainty(q.path,uncertainty,grid);q.yawCost=abs(wrapPi(q.yaw))/pi;q.stopRetreatRisk=double(isempty(q.path)||size(q.path,1)<2||~holdSupport(grid,q.cell));q.utility=score(c,q,max(1,nnz(grid.unknown)),max(1,size(corridor,1)));q.rejectionReasons=unique(reasons,'stable');q.accepted=isempty(q.rejectionReasons);q.progressCertificate=progress;if q.accepted,q.action='FLY_AND_SCAN_SHADOW';elseif any(strcmp(q.rejectionReasons,'STALE_ROUTE_REQUIRES_RESCAN')),q.action='HOLD_AND_RESCAN_SHADOW';else,q.action='REJECT';end;candidates(end+1)=q; %#ok<AGROW>
    end
end
selected=selectCandidate(candidates);diag=struct('candidateCount',numel(candidates),'acceptedCount',nnz([candidates.accepted]),'selectedCandidateId',uint64(0),'commandIssued',false);if ~isempty(selected),diag.selectedCandidateId=selected.candidateId;end
end

function [tier,ok]=tierForFrontier(f,start,target,relevance,information)
tv=double(target-start);fv=double(f.centroid-start);den=norm(tv)*norm(fv);if den>eps,alignment=dot(tv,fv)/den;else,alignment=-1;end;projected=dot(fv,tv)/max(norm(tv),eps);ok=alignment>=0.80&&projected>=2;tier=3;if relevance>0,tier=1;elseif information>0&&ok,tier=2;end
end
function [I,G]=gainTerms(vis,corridor,u,grid)
if isempty(vis),I=0;G=0;return,end;idx=sub2ind(size(grid.unknown),vis(:,1),vis(:,2));if isfield(u,'entropy')&&ndims(u.entropy)==2,H=double(u.entropy(idx));else,H=ones(size(idx));end;I=sum(H);if isempty(corridor),G=0;else,G=sum(H(ismember(vis,corridor,'rows')));end
end
function v=score(c,q,maxI,maxG),v=c.weights.information*min(q.informationGain/maxI,1)+c.weights.target*min(q.targetRelevance/maxG,1)-c.weights.travel*min(q.travelCost/10,1)-c.weights.staticRisk*q.staticRisk-c.weights.dynamicRisk*q.dynamicRisk-c.weights.uncertaintyRisk*q.uncertaintyRisk-c.weights.yaw*q.yawCost-c.weights.stopRetreat*q.stopRetreatRisk;end
function s=selectCandidate(cands),s=[];if isempty(cands),return,end;v=find([cands.accepted]);if isempty(v),return,end;keys=zeros(numel(v),9);for i=1:numel(v),q=cands(v(i));keys(i,:)=[q.tier,-q.utility,-q.targetRelevance,-q.informationGain,q.dynamicRisk,q.staticRisk,q.travelCost,double(q.frontierTrackId),double(q.candidateId)];end;[~,o]=sortrows(keys,1:9);s=cands(v(o(1)));end
function tf=holdSupport(grid,c),tf=false;if ~inside(grid,c),return,end;y=max(1,c(1)-1):min(grid.ny,c(1)+1);x=max(1,c(2)-1):min(grid.nx,c(2)+1);tf=all(grid.knownFree(y,x)&~grid.occ(y,x),'all');end
function vis=visibleUnknown(grid,o,yaw,c),vis=zeros(0,2);angles=linspace(-pi,pi,121);maxStep=floor(6.5/grid.resolution);seen=false(grid.ny,grid.nx);for a=angles,aa=yaw+a;for k=1:maxStep,p=[round(o(1)+k*sin(aa)) round(o(2)+k*cos(aa))];if ~inside(grid,p),break,end;if grid.staticOccupiedRaw(p(1),p(2))||grid.dynamicOccupiedRaw(p(1),p(2)),break,end;if grid.unknown(p(1),p(2)),seen(p(1),p(2))=true;break,end,end,end;[y,x]=find(seen);vis=[y x];end
function corridor=diagnosticCorridor(grid,start,target,c),p=diagnosticAstar(grid,start,target,c.diagnosticUnknownCost);core=p(grid.unknown(sub2ind(size(grid.unknown),p(:,1),p(:,2))),:);mask=false(grid.ny,grid.nx);for i=1:size(core,1),for dy=-c.targetCorridorRadiusCells:c.targetCorridorRadiusCells,for dx=-c.targetCorridorRadiusCells:c.targetCorridorRadiusCells,if dy^2+dx^2<=c.targetCorridorRadiusCells^2,q=core(i,:)+[dy dx];if inside(grid,q)&&grid.unknown(q(1),q(2)),mask(q(1),q(2))=true;end,end,end,end,end;[y,x]=find(mask);corridor=[y x];end
function path=diagnosticAstar(grid,start,target,unknownCost),allowed=grid.knownFree|grid.unknown;blocked=grid.staticOccupied|grid.dynamicOccupied;if ~inside(grid,start)||~inside(grid,target)||blocked(start(1),start(2))||blocked(target(1),target(2)),path=zeros(0,2);return,end;[ny,nx]=size(allowed);N=ny*nx;g=inf(N,1);par=zeros(N,1,'uint32');closed=false(N,1);s=sub2ind([ny nx],start(1),start(2));goal=sub2ind([ny nx],target(1),target(2));g(s)=0;open=uint32(s);fv=h(start,target);nb=[-1 -1;-1 0;-1 1;0 -1;0 1;1 -1;1 0;1 1];while ~isempty(open),[~,k]=min(fv);cur=double(open(k));open(k)=[];fv(k)=[];if closed(cur),continue,end;closed(cur)=true;if cur==goal,break,end;[cy,cx]=ind2sub([ny nx],cur);for j=1:8,yy=cy+nb(j,1);xx=cx+nb(j,2);if yy<1||yy>ny||xx<1||xx>nx||~allowed(yy,xx)||blocked(yy,xx),continue,end;ni=sub2ind([ny nx],yy,xx);step=1+(nb(j,1)~=0&&nb(j,2)~=0)*(sqrt(2)-1);if grid.unknown(yy,xx),step=step*unknownCost;end;ng=g(cur)+step;if ng<g(ni),g(ni)=ng;par(ni)=uint32(cur);open(end+1)=uint32(ni);fv(end+1)=ng+h([yy xx],target);end,end,end;if ~isfinite(g(goal)),path=zeros(0,2);return,end;idx=goal;rev=zeros(N,2);n=0;while idx~=0,n=n+1;[yy,xx]=ind2sub([ny nx],idx);rev(n,:)=[yy xx];if idx==s,break,end;idx=double(par(idx));end;path=flipud(rev(1:n,:));end
function r=pathUncertainty(path,u,grid),if isempty(path),r=1;return,end;if isfield(u,'entropy')&&isequal(size(u.entropy),size(grid.knownFree)),idx=sub2ind(size(grid.knownFree),path(:,1),path(:,2));r=mean(double(u.entropy(idx)));else,r=0;end,end
function d=pathLength(p,res),d=0;for i=2:size(p,1),d=d+res*norm(p(i,:)-p(i-1,:));end,end
function c=xy2cell(grid,xy),c=[round((xy(2)-grid.ys(1))/grid.resolution)+1 round((xy(1)-grid.xs(1))/grid.resolution)+1];end
function tf=inside(grid,c),tf=numel(c)==2&&all(c>=1)&&c(1)<=grid.ny&&c(2)<=grid.nx;end
function d=h(a,b),d=hypot(a(1)-b(1),a(2)-b(2));end
function a=wrapPi(a),a=mod(a+pi,2*pi)-pi;end
function q=emptyCandidate(),q=struct('candidateId',uint64(0),'frontierTrackId',uint64(0),'cell',[nan nan],'yaw',0,'path',zeros(0,2),'visibleUnknown',zeros(0,2),'informationGain',0,'targetRelevance',0,'travelCost',0,'staticRisk',0,'dynamicRisk',0,'uncertaintyRisk',0,'yawCost',0,'stopRetreatRisk',0,'tier',uint8(3),'utility',-inf,'progressCertificate',false,'accepted',false,'rejectionReasons',{{}},'action','NONE');end

import math, numpy as np
RES=.1
CFG=dict(maxSpeed=.32,maxAccel=.65,maxJerk=2.20,nomFrac=.68,intFrac=.58,minSeg=.90,maxIter=12,scaleSafety=1.03,sampleDt=.02,maxDecel=.80,delay=.30,stopMargin=.08,landingRadius=.402)

def cell_occ(occ,cx,cy):
 iy=cy;ix=cx
 return ix<0 or ix>=occ.shape[1] or iy<0 or iy>=occ.shape[0] or bool(occ[iy,ix])

def segment_occupied(occ,a,b,res=.1):
 a=np.asarray(a,float);b=np.asarray(b,float)
 qa=a/res+.5;qb=b/res+.5;cx=int(math.floor(qa[0]));cy=int(math.floor(qa[1]));ex=int(math.floor(qb[0]));ey=int(math.floor(qb[1]))
 if cell_occ(occ,cx,cy):return True
 dx=qb[0]-qa[0];dy=qb[1]-qa[1];sx=0 if dx==0 else (1 if dx>0 else -1);sy=0 if dy==0 else (1 if dy>0 else -1)
 if sx:
  nb=cx+1 if sx>0 else cx;tMx=(nb-qa[0])/dx;tDx=1/abs(dx)
 else:tMx=tDx=float('inf')
 if sy:
  nb=cy+1 if sy>0 else cy;tMy=(nb-qa[1])/dy;tDy=1/abs(dy)
 else:tMy=tDy=float('inf')
 tol=1e-12;cnt=0;maxv=4*(occ.shape[0]+occ.shape[1])+20
 while cx!=ex or cy!=ey:
  if tMx<tMy-tol:
   cx+=sx;tMx+=tDx
   if cell_occ(occ,cx,cy):return True
  elif tMy<tMx-tol:
   cy+=sy;tMy+=tDy
   if cell_occ(occ,cx,cy):return True
  else:
   nx=cx+sx;ny=cy+sy
   if sx and cell_occ(occ,nx,cy):return True
   if sy and cell_occ(occ,cx,ny):return True
   cx=nx;cy=ny;tMx+=tDx;tMy+=tDy
   if cell_occ(occ,cx,cy):return True
  cnt+=1
  if cnt>maxv:return True
 return False

def smooth_path(occ,path):
 if len(path)==0:return []
 out=[np.asarray(path[0],float)];i=0
 while i<len(path)-1:
  j=len(path)-1
  while j>i+1 and segment_occupied(occ,path[i],path[j]):j-=1
  out.append(np.asarray(path[j],float));i=j
 return np.array(out)

def remove_dups(path,tol=.025):
 out=[path[0]]
 for p in path[1:]:
  if np.linalg.norm(np.asarray(p)-np.asarray(out[-1]))>=tol:out.append(p)
 return np.array(out,float)

def eval_seg(C,t,d):
 t=np.atleast_1d(t).astype(float);v=np.zeros((len(t),2))
 for power in range(d,8):
  scale=math.factorial(power)/math.factorial(power-d)
  v += t[:,None]**(power-d)*(scale*C[power][None,:])
 return v

def solve_seg(p0,p1,v0,v1,a0,a1,j0,j1,T):
 M=np.zeros((8,8))
 for d in range(4):
  M[d,d]=math.factorial(d)
  for power in range(d,8):M[4+d,power]=math.factorial(power)/math.factorial(power-d)*T**(power-d)
 B=np.vstack([p0,v0,a0,j0,p1,v1,a1,j1])
 return np.linalg.solve(M,B)

def waypoint_derivatives(path,durs,startV,startA,zero,cfg=CFG):
 n=len(path);V=np.zeros((n,2));A=np.zeros((n,2));J=np.zeros((n,2));V[0]=startV;A[0]=startA
 for i in range(1,n-1):
  if zero:continue
  chord=path[i+1]-path[i-1];nc=np.linalg.norm(chord)
  if nc<1e-12:continue
  speed=min(cfg['intFrac']*cfg['maxSpeed'],.45*(np.linalg.norm(path[i]-path[i-1])/durs[i-1]+np.linalg.norm(path[i+1]-path[i])/durs[i]))
  V[i]=speed*chord/nc
 return V,A,J

def generate(occ,path,startV=(0,0),startA=(0,0),initial=1.,cfg=CFG):
 path=remove_dups(np.asarray(path,float))
 if len(path)<2:return None
 L=np.linalg.norm(np.diff(path,axis=0),axis=1)
 base=np.maximum.reduce([L/(cfg['nomFrac']*cfg['maxSpeed']),np.sqrt(np.maximum(L,1e-9)/(.18*cfg['maxAccel'])),np.full_like(L,cfg['minSeg'])])
 requested=np.maximum(base*initial,np.finfo(float).eps)
 for fallback in [0,1]:
  if fallback==0:durs=base*initial
  else:durs=np.maximum.reduce([L/(.55*cfg['maxSpeed']),np.sqrt(np.maximum(L,1e-9)/(.12*cfg['maxAccel'])),np.full_like(L,.90)])*initial
  for iteration in range(cfg['maxIter']):
   V,A,J=waypoint_derivatives(path,durs,np.asarray(startV),np.asarray(startA),fallback==1,cfg)
   Cs=np.array([solve_seg(path[i],path[i+1],V[i],V[i+1],A[i],A[i+1],J[i],J[i+1],durs[i]) for i in range(len(durs))])
   ps=[];vs=[];acs=[];js=[]
   for i,T in enumerate(durs):
    ts=np.arange(0,T+1e-12,cfg['sampleDt'])
    if len(ts)==0 or ts[-1]<T-1e-12:ts=np.r_[ts,T]
    if i>0:ts=ts[1:]
    ps.append(eval_seg(Cs[i],ts,0));vs.append(eval_seg(Cs[i],ts,1));acs.append(eval_seg(Cs[i],ts,2));js.append(eval_seg(Cs[i],ts,3))
   P=np.vstack(ps);Vv=np.vstack(vs);Aa=np.vstack(acs);Jj=np.vstack(js)
   mxv=np.linalg.norm(Vv,axis=1).max();mxa=np.linalg.norm(Aa,axis=1).max();mxj=np.linalg.norm(Jj,axis=1).max()
   collision=False
   for i in range(len(P)-1):
    if segment_occupied(occ,P[i],P[i+1]):collision=True;break
   ratio=max(mxv/cfg['maxSpeed'],math.sqrt(mxa/cfg['maxAccel']),np.cbrt(mxj/cfg['maxJerk']),1.)
   if ratio<=1.0005 and not collision:return dict(valid=True,path=path,durations=durs,P=P,maxSpeed=mxv,maxAccel=mxa,maxJerk=mxj,fallback=fallback,timeScale=max(durs/requested))
   if collision:break
   durs=durs*(cfg['scaleSafety']*ratio)
 return None

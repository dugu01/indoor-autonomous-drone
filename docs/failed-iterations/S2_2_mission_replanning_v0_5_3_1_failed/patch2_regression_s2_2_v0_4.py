"""Focused regression for S2.2 v0.4 Patch 2.

Checks the two remaining MATLAB failure mechanisms:
1. velocity-mode REJOIN plus executed-speed envelope;
2. brake-before-replan transition from a moving state.
This is not a replacement for the seven-scenario MATLAB validator.
"""
from __future__ import annotations
import math
import numpy as np

DT=0.02
MASS=1.5
I=np.diag([0.030,0.030,0.055])
G=np.array([0.,0.,-9.81])


def qmul(q,r):
    w,x,y,z=q; a,b,c,d=r
    o=np.array([w*a-x*b-y*c-z*d,w*b+x*a+y*d-z*c,w*c-x*d+y*a+z*b,w*d+x*c-y*b+z*a])
    return o/np.linalg.norm(o)

def qexp(rv):
    th=np.linalg.norm(rv)
    if th<1e-12:return np.array([1.,*(0.5*rv)])
    return np.array([math.cos(th/2),*(math.sin(th/2)*rv/th)])

def q2r(q):
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def shape(v,a,target):
    raw=(target-v)/DT
    n=np.linalg.norm(raw)
    if n>0.55: raw*=0.55/n
    da=raw-a; n=np.linalg.norm(da)
    if n>2.0*DT: da*=2.0*DT/n
    a=a+da; n=np.linalg.norm(a)
    if n>0.55:a*=0.55/n
    v=v+a*DT; n=np.linalg.norm(v)
    if n>0.32:v*=0.32/n
    return v,a

def guard(v,a):
    speed=np.linalg.norm(v)
    if speed<1e-8 or speed<0.34:return a
    d=v/speed; tang=a@d
    allowed=min(0.65,2.5*(0.45-speed))
    if speed>=0.45:allowed=min(allowed,-0.30)
    if tang>allowed:a=a+(allowed-tang)*d
    return a

def step(state, ref_p, ref_v, ref_a, prev_cmd_a):
    p,v,q,w,a_prev=state
    ep=ref_p-p; ev=ref_v-v
    a_cmd=ref_a+np.array([2.,2.,3.2])*ep+np.array([1.5,1.5,2.])*ev
    h=np.linalg.norm(a_cmd[:2])
    if h>0.65:a_cmd[:2]*=0.65/h
    a_cmd[2]=np.clip(a_cmd[2],-1.5,1.5)
    a_cmd[:2]=guard(v[:2],a_cmd[:2])
    da=a_cmd-prev_cmd_a; n=np.linalg.norm(da)
    if n>6*DT:a_cmd=prev_cmd_a+da*(6*DT/n)
    h=np.linalg.norm(a_cmd[:2])
    if h>0.65:a_cmd[:2]*=0.65/h
    a_cmd[:2]=guard(v[:2],a_cmd[:2])
    F=MASS*(a_cmd-G); b3=F/np.linalg.norm(F); b1c=np.array([1.,0.,0.])
    b2=np.cross(b3,b1c)
    if np.linalg.norm(b2)<1e-6:b2=np.array([0.,1.,0.])
    b2/=np.linalg.norm(b2); rd=np.column_stack((np.cross(b2,b3),b2,b3))
    R=q2r(q); E=rd.T@R-R.T@rd; eR=.5*np.array([E[2,1],E[0,2],E[1,0]])
    mom=-np.array([5.,5.,2.2])*eR-np.array([.55,.55,.25])*w+np.cross(w,I@w)
    mom=np.clip(mom,[-.9,-.9,-.35],[.9,.9,.35])
    thrust=np.clip(F@R[:,2],0,2.4*MASS*9.81)
    acc=R@np.array([0.,0.,thrust])/MASS+G-.1*v
    wdot=np.linalg.solve(I,mom-np.cross(w,I@w)-.02*w)
    p=p+v*DT+.5*acc*DT*DT;v=v+acc*DT
    q=qmul(q,qexp((w+.5*wdot*DT)*DT));w=w+wdot*DT
    return (p,v,q,w,acc),a_cmd

def main():
    # 8 s velocity-mode REJOIN, then TRACK with only 0.30 m position error.
    state=(np.array([0.,0.,1.15]),np.array([0.30,0.,0.]),np.array([1.,0.,0.,0.]),np.zeros(3),np.zeros(3))
    prev=np.zeros(3);vc=np.array([0.30,0.]);ac=np.zeros(2);max_speed=0.;max_jerk=0.;last_a=np.zeros(3)
    for k in range(1200):
        if k<400:
            target=np.array([0.32,0.0]);vc,ac=shape(vc,ac,target)
            # REJOIN is velocity mode: XY position anchored to current state.
            ref_p=np.array([state[0][0],state[0][1],1.15]);ref_v=np.r_[vc,0.];ref_a=np.r_[ac,0.]
        else:
            # TRACK resumes only near the reference. Model a moving reference
            # initially 0.30 m ahead at 0.31 m/s.
            elapsed=(k-400)*DT
            ref_p=np.array([state[0][0]+max(0.,0.30-0.08*elapsed),0.,1.15])
            ref_v=np.array([0.31,0.,0.]);ref_a=np.zeros(3)
        state,prev=step(state,ref_p,ref_v,ref_a,prev)
        speed=np.linalg.norm(state[1][:2]);jerk=np.linalg.norm((state[4]-last_a)[:2])/DT;last_a=state[4].copy()
        max_speed=max(max_speed,speed);max_jerk=max(max_jerk,jerk)
    assert max_speed<=0.45+1e-9,(max_speed,max_jerk)

    # Brake-before-replan from 0.29 m/s. Verify near-hover eligibility before
    # the 4 s timeout without increasing speed.
    state=(np.array([0.,0.,1.15]),np.array([0.29,0.02,0.]),np.array([1.,0.,0.,0.]),np.zeros(3),np.zeros(3))
    prev=np.zeros(3);vc=state[1][:2].copy();ac=np.zeros(2);eligible=None;peak=np.linalg.norm(vc)
    filt_acc=np.zeros(2);old_v=state[1][:2].copy()
    for k in range(int(4/DT)):
        vc,ac=shape(vc,ac,np.zeros(2))
        ref_p=np.array([state[0][0],state[0][1],1.15]);ref_v=np.r_[vc,0.];ref_a=np.r_[ac,0.]
        state,prev=step(state,ref_p,ref_v,ref_a,prev)
        raw=(state[1][:2]-old_v)/DT;old_v=state[1][:2].copy();filt_acc=.92*filt_acc+.08*raw
        peak=max(peak,np.linalg.norm(state[1][:2]))
        if np.linalg.norm(state[1][:2])<=.05 and np.linalg.norm(filt_acc)<=.12:
            eligible=(k+1)*DT;break
    assert eligible is not None and eligible<4.0,eligible
    assert peak<=0.45,peak
    print('PATCH 2 REGRESSION: PASS')
    print(f'rejoin max executed speed: {max_speed:.4f} m/s; max jerk {max_jerk:.4f} m/s^3')
    print(f'brake-before-replan eligible: {eligible:.2f} s; peak speed {peak:.4f} m/s')

if __name__=='__main__':main()

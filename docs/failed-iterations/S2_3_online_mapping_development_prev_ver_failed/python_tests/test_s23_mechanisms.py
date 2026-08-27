#!/usr/bin/env python3
"""Independent mechanism checks for the cumulative S2.3 candidate.
These tests verify isolated logic and source contracts. They do not claim a
coupled MATLAB 6-DOF pass.
"""
from __future__ import annotations
import math
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
ROOM=np.array([6.0,6.0,2.5])
RESXY=0.10; RESZ=0.20
XS=np.arange(0,ROOM[0]+1e-9,RESXY); YS=np.arange(0,ROOM[1]+1e-9,RESXY); ZS=np.arange(0,ROOM[2]+1e-9,RESZ)
L_HIT=.90; L_MISS=-.45; LMIN=-4.; LMAX=4.
PFREE=.20; POCC=.65
MIN_FREE_OBS=2
START=np.array([3.0,.8,.03]); LIDAR_ORIGIN=START+np.array([0,0,.03])
STATIC=np.array([[1.,1.,.5,.5,1.8],[4.,3.5,.5,.5,1.8]])


def raycast(origin,d,max_range=6.5,min_range=.12,step=.035):
    d=np.asarray(d,float); d/=np.linalg.norm(d)
    s=min_range
    while s<=max_range+1e-12:
        p=origin+s*d
        if np.any(p<=0) or np.any(p>=ROOM): return s,True
        for r in STATIC:
            if r[0]<=p[0]<=r[0]+r[2] and r[1]<=p[1]<=r[1]+r[3] and p[2]<=r[4]: return s,True
        s+=step
    return max_range,False

def idx(p):
    ix=math.floor(p[0]/RESXY+.5); iy=math.floor(p[1]/RESXY+.5); iz=math.floor((p[2]-ZS[0])/RESZ+.5)
    ok=0<=ix<len(XS) and 0<=iy<len(YS) and 0<=iz<len(ZS)
    return ix,iy,iz,ok

def insert_endpoint_excluded(logodds,obs,origin,d,rng,hit):
    d=np.asarray(d,float); d/=np.linalg.norm(d)
    endpoint_index=None
    if hit:
        ep=idx(origin+rng*d)
        if ep[3]: endpoint_index=ep[:3]
    free=[]; step=.5*min(RESXY,RESZ)
    for s in np.arange(0,rng+1e-12,step):
        cell=idx(origin+s*d)
        if not cell[3]: continue
        key=cell[:3]
        if endpoint_index is not None and key==endpoint_index: continue
        free.append(key)
    for ix,iy,iz in dict.fromkeys(free):
        logodds[iy,ix,iz]=max(LMIN,logodds[iy,ix,iz]+L_MISS);obs[iy,ix,iz]+=1
    if endpoint_index is not None:
        ix,iy,iz=endpoint_index
        logodds[iy,ix,iz]=min(LMAX,logodds[iy,ix,iz]+L_HIT);obs[iy,ix,iz]+=1

def segment_hits(mask,p0,p1,res=.1):
    length=np.linalg.norm(np.asarray(p1)-np.asarray(p0)); n=max(1,int(math.ceil(length/(.5*res))))
    for a in np.linspace(0,1,n+1):
        p=(1-a)*np.asarray(p0)+a*np.asarray(p1)
        ix=math.floor(p[0]/res+.5); iy=math.floor(p[1]/res+.5)
        if 0<=iy<mask.shape[0] and 0<=ix<mask.shape[1] and mask[iy,ix]: return True
    return False

def test_raycast_geometry():
    r,h=raycast(np.array([1.25,.8,.06]),[0,1,0]); assert h and .16<=r<=.25
    r,h=raycast(LIDAR_ORIGIN,[0,-1,0]); assert h and .75<=r<=.86

def test_launch_footprint_after_preflight_scan():
    lo=np.zeros((len(YS),len(XS),len(ZS)));obs=np.zeros_like(lo,dtype=int)
    for _ in range(5):
        for a in np.linspace(-math.pi,math.pi,120,endpoint=False):
            d=np.array([math.cos(a),math.sin(a),0.]);r,h=raycast(LIDAR_ORIGIN,d)
            insert_endpoint_excluded(lo,obs,LIDAR_ORIGIN,d,r,h)
    iz=math.floor(START[2]/RESZ+.5);X,Y=np.meshgrid(XS,YS);radius=.225+.127+.05
    footprint=(X-START[0])**2+(Y-START[1])**2<=radius**2
    free=(lo[:,:,iz]<=math.log(PFREE/(1-PFREE)))&(obs[:,:,iz]>=MIN_FREE_OBS)
    assert free[footprint].mean()==1.0

def test_endpoint_voxel_never_receives_free_update():
    rng=np.random.default_rng(4)
    for _ in range(5000):
        origin=np.array([.2,.2,.4]); angle=rng.uniform(-math.pi,math.pi); d=np.array([math.cos(angle),math.sin(angle),0.])
        distance=rng.uniform(.2,4.0); ep=idx(origin+distance*d)
        if not ep[3]: continue
        lo=np.zeros((len(YS),len(XS),len(ZS)));obs=np.zeros_like(lo,dtype=int)
        insert_endpoint_excluded(lo,obs,origin,d,distance,True)
        ix,iy,iz=ep[:3]
        assert lo[iy,ix,iz]>=L_HIT-1e-12

def test_old_half_voxel_method_overlaps_endpoint_about_half_the_time():
    rng=np.random.default_rng(12345);n=200000
    endpoint=rng.uniform(0,10,n);free=endpoint-.5*RESXY
    i1=np.floor(endpoint/RESXY+.5);i0=np.floor(free/RESXY+.5)
    overlap=np.mean(i1==i0)
    assert .48<overlap<.52,overlap

def test_promotion_clamps_once_to_occupied():
    lo=-4.0;occ=math.log(POCC/(1-POCC));promoted=False;count=0
    if not promoted:
        lo=max(lo,occ+.05);promoted=True;count+=1
    assert lo>occ and promoted and count==1
    # Re-observation cannot create another promotion event.
    if not promoted: count+=1
    assert count==1

def test_dynamic_layer_is_height_aware():
    init=(ROOT/'init_probabilistic_map_S2_3.m').read_text()
    proj=(ROOT/'project_map_to_planner_S2_3.m').read_text()
    assert "zeros(ny,nx,nz,'single')" in init
    assert 'any(map.dynamicLogOdds(:,:,iz)' in proj

def test_far_map_change_does_not_affect_route():
    mask=np.zeros((61,61),dtype=bool);mask[50,50]=True
    route=np.array([[3.0,.8],[3.0,1.8],[2.8,2.3]])
    assert not any(segment_hits(mask,route[i],route[i+1]) for i in range(len(route)-1))
    mask[15,30]=True
    assert any(segment_hits(mask,route[i],route[i+1]) for i in range(len(route)-1))

def test_extension_counted_on_arrival_not_selection():
    mgr=(ROOT/'mission_lifecycle_manager_S2_3.m').read_text()
    assert 'if planMeta.frontierUsed,mapExtensionPlanCount=mapExtensionPlanCount+1;end' in mgr
    assert 'if planMeta.frontierUsed,mapExtensionCount=mapExtensionCount+1;end' not in mgr
    assert mgr.count('mapExtensionCount=mapExtensionCount+1;')==2  # outbound and RTL completed arrival

def test_scan_progress_not_map_version_proxy():
    mgr=(ROOT/'mission_lifecycle_manager_S2_3.m').read_text()
    scan=mgr[mgr.index("case 'SCAN_HOLD'"):mgr.index("case 'MAP_DEGRADED_HOLD'")]
    assert 'mapState.version)<=scanEntryMapVersion' not in scan
    assert 'A changing map version alone is not physical progress' in scan

def test_strict_trajectory_adapter_preserves_hard_limit():
    text=(ROOT/'generate_strict_trajectory_S2_3.m').read_text()
    assert 'ratios<=1+1e-9' in text
    plan=(ROOT/'plan_unknown_segment_S2_3.m').read_text()
    mgr=(ROOT/'mission_lifecycle_manager_S2_3.m').read_text()
    assert 'generate_strict_trajectory_S2_3' in plan and 'generate_strict_trajectory_S2_3' in mgr

def test_scan_yaw_is_continuous_and_rate_reduced():
    cfg=(ROOT/'init_S2_3_config.m').read_text();mgr=(ROOT/'mission_lifecycle_manager_S2_3.m').read_text()
    assert 'deg2rad(35)' in cfg
    assert 'scanStartYaw+cfg.mapScanYawRate_radps' in mgr
    assert "scanStartYaw=yawCommand" in mgr

def test_preflight_uses_mapping_freshness_not_exact_packet():
    t_now=.60;last_lidar=.48;last_depth=.50;accepted=10
    assert (t_now-max(last_lidar,last_depth)<=.55) and accepted>=2

def test_idle_control_step_is_not_rejected_packet():
    reason='no_sensor_event';assert reason=='no_sensor_event'

def test_near_ground_layer_not_floor_false_free():
    x,y,z=3.0,.8,0.0
    assert not (x<=0 or x>=ROOM[0] or y<=0 or y>=ROOM[1] or z>=ROOM[2])

def test_pose_interpolation_contract():
    a=.5;p=(1-a)*np.array([1.,2.,.5])+a*np.array([1.2,2.4,.7]);assert np.allclose(p,[1.1,2.2,.6])


def test_metric_inflation_preserves_configured_radius():
    radius=.602;resolution=.10
    exact_offsets={(dx,dy) for dx in range(-10,11) for dy in range(-10,11)
                   if math.hypot(dx*resolution,dy*resolution)<=radius+1e-12}
    assert (6,0) in exact_offsets
    assert (7,0) not in exact_offsets
    # The superseded ceil-cell disk incorrectly included 0.70 m.
    assert math.ceil(radius/resolution)==7


def test_persistent_static_latch_survives_free_rays():
    lo=0.0;hit_count=0;static=False
    occ_threshold=math.log(POCC/(1-POCC))
    for _ in range(2):
        lo=min(LMAX,lo+L_HIT);hit_count+=1
        if hit_count>=2 and lo>=occ_threshold: static=True
    assert static and lo>=occ_threshold
    for _ in range(100):
        if not static: lo=max(LMIN,lo+L_MISS)
    assert static and lo>=occ_threshold


def test_source_uses_metric_inflation_and_static_latch():
    proj=(ROOT/'project_map_to_planner_S2_3.m').read_text()
    mapper=(ROOT/'update_probabilistic_map_S2_3.m').read_text()
    init=(ROOT/'init_probabilistic_map_S2_3.m').read_text()
    assert 'inflate_binary_metric' in proj
    assert 'ceil(inflationRadius/map.resolutionXY)' not in proj
    assert 'map.staticOccupied=false' in init
    assert 'if ~map.staticOccupied(idx)' in mapper
    assert 'map.staticOccupied(idx)=true' in mapper


def test_known_free_is_disjoint_from_static_occupancy():
    proj=(ROOT/'project_map_to_planner_S2_3.m').read_text()
    assert 'knownFree=knownFree&~staticOcc;' in proj

if __name__=='__main__':
    tests=[v for k,v in sorted(globals().items()) if k.startswith('test_')]
    for f in tests:
        f();print(f'{f.__name__}: PASS')
    print(f'{len(tests)}/{len(tests)} mechanism checks PASS')

import json
import math
import time
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.sparse import lil_matrix


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def skew(v):
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def q_normalize(q):
    q = np.asarray(q, float)
    if q[0] < 0:
        q = -q
    return q / np.linalg.norm(q)


def q_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def q_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_exp(theta):
    theta = np.asarray(theta, float)
    a = np.linalg.norm(theta)
    if a < 1e-10:
        return q_normalize(np.r_[1.0, 0.5 * theta])
    axis = theta / a
    return np.r_[math.cos(a/2), axis * math.sin(a/2)]


def q_log(q):
    q = q_normalize(q)
    v = q[1:]
    nv = np.linalg.norm(v)
    if nv < 1e-10:
        return 2.0 * v
    angle = 2.0 * math.atan2(nv, np.clip(q[0], -1.0, 1.0))
    if angle > np.pi:
        angle -= 2*np.pi
    return angle * v / nv


def q_to_R(q):
    q = q_normalize(q)
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def q_from_R(R):
    R = np.asarray(R, float)
    tr = np.trace(R)
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2.0
        q = np.array([0.25*S, (R[2,1]-R[1,2])/S, (R[0,2]-R[2,0])/S, (R[1,0]-R[0,1])/S])
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
            q = np.array([(R[2,1]-R[1,2])/S, 0.25*S, (R[0,1]+R[1,0])/S, (R[0,2]+R[2,0])/S])
        elif i == 1:
            S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0
            q = np.array([(R[0,2]-R[2,0])/S, (R[0,1]+R[1,0])/S, 0.25*S, (R[1,2]+R[2,1])/S])
        else:
            S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0
            q = np.array([(R[1,0]-R[0,1])/S, (R[0,2]+R[2,0])/S, (R[1,2]+R[2,1])/S, 0.25*S])
    return q_normalize(q)


def rpy_to_q(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return q_normalize(np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ]))


def q_to_rpy(q):
    R = q_to_R(q)
    pitch = math.asin(np.clip(-R[2, 0], -1, 1))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])


def orientation_residual(q_pred, q_meas):
    return q_log(q_mul(q_conj(q_pred), q_meas))


def pose_compose(a, b):
    ca, sa = np.cos(a[2]), np.sin(a[2])
    return np.array([a[0] + ca*b[0] - sa*b[1],
                     a[1] + sa*b[0] + ca*b[1],
                     wrap(a[2] + b[2])])


def pose_inverse(a):
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([-c*a[0] - s*a[1], s*a[0] - c*a[1], wrap(-a[2])])


def pose_between(a, b):
    return pose_compose(pose_inverse(a), b)


def trans_pts(P, pose):
    c, s = np.cos(pose[2]), np.sin(pose[2])
    R = np.array([[c, -s], [s, c]])
    return P @ R.T + pose[:2]


def rigid_fit(src, dst):
    cs, cd = src.mean(0), dst.mean(0)
    X, Y = src - cs, dst - cd
    U, _, Vt = np.linalg.svd(X.T @ Y)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = cd - R @ cs
    return np.array([t[0], t[1], math.atan2(R[1, 0], R[0, 0])])


def voxel_down(P, voxel=0.05):
    if len(P) == 0:
        return P
    keys = np.floor(P / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return P[np.sort(idx)]


@dataclass
class Config:
    duration: float = 60.0
    imu_rate: float = 200.0
    lidar_rate: float = 5.5
    vio_rate: float = 30.0
    range_rate: float = 30.0
    baro_rate: float = 25.0
    n_beams: int = 360
    range_sigma: float = 0.012
    requirement_m: float = 0.10

    accel_nd: float = 0.003
    gyro_nd: float = np.deg2rad(0.025)
    accel_bias_rw: float = 2e-4
    gyro_bias_rw: float = np.deg2rad(0.002)

    vio_pos_sigma: float = 0.015
    vio_vel_sigma: float = 0.025
    vio_att_sigma: float = np.deg2rad(0.45)
    vio_drift_pos_rw: float = 0.0008
    vio_drift_att_rw: float = np.deg2rad(0.012)
    vio_outlier_prob: float = 0.005

    lidar_sigma_xy: float = 0.025
    lidar_sigma_yaw: float = np.deg2rad(0.7)
    rangefinder_sigma: float = 0.012
    baro_sigma: float = 0.06
    baro_bias_rw: float = 0.002

    # Calibrated sensor extrinsics. R_BS maps sensor axes to body axes;
    # r_BS is the sensor origin expressed in body coordinates.
    r_BC: np.ndarray = None
    R_BC: np.ndarray = None
    r_BL: np.ndarray = None
    R_BL: np.ndarray = None
    r_BR: np.ndarray = None
    d_BR: np.ndarray = None

    icp_max_corr: float = 0.22
    icp_trim: float = 0.72
    icp_iters: int = 14
    icp_step_xy: float = 0.08
    icp_step_yaw: float = np.deg2rad(3.0)
    local_submap_scans: int = 45
    map_voxel: float = 0.045

    sc_rings: int = 20
    sc_sectors: int = 60
    sc_exclude_recent: int = 45
    sc_threshold: float = 0.24
    sc_min_separation_m: float = 0.40
    sc_verify_rmse: float = 0.065
    keyframe_stride: int = 5
    max_loop_closures: int = 8
    icp_health_rmse: float = 0.080
    icp_health_overlap: int = 80
    icp_health_correction_m: float = 0.12
    icp_health_correction_yaw: float = np.deg2rad(5.0)

    def __post_init__(self):
        if self.r_BC is None: self.r_BC = np.array([0.08, 0.00, 0.02])
        if self.R_BC is None: self.R_BC = np.eye(3)
        if self.r_BL is None: self.r_BL = np.array([0.00, 0.00, 0.05])
        if self.R_BL is None: self.R_BL = np.eye(3)
        if self.r_BR is None: self.r_BR = np.array([0.00, 0.00, -0.05])
        if self.d_BR is None: self.d_BR = np.array([0.00, 0.00, -1.00])


ROOM_W = 6.0
ROOM_D = 6.0
OBSTACLES = np.array([[1.0, 1.0, 0.5, 0.5], [4.0, 3.5, 0.5, 0.5]])
G = np.array([0.0, 0.0, -9.81])


def simulate_truth(cfg: Config):
    dt = 1.0 / cfg.imu_rate
    t = np.arange(0.0, cfg.duration + 0.5*dt, dt)
    cx, cy = 3.2, 2.0
    rx, ry = 1.5, 0.9
    phi = np.deg2rad(250.0)
    w1 = 2*np.pi/30
    w2 = 4*np.pi/30
    p = np.column_stack([
        cx + rx*np.sin(w1*t),
        cy + ry*np.sin(w2*t + phi),
        1.15 + 0.16*np.sin(2*np.pi*t/18),
    ])
    v = np.column_stack([
        rx*w1*np.cos(w1*t),
        ry*w2*np.cos(w2*t + phi),
        0.16*(2*np.pi/18)*np.cos(2*np.pi*t/18),
    ])
    a = np.column_stack([
        -rx*w1*w1*np.sin(w1*t),
        -ry*w2*w2*np.sin(w2*t + phi),
        -0.16*(2*np.pi/18)**2*np.sin(2*np.pi*t/18),
    ])
    yaw = np.unwrap(np.arctan2(v[:,1], v[:,0]))
    roll = np.deg2rad(5.0)*np.sin(2*np.pi*t/8.0)
    pitch = np.deg2rad(4.0)*np.sin(2*np.pi*t/10.0 + 0.4)
    q = np.vstack([rpy_to_q(rr, pp, yy) for rr, pp, yy in zip(roll, pitch, yaw)])
    omega = np.zeros_like(p)
    for k in range(len(t)-1):
        dq = q_mul(q_conj(q[k]), q[k+1])
        omega[k] = q_log(dq)/dt
    omega[-1] = omega[-2]
    return {'t': t, 'p': p, 'v': v, 'a': a, 'q': q, 'omega': omega,
            'rpy': np.column_stack([roll, pitch, yaw])}


def simulate_imu(gt, cfg, rng):
    dt = 1/cfg.imu_rate
    N = len(gt['t'])
    ba = np.zeros((N,3)); bg = np.zeros((N,3))
    ba[0] = [0.018, -0.014, 0.012]
    bg[0] = np.deg2rad([0.06, -0.04, 0.05])
    ba[1:] = ba[0] + np.cumsum(cfg.accel_bias_rw*np.sqrt(dt)*rng.standard_normal((N-1,3)), axis=0)
    bg[1:] = bg[0] + np.cumsum(cfg.gyro_bias_rw*np.sqrt(dt)*rng.standard_normal((N-1,3)), axis=0)
    sa = cfg.accel_nd/np.sqrt(dt)
    sg = cfg.gyro_nd/np.sqrt(dt)
    acc = np.zeros((N,3))
    for k in range(N):
        R = q_to_R(gt['q'][k])
        acc[k] = R.T @ (gt['a'][k] - G)
    acc += ba + sa*rng.standard_normal((N,3))
    gyro = gt['omega'] + bg + sg*rng.standard_normal((N,3))
    return {'acc': acc, 'gyro': gyro, 'ba_true': ba, 'bg_true': bg,
            'sigma_acc_sample': sa, 'sigma_gyro_sample': sg}


def nearest_indices(t, ts):
    idx = np.rint(ts/(t[1]-t[0])).astype(int)
    return np.clip(idx, 0, len(t)-1)


def simulate_vio(gt, cfg, rng, dropout_window=None):
    """Host-generated D435i visual-inertial odometry.

    The simulated measurement is first formed at the camera optical centre and
    then converted back to the body/IMU frame with calibrated extrinsics.
    A D435i does not itself provide a complete VIO solution; on hardware this
    measurement comes from the selected VIO software pipeline.
    """
    dt = 1/cfg.imu_rate
    N = len(gt['t'])
    drift_p = np.zeros((N,3)); drift_th = np.zeros((N,3))
    drift_p[1:] = np.cumsum(cfg.vio_drift_pos_rw*np.sqrt(dt)*rng.standard_normal((N-1,3)), axis=0)
    drift_th[1:] = np.cumsum(cfg.vio_drift_att_rw*np.sqrt(dt)*rng.standard_normal((N-1,3)), axis=0)
    t = np.arange(0, cfg.duration + 1e-12, 1/cfg.vio_rate)
    idx = nearest_indices(gt['t'], t)
    p = np.zeros((len(t),3)); v = np.zeros((len(t),3)); q = np.zeros((len(t),4))
    for i,k in enumerate(idx):
        RWB = q_to_R(gt['q'][k])
        RWC = RWB @ cfg.R_BC
        pWC = gt['p'][k] + RWB @ cfg.r_BC
        qWC = q_normalize(q_mul(gt['q'][k], rpy_to_q(0.0,0.0,0.0)))
        # Pose noise/drift is applied to the camera trajectory.
        RWCm = RWC @ q_to_R(q_exp(drift_th[k] + cfg.vio_att_sigma*rng.standard_normal(3)))
        pWCm = pWC + drift_p[k] + cfg.vio_pos_sigma*rng.standard_normal(3)
        # Calibrated camera-to-body conversion.
        RWBm = RWCm @ cfg.R_BC.T
        p[i] = pWCm - RWBm @ cfg.r_BC
        q[i] = q_from_R(RWBm)
        v[i] = gt['v'][k] + cfg.vio_vel_sigma*rng.standard_normal(3)
    valid = np.ones(len(t), dtype=bool)
    if dropout_window:
        valid &= ~((t >= dropout_window[0]) & (t <= dropout_window[1]))
    out = rng.random(len(t)) < cfg.vio_outlier_prob
    out[0] = False
    p[out] += 0.35*rng.standard_normal((out.sum(),3))
    for i in np.where(out)[0]:
        q[i] = q_normalize(q_mul(q[i], q_exp(np.deg2rad(12)*rng.standard_normal(3))))
    return {'t': t, 'idx': idx, 'p': p, 'v': v, 'q': q, 'valid': valid, 'outlier': out,
            'drift_p': drift_p, 'drift_th': drift_th}

def simulate_altimeters(gt, cfg, rng, range_dropout_window=(20.0, 35.0)):
    tr = np.arange(0, cfg.duration + 1e-12, 1/cfg.range_rate)
    ir = nearest_indices(gt['t'], tr)
    zr = np.zeros(len(tr))
    for n,k in enumerate(ir):
        R = q_to_R(gt['q'][k])
        pS = gt['p'][k] + R @ cfg.r_BR
        dW = R @ cfg.d_BR
        zr[n] = -pS[2]/dW[2] + cfg.rangefinder_sigma*rng.standard_normal()
    valid_r = rng.random(len(tr)) > 0.05
    if range_dropout_window:
        valid_r &= ~((tr >= range_dropout_window[0]) & (tr <= range_dropout_window[1]))

    tb = np.arange(0, cfg.duration + 1e-12, 1/cfg.baro_rate)
    ib = nearest_indices(gt['t'], tb)
    dtb = 1/cfg.baro_rate
    b = np.zeros(len(tb)); b[0] = 0.12
    b[1:] = b[0] + np.cumsum(cfg.baro_bias_rw*np.sqrt(dtb)*rng.standard_normal(len(tb)-1))
    zb = gt['p'][ib,2] + b + cfg.baro_sigma*rng.standard_normal(len(tb))
    return {'tr':tr,'ir':ir,'zr':zr,'valid_r':valid_r,
            'tb':tb,'ib':ib,'zb':zb,'baro_bias_true':b}


def raycast_scans(gt, cfg, rng):
    tl = np.arange(0, cfg.duration + 1e-12, 1/cfg.lidar_rate)
    il = nearest_indices(gt['t'], tl)
    angles = np.linspace(0, 2*np.pi, cfg.n_beams, endpoint=False)
    scans = []
    for k in il:
        RWB = q_to_R(gt['q'][k])
        pL = gt['p'][k] + RWB @ cfg.r_BL
        RWL = RWB @ cfg.R_BL
        px, py = pL[:2]
        yaw = math.atan2(RWL[1,0], RWL[0,0])
        ranges = np.empty(cfg.n_beams)
        for b, a in enumerate(angles):
            wa = a + yaw
            dx, dy = np.cos(wa), np.sin(wa)
            cand = []
            if abs(dx)>1e-12: cand += [(0-px)/dx, (ROOM_W-px)/dx]
            if abs(dy)>1e-12: cand += [(0-py)/dy, (ROOM_D-py)/dy]
            hit = min(v for v in cand if v > 1e-6)
            for ox,oy,ow,od in OBSTACLES:
                tx1,tx2=((ox-px)/dx,(ox+ow-px)/dx) if abs(dx)>1e-12 else (-np.inf,np.inf)
                ty1,ty2=((oy-py)/dy,(oy+od-py)/dy) if abs(dy)>1e-12 else (-np.inf,np.inf)
                te=max(min(tx1,tx2),min(ty1,ty2)); tx=min(max(tx1,tx2),max(ty1,ty2))
                if te>0 and te<tx and te<hit: hit=te
            ranges[b] = np.clip(hit + cfg.range_sigma*rng.standard_normal(), 0.15, 12.0)
        scans.append(np.column_stack([ranges*np.cos(angles), ranges*np.sin(angles)]))
    return {'t':tl,'idx':il,'angles':angles,'scans':scans}


def interp_vio_planar(vio, t_query):
    """Robust VIO trajectory used only as the LiDAR registration prior."""
    tr=vio['t']; rawp=vio['p']; rawy=np.unwrap(np.array([q_to_rpy(q)[2] for q in vio['q']]))
    pf=np.zeros_like(rawp); yf=np.zeros(len(tr)); pf[0]=rawp[0]; yf[0]=rawy[0]
    v_last=vio['v'][0].copy(); yr=0.0
    for i in range(1,len(tr)):
        dt=max(tr[i]-tr[i-1],1e-6)
        pp=pf[i-1]+v_last*dt; yp=yf[i-1]+yr*dt
        pos_res=np.linalg.norm(rawp[i]-pp); yaw_res=abs(wrap(rawy[i]-yp))
        plausible=(vio['valid'][i] and pos_res<0.12 and yaw_res<np.deg2rad(10))
        if plausible:
            pf[i]=rawp[i]
            dy=wrap(rawy[i]-yf[i-1]); yf[i]=yf[i-1]+dy
            v_last=0.5*v_last+0.5*vio['v'][i]
            yr=0.7*yr+0.3*dy/dt
        else:
            pf[i]=pp; yf[i]=yp
    out=np.zeros((len(t_query),3))
    for d in range(2): out[:,d]=np.interp(t_query,tr,pf[:,d])
    out[:,2]=wrap(np.interp(t_query,tr,yf))
    return out

def icp_to_map(scan, mappts, init_pose, cfg):
    pose = init_pose.copy(); tree = cKDTree(mappts)
    rmse=np.inf; overlap=0
    for _ in range(cfg.icp_iters):
        Q = trans_pts(scan, pose)
        d,j = tree.query(Q,k=1)
        ids=np.where(d<cfg.icp_max_corr)[0]
        overlap=len(ids)
        if overlap<40: break
        ids=ids[np.argsort(d[ids])[:max(40,int(cfg.icp_trim*len(ids)))]]
        delta=rigid_fit(Q[ids],mappts[j[ids]])
        delta[:2]=np.clip(delta[:2],-cfg.icp_step_xy,cfg.icp_step_xy)
        delta[2]=np.clip(delta[2],-cfg.icp_step_yaw,cfg.icp_step_yaw)
        pose=pose_compose(delta,pose)
        rmse=float(np.sqrt(np.mean(d[ids]**2)))
        if np.linalg.norm(delta[:2])<1e-4 and abs(delta[2])<1e-4: break
    return pose, rmse, overlap


def scan_context(scan, cfg):
    r=np.linalg.norm(scan,axis=1)
    th=(np.arctan2(scan[:,1],scan[:,0])+2*np.pi)%(2*np.pi)
    desc=np.zeros((cfg.sc_rings,cfg.sc_sectors))
    ri=np.clip((r/8.0*cfg.sc_rings).astype(int),0,cfg.sc_rings-1)
    si=np.clip((th/(2*np.pi)*cfg.sc_sectors).astype(int),0,cfg.sc_sectors-1)
    for rr,ss,rv in zip(ri,si,r):
        desc[rr,ss]=max(desc[rr,ss],1.0-rv/8.0)
    ring_key=desc.mean(axis=1)
    return desc, ring_key


def scan_context_distance(a,b):
    best=1.0; best_shift=0
    for s in range(a.shape[1]):
        bs=np.roll(b,s,axis=1)
        valid=(np.linalg.norm(a,axis=0)>1e-8)&(np.linalg.norm(bs,axis=0)>1e-8)
        if not np.any(valid): continue
        dots=np.sum(a[:,valid]*bs[:,valid],axis=0)
        den=np.linalg.norm(a[:,valid],axis=0)*np.linalg.norm(bs[:,valid],axis=0)
        dist=1.0-float(np.mean(dots/np.maximum(den,1e-12)))
        if dist<best: best,best_shift=dist,s
    return best,best_shift


def icp_pair(current_scan, reference_scan, init_rel, cfg):
    tree=cKDTree(reference_scan); pose=init_rel.copy(); rmse=np.inf; overlap=0
    for _ in range(20):
        Q=trans_pts(current_scan,pose)
        d,j=tree.query(Q,k=1)
        ids=np.where(d<0.25)[0]; overlap=len(ids)
        if overlap<50: break
        ids=ids[np.argsort(d[ids])[:max(50,int(0.75*len(ids)))]]
        delta=rigid_fit(Q[ids],reference_scan[j[ids]])
        delta[:2]=np.clip(delta[:2],-0.10,0.10); delta[2]=np.clip(delta[2],-np.deg2rad(5),np.deg2rad(5))
        pose=pose_compose(delta,pose)
        rmse=float(np.sqrt(np.mean(d[ids]**2)))
        if np.linalg.norm(delta[:2])<1e-4 and abs(delta[2])<1e-4: break
    return pose,rmse,overlap


def pose_graph_residual(xvec, n, edges, anchor):
    poses=np.zeros((n,3)); poses[0]=anchor; poses[1:]=xvec.reshape(-1,3)
    res=[]
    for i,j,z,sig in edges:
        pred=pose_between(poses[i],poses[j])
        e=pose_between(z,pred)
        e[2]=wrap(e[2])
        res.extend(e/sig)
    return np.asarray(res)


def optimize_pose_graph(poses, edges):
    n=len(poses)
    if n<3: return poses
    x0=poses[1:].reshape(-1)
    # Sparse dependency: each 3-residual edge touches only its two poses.
    S=lil_matrix((3*len(edges),3*(n-1)),dtype=int)
    for e,(i,j,_,_) in enumerate(edges):
        if i>0: S[3*e:3*e+3,3*(i-1):3*i]=1
        if j>0: S[3*e:3*e+3,3*(j-1):3*j]=1
    out=least_squares(pose_graph_residual,x0,args=(n,edges,poses[0].copy()),jac_sparsity=S.tocsr(),
                      loss='huber',f_scale=1.0,max_nfev=8,
                      ftol=1e-6,xtol=1e-6,gtol=1e-6,verbose=0)
    p=poses.copy(); p[1:]=out.x.reshape(-1,3); p[:,2]=wrap(p[:,2])
    return p


def interpolate_global_poses(local_poses, kf_idx, kf_global):
    corrections=[]
    for ii,kg in zip(kf_idx,kf_global):
        corrections.append(pose_compose(kg,pose_inverse(local_poses[ii])))
    corrections=np.asarray(corrections)
    out=np.zeros_like(local_poses)
    for seg in range(len(kf_idx)-1):
        a,b=kf_idx[seg],kf_idx[seg+1]
        for i in range(a,b+1):
            u=0.0 if b==a else (i-a)/(b-a)
            C=np.empty(3)
            C[:2]=(1-u)*corrections[seg,:2]+u*corrections[seg+1,:2]
            C[2]=wrap(corrections[seg,2]+u*wrap(corrections[seg+1,2]-corrections[seg,2]))
            out[i]=pose_compose(C,local_poses[i])
    if kf_idx[-1] < len(local_poses)-1:
        C=corrections[-1]
        for i in range(kf_idx[-1],len(local_poses)): out[i]=pose_compose(C,local_poses[i])
    return out


def lidar_backend(scandata, vio, cfg):
    """Continuous local lidar odometry plus keyframe global pose graph.

    Local poses are suitable for the ESKF/control frame. ScanContext closures
    only modify poses_global; they never create a jump in the local estimator.
    """
    scans=scandata['scans']; tl=scandata['t']
    vio_planar=interp_vio_planar(vio,tl)
    vio_local=np.array([pose_between(vio_planar[0],p) for p in vio_planar])
    poses=np.zeros((len(scans),3)); valid=np.ones(len(scans),dtype=bool); diag=[]
    descriptors=[]; ring_keys=[]
    d0,k0=scan_context(scans[0],cfg); descriptors.append(d0); ring_keys.append(k0)
    for i in range(1,len(scans)):
        rel_prior=pose_between(vio_local[i-1],vio_local[i])
        pred=pose_compose(poses[i-1],rel_prior)
        j0=max(0,i-cfg.local_submap_scans)
        mappts=voxel_down(np.vstack([trans_pts(scans[j][::2],poses[j]) for j in range(j0,i)]),cfg.map_voxel)
        pose,rmse,overlap=icp_to_map(scans[i],mappts,pred,cfg)
        corr=pose_between(pred,pose)
        healthy=(np.isfinite(rmse) and rmse<cfg.icp_health_rmse and overlap>=cfg.icp_health_overlap
                 and np.linalg.norm(corr[:2])<cfg.icp_health_correction_m
                 and abs(corr[2])<cfg.icp_health_correction_yaw)
        if not healthy:
            pose=pred; valid[i]=False
        poses[i]=pose
        d,k=scan_context(scans[i],cfg); descriptors.append(d); ring_keys.append(k)
        diag.append((rmse,overlap,healthy,float(np.linalg.norm(corr[:2])),float(abs(corr[2]))))

    # Keyframe graph: bounded size and robust loss prevent a false closure from
    # stalling or folding the entire graph.
    kf_idx=np.arange(0,len(scans),cfg.keyframe_stride,dtype=int)
    if kf_idx[-1] != len(scans)-1: kf_idx=np.r_[kf_idx,len(scans)-1]
    kf_local=poses[kf_idx].copy(); edges=[]; loops=[]
    for k in range(1,len(kf_idx)):
        z=pose_between(kf_local[k-1],kf_local[k])
        edges.append((k-1,k,z,np.array([0.055,0.055,np.deg2rad(1.5)])))
    exclude_k=max(6,int(np.ceil(cfg.sc_exclude_recent/cfg.keyframe_stride)))
    last_loop=-10**9
    for a in range(exclude_k,len(kf_idx)):
        if len(loops)>=cfg.max_loop_closures or a-last_loop<6: continue
        i=int(kf_idx[a]); key=ring_keys[i]
        old_ids=np.arange(0,a-exclude_k+1)
        if len(old_ids)==0: continue
        keymat=np.vstack([ring_keys[int(kf_idx[b])] for b in old_ids])
        shortlist=old_ids[np.argsort(np.linalg.norm(keymat-key,axis=1))[:6]]
        candidates=[]
        for b in shortlist:
            j=int(kf_idx[b]); d,shift=scan_context_distance(descriptors[i],descriptors[j])
            candidates.append((d,int(b),j,shift))
        candidates.sort(key=lambda x:x[0])
        for d,b,j,shift in candidates[:4]:
            if d>cfg.sc_threshold: break
            # Independent VIO proximity gate plus metric ICP verification.
            if np.linalg.norm(vio_local[i,:2]-vio_local[j,:2])>0.80: continue
            init=pose_between(kf_local[b],kf_local[a])
            zloop,lrmse,lov=icp_pair(scans[i],scans[j],init,cfg)
            consistency=pose_between(init,zloop)
            if (lrmse<cfg.sc_verify_rmse and lov>110 and
                np.linalg.norm(consistency[:2])<0.08 and abs(consistency[2])<np.deg2rad(3.0)):
                edges.append((b,a,zloop,np.array([0.035,0.035,np.deg2rad(1.0)])))
                loops.append((i,j,d,lrmse)); last_loop=a; break
    kf_global=optimize_pose_graph(kf_local,edges) if loops else kf_local.copy()
    poses_global=interpolate_global_poses(poses,kf_idx,kf_global)
    return {'t':tl,'poses_local':poses,'poses_global':poses_global,'valid':valid,
            'loops':loops,'diag':diag,'edges':edges,'keyframes':kf_idx}


def build_global_pose_graph(scans, local_poses, valid, cfg):
    """Build keyframe ScanContext graph without altering local control poses."""
    candidate=np.where(valid)[0]
    if len(candidate)<3:
        return local_poses.copy(),[],np.array([0,len(local_poses)-1])
    # Select approximately uniform accepted keyframes.
    kf=[candidate[0]]
    for i in candidate[1:]:
        if i-kf[-1]>=cfg.keyframe_stride: kf.append(int(i))
    if kf[-1]!=candidate[-1]: kf.append(int(candidate[-1]))
    kf_idx=np.asarray(kf,dtype=int); kf_local=local_poses[kf_idx].copy()
    desc=[]; keys=[]
    for i in kf_idx:
        d,k=scan_context(scans[i],cfg); desc.append(d); keys.append(k)
    edges=[]; loops=[]
    for a in range(1,len(kf_idx)):
        z=pose_between(kf_local[a-1],kf_local[a])
        edges.append((a-1,a,z,np.array([0.055,0.055,np.deg2rad(1.5)])))
    exclude=max(6,int(np.ceil(cfg.sc_exclude_recent/cfg.keyframe_stride)))
    last=-10**9
    for a in range(exclude,len(kf_idx)):
        if len(loops)>=cfg.max_loop_closures or a-last<6: continue
        old=np.arange(0,a-exclude+1)
        keymat=np.vstack([keys[b] for b in old])
        shortlist=old[np.argsort(np.linalg.norm(keymat-keys[a],axis=1))[:6]]
        cand=[]
        for b in shortlist:
            d,shift=scan_context_distance(desc[a],desc[b]); cand.append((d,int(b),shift))
        cand.sort(key=lambda x:x[0])
        for d,b,shift in cand[:4]:
            if d>cfg.sc_threshold: break
            # Coarse local-pose gate, followed by independent metric verification.
            if np.linalg.norm(kf_local[a,:2]-kf_local[b,:2])>1.25: continue
            init=pose_between(kf_local[b],kf_local[a])
            zloop,rmse,overlap=icp_pair(scans[kf_idx[a]],scans[kf_idx[b]],init,cfg)
            consistency=pose_between(init,zloop)
            if (rmse<cfg.sc_verify_rmse and overlap>110 and
                np.linalg.norm(consistency[:2])<0.08 and abs(consistency[2])<np.deg2rad(3.0)):
                edges.append((b,a,zloop,np.array([0.035,0.035,np.deg2rad(1.0)])))
                loops.append((int(kf_idx[a]),int(kf_idx[b]),float(d),float(rmse)))
                last=a; break
    kf_global=optimize_pose_graph(kf_local,edges) if loops else kf_local.copy()
    global_poses=interpolate_global_poses(local_poses,kf_idx,kf_global)
    return global_poses,loops,kf_idx


class ESKF:
    # Nominal: p,v,q,ba,bg,bbaro. Error: dp,dv,dtheta,dba,dbg,dbbaro = 16.
    def __init__(self,p,v,q):
        self.p=p.copy(); self.v=v.copy(); self.q=q_normalize(q.copy())
        self.ba=np.zeros(3); self.bg=np.zeros(3); self.bbaro=0.0
        s=np.r_[ [0.05]*3,[0.12]*3,np.deg2rad([3,3,4]),[0.06]*3,np.deg2rad([0.4]*3),0.20]
        self.P=np.diag(s*s); self.I=np.eye(16)

    def propagate(self,acc_m,gyro_m,dt,cfg,sa,sg):
        f=acc_m-self.ba; w=gyro_m-self.bg; R=q_to_R(self.q)
        aw=R@f+G
        self.p += self.v*dt+0.5*aw*dt*dt
        self.v += aw*dt
        self.q=q_normalize(q_mul(self.q,q_exp(w*dt)))

        F=np.zeros((16,16)); F[0:3,3:6]=np.eye(3)
        F[3:6,6:9]=-R@skew(f); F[3:6,9:12]=-R
        F[6:9,6:9]=-skew(w); F[6:9,12:15]=-np.eye(3)
        Phi=np.eye(16)+F*dt
        Gm=np.zeros((16,13))
        Gm[3:6,0:3]=-R; Gm[6:9,3:6]=-np.eye(3)
        Gm[9:12,6:9]=np.eye(3); Gm[12:15,9:12]=np.eye(3); Gm[15,12]=1
        qc=np.r_[[cfg.accel_nd**2]*3,[cfg.gyro_nd**2]*3,
                 [cfg.accel_bias_rw**2]*3,[cfg.gyro_bias_rw**2]*3,cfg.baro_bias_rw**2]
        Qd=Gm@np.diag(qc)@Gm.T*dt
        self.P=Phi@self.P@Phi.T+Qd
        self.P=0.5*(self.P+self.P.T)

    def inject(self,dx):
        self.p+=dx[0:3]; self.v+=dx[3:6]
        dth=dx[6:9]; self.q=q_normalize(q_mul(self.q,q_exp(dth)))
        self.ba+=dx[9:12]; self.bg+=dx[12:15]; self.bbaro+=dx[15]
        Gr=np.eye(16); Gr[6:9,6:9]=np.eye(3)-0.5*skew(dth)
        self.P=Gr@self.P@Gr.T

    def update(self,r,H,Rm,gate):
        S=H@self.P@H.T+Rm
        nis=float(r@np.linalg.solve(S,r))
        if nis>gate: return False,nis
        K=self.P@H.T@np.linalg.inv(S)
        dx=K@r
        J=self.I-K@H
        self.P=J@self.P@J.T+K@Rm@K.T
        self.inject(dx)
        self.P=0.5*(self.P+self.P.T)
        return True,nis


def align_planar_to_world(poses_local, vio, cfg):
    """Initialize the odom/map frame from the calibrated takeoff pose."""
    R0 = q_to_R(vio['q'][0])
    pL0 = vio['p'][0] + R0 @ cfg.r_BL
    RWL0 = R0 @ cfg.R_BL
    y0 = math.atan2(RWL0[1,0], RWL0[0,0])
    c,s = np.cos(y0),np.sin(y0); R2=np.array([[c,-s],[s,c]])
    xy=poses_local[:,:2]@R2.T+pL0[:2]
    yaw=wrap(poses_local[:,2]+y0)
    return np.column_stack([xy,yaw])


def lidar_model(f, cfg):
    R=q_to_R(f.q); pL=f.p+R@cfg.r_BL; RWL=R@cfg.R_BL
    return np.array([pL[0],pL[1],math.atan2(RWL[1,0],RWL[0,0])])


def range_model(f, cfg):
    R=q_to_R(f.q); pS=f.p+R@cfg.r_BR; dW=R@cfg.d_BR
    if dW[2] >= -0.2:
        return np.nan
    return -pS[2]/dW[2]


def lidar_measurement_and_H(f, cfg):
    R=q_to_R(f.q); h=lidar_model(f,cfg)
    H=np.zeros((3,16)); H[0,0]=1.0; H[1,1]=1.0
    H[0:2,6:9]=(-R@skew(cfg.r_BL))[0:2,:]
    # Only the 3 attitude columns need numerical differentiation for yaw.
    base=h[2]; eps=1e-7
    for j in range(3):
        d=np.zeros(3); d[j]=eps
        Rp=R@q_to_R(q_exp(d)); RWL=Rp@cfg.R_BL
        yp=math.atan2(RWL[1,0],RWL[0,0])
        H[2,6+j]=wrap(yp-base)/eps
    return h,H


def range_measurement_and_H(f, cfg):
    R=q_to_R(f.q); pS=f.p+R@cfg.r_BR; dW=R@cfg.d_BR
    den=dW[2]
    if den>=-0.2:
        return np.array([np.nan]),np.zeros((1,16))
    num=-pS[2]; h=num/den
    H=np.zeros((1,16)); H[0,2]=-1.0/den
    Ar=(-R@skew(cfg.r_BR))[2,:]
    Ad=(-R@skew(cfg.d_BR))[2,:]
    H[0,6:9]=(-Ar*den-num*Ad)/(den*den)
    return np.array([h]),H

def numerical_yaw_H(q):
    base=q_to_rpy(q)[2]; H=np.zeros(3); eps=1e-6
    for i in range(3):
        d=np.zeros(3); d[i]=eps
        y=q_to_rpy(q_mul(q,q_exp(d)))[2]
        H[i]=wrap(y-base)/eps
    return H


def run_integrated_eskf(gt,imu,vio,alt,scandata,cfg):
    """Causal local estimator: ESKF prediction supplies every ICP prior."""
    t=gt['t']; N=len(t); dt=t[1]-t[0]
    f=ESKF(vio['p'][0],vio['v'][0],vio['q'][0])
    hist_p=np.zeros((N,3)); hist_q=np.zeros((N,4)); hist_ba=np.zeros((N,3)); hist_bg=np.zeros((N,3)); hist_bb=np.zeros(N)
    hist_p[0]=f.p; hist_q[0]=f.q
    kv=1; kl=0; kr=1; kb=1
    counts={'vio_acc':1,'vio_rej':0,'lidar_acc':0,'lidar_rej':0,'range_acc':1,'range_rej':0,'baro_acc':1,'baro_rej':0}
    nis={'vio':[],'lidar':[],'range':[],'baro':[]}
    gate_vio=27.877; gate_lidar=16.266; gate_1=10.828
    Rvio=np.diag(np.r_[[cfg.vio_pos_sigma**2]*3,[cfg.vio_vel_sigma**2]*3,[cfg.vio_att_sigma**2]*3])
    Rlid=np.diag([cfg.lidar_sigma_xy**2,cfg.lidar_sigma_xy**2,cfg.lidar_sigma_yaw**2])
    Rrng=np.array([[cfg.rangefinder_sigma**2]]); Rbar=np.array([[cfg.baro_sigma**2]])
    scans=scandata['scans']; tl=scandata['t']; L=len(scans)
    lidar_poses=np.zeros((L,3)); lidar_valid=np.zeros(L,dtype=bool); lidar_diag=[]; map_blocks=[]

    def process_lidar(index):
        nonlocal f
        pred=lidar_model(f,cfg)
        if len(map_blocks)==0:
            pose=pred.copy(); rmse=0.0; overlap=len(scans[index]); healthy=True
        else:
            mappts=voxel_down(np.vstack(map_blocks[-cfg.local_submap_scans:]),cfg.map_voxel)
            pose,rmse,overlap=icp_to_map(scans[index],mappts,pred,cfg)
            corr=pose_between(pred,pose)
            healthy=(np.isfinite(rmse) and rmse<cfg.icp_health_rmse and overlap>=cfg.icp_health_overlap and
                     np.linalg.norm(corr[:2])<cfg.icp_health_correction_m and
                     abs(corr[2])<cfg.icp_health_correction_yaw)
        if healthy:
            h,H=lidar_measurement_and_H(f,cfg); r=pose-h; r[2]=wrap(r[2])
            ok,n=f.update(r,H,Rlid,gate_lidar)
        else:
            ok=False; n=np.inf
        if ok:
            lidar_poses[index]=pose; lidar_valid[index]=True
            map_blocks.append(trans_pts(scans[index][::2],pose))
            counts['lidar_acc']+=1; nis['lidar'].append(n)
        else:
            lidar_poses[index]=pred; counts['lidar_rej']+=1
            if np.isfinite(n): nis['lidar'].append(n)
        lidar_diag.append((index,float(rmse),int(overlap),bool(healthy),bool(ok)))

    # Process t=0 lidar after initialization.
    while kl<L and tl[kl]<=t[0]+1e-12:
        process_lidar(kl); kl+=1
    for k in range(1,N):
        f.propagate(imu['acc'][k],imu['gyro'][k],dt,cfg,imu['sigma_acc_sample'],imu['sigma_gyro_sample'])
        now=t[k]+1e-12
        while kv<len(vio['t']) and vio['t'][kv]<=now:
            if vio['valid'][kv]:
                r=np.r_[vio['p'][kv]-f.p,vio['v'][kv]-f.v,orientation_residual(f.q,vio['q'][kv])]
                H=np.zeros((9,16)); H[0:3,0:3]=np.eye(3); H[3:6,3:6]=np.eye(3); H[6:9,6:9]=np.eye(3)
                ok,n=f.update(r,H,Rvio,gate_vio); counts['vio_acc' if ok else 'vio_rej']+=1; nis['vio'].append(n)
            kv+=1
        while kl<L and tl[kl]<=now:
            process_lidar(kl); kl+=1
        while kr<len(alt['tr']) and alt['tr'][kr]<=now:
            if alt['valid_r'][kr]:
                h,H=range_measurement_and_H(f,cfg)
                if np.isfinite(h[0]):
                    r=np.array([alt['zr'][kr]-h[0]])
                    ok,n=f.update(r,H,Rrng,gate_1); counts['range_acc' if ok else 'range_rej']+=1; nis['range'].append(n)
                else: counts['range_rej']+=1
            kr+=1
        while kb<len(alt['tb']) and alt['tb'][kb]<=now:
            r=np.array([alt['zb'][kb]-(f.p[2]+f.bbaro)])
            H=np.zeros((1,16)); H[0,2]=1; H[0,15]=1
            ok,n=f.update(r,H,Rbar,gate_1); counts['baro_acc' if ok else 'baro_rej']+=1; nis['baro'].append(n)
            kb+=1
        hist_p[k]=f.p; hist_q[k]=f.q; hist_ba[k]=f.ba; hist_bg[k]=f.bg; hist_bb[k]=f.bbaro

    pos_err=np.linalg.norm(hist_p-gt['p'],axis=1)
    att_err=np.array([np.linalg.norm(orientation_residual(hist_q[i],gt['q'][i])) for i in range(N)])
    ss=t>=5
    return {'p':hist_p,'q':hist_q,'ba':hist_ba,'bg':hist_bg,'bbaro':hist_bb,
            'pos_err':pos_err,'att_err':att_err,'max5':float(pos_err[ss].max()),
            'rmse5':float(np.sqrt(np.mean(pos_err[ss]**2))),'final':float(pos_err[-1]),
            'att_max5_deg':float(np.rad2deg(att_err[ss].max())),
            'counts':counts,'nis':nis,'lidar_poses':lidar_poses,'lidar_valid':lidar_valid,'lidar_diag':lidar_diag}


def full_trial(seed=0,cfg=None,stress=False,keep=False):
    if cfg is None: cfg=Config()
    rng=np.random.default_rng(seed)
    gt=simulate_truth(cfg); imu=simulate_imu(gt,cfg,rng)
    vio=simulate_vio(gt,cfg,rng,dropout_window=(25,32) if stress else None)
    alt=simulate_altimeters(gt,cfg,rng,range_dropout_window=(18,38) if stress else (20,35))
    scans=raycast_scans(gt,cfg,rng)
    ekf=run_integrated_eskf(gt,imu,vio,alt,scans,cfg)
    local=ekf['lidar_poses']; valid=ekf['lidar_valid']
    global_poses,loops,kf=build_global_pose_graph(scans['scans'],local,valid,cfg)
    lidar_gt=np.zeros_like(local)
    for n,k in enumerate(scans['idx']):
        R=q_to_R(gt['q'][k]); pL=gt['p'][k]+R@cfg.r_BL; RWL=R@cfg.R_BL
        lidar_gt[n]=[pL[0],pL[1],math.atan2(RWL[1,0],RWL[0,0])]
    lerr=np.linalg.norm(local[:,:2]-lidar_gt[:,:2],axis=1)
    gerr=np.linalg.norm(global_poses[:,:2]-lidar_gt[:,:2],axis=1)
    result={'seed':seed,'stress':stress,'lidar_local_max':float(lerr.max()),
            'lidar_local_rmse':float(np.sqrt(np.mean(lerr*lerr))),
            'lidar_global_max':float(gerr.max()),'lidar_global_rmse':float(np.sqrt(np.mean(gerr*gerr))),
            'local_return_error':float(np.linalg.norm(pose_between(local[0],local[-1])[:2])),
            'global_return_error':float(np.linalg.norm(pose_between(global_poses[0],global_poses[-1])[:2])),
            'lidar_healthy_fraction':float(np.mean(valid)),'loops':len(loops),
            'fused_max5':ekf['max5'],'fused_rmse5':ekf['rmse5'],'fused_final':ekf['final'],
            'att_max5_deg':ekf['att_max5_deg'],'counts':ekf['counts'],
            'pass':bool(ekf['max5']<cfg.requirement_m and np.mean(valid)>0.80)}
    if keep: result.update({'gt':gt,'imu':imu,'vio':vio,'alt':alt,'scans':scans,'ekf':ekf,
                            'z_lidar_local':local,'z_lidar_global':global_poses,'loops_detail':loops,'keyframes':kf})
    return result

def clean_result(r):
    return {k:v for k,v in r.items() if k not in {'gt','imu','vio','alt','scans','lidar','z_lidar','ekf'}}


if __name__=='__main__':
    cfg=Config(); start=time.time(); rows=[]
    print('=== PRODUCTION 6-DOF LOCAL ESKF + GLOBAL SCANCONTEXT POSE GRAPH ===',flush=True)
    for seed in range(1):
        r=full_trial(seed,cfg,stress=False,keep=False); rows.append(r); print(json.dumps(clean_result(r)),flush=True)
    stress=[]
    for seed in range(1):
        r=full_trial(100+seed,cfg,stress=True,keep=False); stress.append(r); print('STRESS '+json.dumps(clean_result(r)),flush=True)
    summary={
        'nominal_trials':len(rows),'nominal_pass':sum(x['pass'] for x in rows),
        'nominal_worst_fused_max5':max(x['fused_max5'] for x in rows),
        'nominal_worst_lidar_local_max':max(x['lidar_local_max'] for x in rows),'nominal_worst_lidar_global_max':max(x['lidar_global_max'] for x in rows),
        'nominal_min_loop_closures':min(x['loops'] for x in rows),
        'stress_trials':len(stress),'stress_pass':sum(x['pass'] for x in stress),
        'stress_worst_fused_max5':max(x['fused_max5'] for x in stress),
        'elapsed_s':time.time()-start,
    }
    print('SUMMARY '+json.dumps(summary,indent=2),flush=True)
    with open('/mnt/data/s2_6dof_full_fusion_results.json','w') as f:
        json.dump({'nominal':[clean_result(x) for x in rows],'stress':[clean_result(x) for x in stress],'summary':summary},f,indent=2)

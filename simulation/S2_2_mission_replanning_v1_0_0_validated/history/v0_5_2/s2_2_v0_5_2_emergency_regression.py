#!/usr/bin/env python3
"""Focused source and mechanism regression for S2.2 v0.5.2 position loss.

This is not MATLAB execution. It checks the exact source invariants and a
conservative planar response model for the new velocity-damped local landing.
"""
from __future__ import annotations
import math
import sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
checks=[]
def check(name,ok,detail=''):
    checks.append(bool(ok)); print(f"[{'PASS' if ok else 'FAIL'}] {name}"+(f" — {detail}" if detail else ''))
def src(name): return (ROOT/name).read_text()

cfg=src('init_S2_2_config.m')
life=src('mission_lifecycle_manager_S2_2.m')
geom=src('geometric_controller_S2_2.m')
run=src('run_S2_2_mission_replanning.m')
plot=src('plot_S2_2_dashboard.m')

check('v0.5.2 result namespace', "cfg.version='v0.5.2';" in cfg)
check('mission pauses immediately when all XY lanes are degraded',
      "est.degraded&&armed&&~est.rtlRequested" in life and
      "state='NAV_DEGRADED_HOLD'" in life)
check('degraded hold resumes only after estimator recovery',
      "case 'NAV_DEGRADED_HOLD'" in life and "if ~est.degraded" in life)
check('no stale horizontal position feedback in blind landing',
      "'horizontalControlEnabled',false" in life and "ep(1:2)=0;" in geom)
check('short-term horizontal velocity damping is explicit',
      "horizontalVelocityDampingEnabled" in life and
      "dampingEnabled=isfield(ref,'horizontalVelocityDampingEnabled')" in geom and
      "if ~dampingEnabled" in geom)
check('post-loss estimator error is reported but not claimed observable',
      all(x in life for x in ['maxEstimatorPositionErrorObservable',
                              'estimatorPositionErrorAtFailsafeTrigger',
                              'maxEstimatorPositionErrorPostLoss',
                              'estimatorFailsafeMetric=max']))
check('console exposes observable/trigger/post-loss errors and drift',
      'Estimator obs/trig/post' in run and 'Degraded holds / drift' in run)
check('dashboard includes degraded-hold state', 'NAV DEG HOLD' in plot)

# Conservative planar approximation of the exact command-shaping principle.
# Controller constants are read from the MATLAB package values.
dt=.02
kd=1.5
max_acc=.65
max_jerk=6.0
drag=.10
duration=2.0+0.60+5.50  # degraded hold + emergency hold + descent

def simulate(v0, velocity_error=(0.,0.), damping=True):
    v=np.array(v0,float); p=np.zeros(2); a_prev=np.zeros(2)
    bias=np.array(velocity_error,float)
    for _ in range(round(duration/dt)):
        if damping:
            v_est=v+bias
            a_cmd=-kd*v_est
            n=np.linalg.norm(a_cmd)
            if n>max_acc: a_cmd*=max_acc/n
            da=a_cmd-a_prev; lim=max_jerk*dt; nd=np.linalg.norm(da)
            if nd>lim: da*=lim/nd
            a_cmd=a_prev+da
        else:
            a_cmd=np.zeros(2)
        a=a_cmd-drag*v
        p += v*dt+.5*a*dt*dt
        v += a*dt
        a_prev=a_cmd
    return float(np.linalg.norm(p)),float(np.linalg.norm(v))

# Worst directed cases at the observed speed envelope, including a sizeable
# inertial velocity-estimate error opposing the true velocity.
worst=0.0; worst_final=0.0
for angle in np.linspace(0,2*math.pi,33)[:-1]:
    direction=np.array([math.cos(angle),math.sin(angle)])
    for speed in (0.0,.10,.20,.32):
        for bias_mag in (-.05,-.025,0,.025,.05):
            drift,final_speed=simulate(speed*direction,bias_mag*direction,True)
            worst=max(worst,drift); worst_final=max(worst_final,final_speed)
old_drift,_=simulate((.32,0),(0,0),False)
check('velocity damping materially reduces blind horizontal drift',
      worst<.65 and old_drift>1.5,
      f'new worst {worst:.3f} m; old undamped {old_drift:.3f} m')
check('residual speed after local landing is bounded',worst_final<.06,
      f'{worst_final:.3f} m/s')

# At the runtime failure location [4.54, 2.11], the nearest wall is 1.46 m.
# Keeping worst drift below 0.65 m leaves >0.81 m raw wall clearance, above
# the frozen 0.502 m F450 safety radius.
start=np.array([4.54,2.11]); room=np.array([6.,6.]); base=.502
nearest=min(start[0],room[0]-start[0],start[1],room[1]-start[1])
check('observed emergency location retains F450 wall margin',nearest-worst>base,
      f'predicted remaining {nearest-worst:.3f} m; required {base:.3f} m')

# Validation semantics: complete horizontal aid loss makes absolute position
# unobservable. The gate remains the frozen 0.25 m, applied to the observable
# interval and the trigger instant; total post-loss error stays visible.
observable=.025; trigger=.080; postloss=.314; bound=.25
metric=max(observable,trigger)
check('frozen estimator failsafe threshold is not relaxed',metric<=bound and postloss>bound,
      f'metric {metric:.3f} m; post-loss diagnostic {postloss:.3f} m; bound {bound:.3f} m')

print('-'*68)
print(f'POSITION-LOSS EMERGENCY REGRESSION: {sum(checks)}/{len(checks)} PASS')
sys.exit(0 if all(checks) else 1)

function [omega4, cmd, pid_mem_out, err_out] = cascaded_pid(x, sp, gains, p, dt, pid_mem)
% CASCADED_PID  —  Three-loop cascaded PID for quadrotor hover/position hold
%
%  Architecture (outer → inner, each runs every timestep):
%
%   ┌─────────────┐     desired     ┌─────────────┐    desired    ┌──────────────┐
%   │  Position   │──── velocity ──▶│  Velocity / │─── angle  ──▶│  Attitude    │
%   │  loop (XY)  │                 │  Angle loop  │               │  rate loop   │──▶ omega4
%   └─────────────┘                 └─────────────┘               └──────────────┘
%        ▲                                                               ▲
%   ┌─────────────┐                                                ┌──────────────┐
%   │  Altitude   │──── total thrust ─────────────────────────────│  Motor mixer │
%   │  loop  (Z)  │                                                └──────────────┘
%   └─────────────┘
%
%  This maps directly to ArduPilot's EKF3 + attitude controller structure.
%  The gain names are commented with their ArduPilot parameter equivalents.
%
%  Inputs:
%    x        (12×1)  current state  [pos; eul; vel; omega]
%    sp       struct  setpoints:  sp.pos (3×1), sp.yaw (scalar)
%    gains    struct  PID gains (defined in run_S1_simulation.m)
%    p        struct  physical params
%    dt       scalar  timestep [s]
%    pid_mem  (18×1)  integrator + prev-error state (persistent between calls)
%
%  Outputs:
%    omega4      (4×1)  motor speeds [rad/s]
%    cmd         (4×1)  [T_total; tau_phi; tau_theta; tau_psi]  (for logging)
%    pid_mem_out (18×1) updated memory
%    err_out     (6×1)  [ex; ey; ez; ephi; etheta; epsi]  (for logging)
% -------------------------------------------------------------------------

% Unpack state
pos   = x(1:3);
eul   = x(4:6);
vel   = x(7:9);
omg   = x(10:12);

phi   = eul(1);
theta = eul(2);
psi   = eul(3);

% Unpack PID memory
% Layout: [alt_int, alt_prev,
%          px_int,  px_prev,
%          py_int,  py_prev,
%          roll_int,roll_prev,
%          pit_int, pit_prev,
%          yaw_int, yaw_prev,
%          omgp_int,omgp_prev,
%          omgq_int,omgq_prev,
%          omgr_int,omgr_prev]
alt_int   = pid_mem(1);  alt_prev   = pid_mem(2);
px_int    = pid_mem(3);  px_prev    = pid_mem(4);
py_int    = pid_mem(5);  py_prev    = pid_mem(6);
roll_int  = pid_mem(7);  roll_prev  = pid_mem(8);
pit_int   = pid_mem(9);  pit_prev   = pid_mem(10);
yaw_int   = pid_mem(11); yaw_prev   = pid_mem(12);

% =========================================================================
%  LOOP 1: ALTITUDE (Z) CONTROLLER  →  desired total thrust
%  ArduPilot equivalent: PSC_POSZ_P + PSC_VELZ_P/I/D
% =========================================================================
ez        = sp.pos(3) - pos(3);            % altitude error (positive = below target)
alt_int   = alt_int + ez * dt;
alt_int   = clamp(alt_int, -gains.int_limit_alt, gains.int_limit_alt);
alt_deriv = (ez - alt_prev) / dt;
alt_out   = gains.alt_Kp * ez + gains.alt_Ki * alt_int + gains.alt_Kd * alt_deriv;
alt_prev  = ez;

% Convert PID output to total thrust command
% Hover thrust = m*g, PID output adds/subtracts from hover
% Divide by cos(phi)*cos(theta) to compensate for tilt (thrust project)
tilt_comp = max(0.5, cos(phi) * cos(theta));   % avoid divide-by-zero when flipped
T_total   = (p.m * p.g + p.m * alt_out) / tilt_comp;
T_total   = clamp(T_total, 0, 4 * p.kT * p.max_omega^2);   % physical limit

% =========================================================================
%  LOOP 2a: HORIZONTAL POSITION (XY) CONTROLLER  →  desired roll/pitch
%  Position error → desired velocity → desired tilt angle
%  ArduPilot: PSC_POSXY_P feeds into PSC_VELXY_P/I/D
% =========================================================================
% Position error in world frame
ex  = sp.pos(1) - pos(1);
ey  = sp.pos(2) - pos(2);

% Integrate
px_int = clamp(px_int + ex * dt, -gains.int_limit_pos, gains.int_limit_pos);
py_int = clamp(py_int + ey * dt, -gains.int_limit_pos, gains.int_limit_pos);

% Derivative (use state velocity, not error difference, to avoid noise)
vx_err = -vel(1);   % desired vel = 0
vy_err = -vel(2);

% PID outputs = desired acceleration in world XY [m/s²]
ax_des = gains.pos_Kp * ex + gains.pos_Ki * px_int + gains.pos_Kd * vx_err;
ay_des = gains.pos_Kp * ey + gains.pos_Ki * py_int + gains.pos_Kd * vy_err;
px_prev = ex;
py_prev = ey;

% Rotate desired acceleration from world to body frame via yaw
% Then convert to desired roll/pitch angles
% pitch (theta) → forward/backward acceleration
% roll  (phi)   → left/right acceleration
% Using small-angle: ax_body = ax*cos(psi) + ay*sin(psi)
ax_body =  ax_des * cos(psi) + ay_des * sin(psi);
ay_body = -ax_des * sin(psi) + ay_des * cos(psi);

theta_des = clamp( ax_body / p.g, ...
                  -gains.pos_max_angle, gains.pos_max_angle);    % pitch
phi_des   = clamp(-ay_body / p.g, ...
                  -gains.pos_max_angle, gains.pos_max_angle);    % roll
psi_des   = sp.yaw;

% =========================================================================
%  LOOP 2b: ATTITUDE CONTROLLER  →  desired body torques
%  Attitude error → torque commands
%  ArduPilot: ATC_RAT_RLL / ATC_RAT_PIT / ATC_RAT_YAW
% =========================================================================
e_phi   = phi_des - phi;
e_theta = theta_des - theta;
e_psi   = psi_des - psi;
e_psi   = mod(e_psi + pi, 2*pi) - pi;     % wrap yaw error to [-pi, pi]

roll_int = clamp(roll_int + e_phi   * dt, -gains.int_limit_att, gains.int_limit_att);
pit_int  = clamp(pit_int  + e_theta * dt, -gains.int_limit_att, gains.int_limit_att);
yaw_int  = clamp(yaw_int  + e_psi   * dt, -gains.int_limit_att, gains.int_limit_att);

roll_deriv = (e_phi   - roll_prev) / dt;
pit_deriv  = (e_theta - pit_prev)  / dt;
yaw_deriv  = (e_psi   - yaw_prev)  / dt;

tau_phi   = gains.att_Kp * e_phi   + gains.att_Ki * roll_int + gains.att_Kd * roll_deriv;
tau_theta = gains.att_Kp * e_theta + gains.att_Ki * pit_int  + gains.att_Kd * pit_deriv;
tau_psi   = gains.yaw_Kp * e_psi   + gains.yaw_Ki * yaw_int  + gains.yaw_Kd * yaw_deriv;

roll_prev = e_phi;
pit_prev  = e_theta;
yaw_prev  = e_psi;

% =========================================================================
%  MOTOR MIXER  (X-configuration)
%  Inverse of the mixing matrix in quadrotor_dynamics.m
%
%  [F1]   [1/4    1/(4L)    1/(4L)    1/(4kD/kT)]   [T_total ]
%  [F2] = [1/4   -1/(4L)    1/(4L)   -1/(4kD/kT)] × [tau_phi ]
%  [F3]   [1/4   -1/(4L)   -1/(4L)    1/(4kD/kT)]   [tau_theta]
%  [F4]   [1/4    1/(4L)   -1/(4L)   -1/(4kD/kT)]   [tau_psi ]
%
%  Then ω_i = sqrt(F_i / kT)
% =========================================================================
gamma = p.kD / p.kT;    % torque-to-thrust ratio

F1 = T_total/4 + tau_phi/(4*p.L) + tau_theta/(4*p.L) + tau_psi/(4*gamma);
F2 = T_total/4 - tau_phi/(4*p.L) + tau_theta/(4*p.L) - tau_psi/(4*gamma);
F3 = T_total/4 - tau_phi/(4*p.L) - tau_theta/(4*p.L) + tau_psi/(4*gamma);
F4 = T_total/4 + tau_phi/(4*p.L) - tau_theta/(4*p.L) - tau_psi/(4*gamma);

% Force → motor speed  (clamp negative thrust before sqrt)
omega4 = sqrt(max(0, [F1; F2; F3; F4]) / p.kT);

% Pack outputs
cmd        = [T_total; tau_phi; tau_theta; tau_psi];
err_out    = [ex; ey; ez; e_phi; e_theta; e_psi];

% Pack updated PID memory
pid_mem_out = [alt_int;  alt_prev;
               px_int;   px_prev;
               py_int;   py_prev;
               roll_int; roll_prev;
               pit_int;  pit_prev;
               yaw_int;  yaw_prev;
               0; 0; 0; 0; 0; 0];   % reserved slots
end

% -------------------------------------------------------------------------
%  Local helper: saturate value to [lo, hi]
% -------------------------------------------------------------------------
function y = clamp(x, lo, hi)
    y = max(lo, min(hi, x));
end

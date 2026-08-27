% =========================================================================
%  run_S1_simulation.m  —  Stage S1: Quadrotor Dynamics + Cascaded PID
%  Project : Indoor GPS-Denied Autonomous Drone (Minor Project 2025-26)
%  Week    : 1  (19 May – 25 May 2026)
%
%  HARDWARE (as ordered):
%    Frame   : F450  (450mm wheelbase, ~68cm tip-to-tip with 9" props)
%    Motors  : A2212 920KV × 4
%    Props   : 9045 CW/CCW pairs
%    Battery : 3S 2200mAh 25C LiPo (11.1V)
%    Sensors : RPLidar A1 + RealSense D435i + Pi Cam v2 + VL53L0X
%    AUW     : ~1,333g
%
%  SAFETY NOTE:
%    F450 + 9045 = 68cm tip-to-tip. Prop guards mandatory indoors.
%    Minimum test area: 3m x 3m clear.
%
%  PASS: altitude error < ±5cm for t > 5s, XY drift < 5cm
% =========================================================================
clear; clc; close all;
addpath(fileparts(mfilename('fullpath')));

fprintf('=== Stage S1: Quadrotor Dynamics + Cascaded PID ===\n');
fprintf('F450 | A2212 920KV | 9045 props | 3S 11.1V | AUW 1333g\n\n');

% =========================================================================
%  PHYSICAL PARAMETERS
% =========================================================================
p.m   = 1.333;      % AUW [kg]
%                     F450 282 + motors 240 + ESC 80 + Pixhawk 38
%                     + RPi5 46 + RPLidar 170 + RealSense 72
%                     + PiCam 25 + Arduino+sensors 65 + VL53L0X+ESP32 25
%                     + 3S battery 140 + wiring/misc 150
p.g   = 9.81;
p.L   = 0.225;      % arm length [m]  (F450: 450mm/2)
p.Ixx = 1.60e-2;    % roll  inertia [kg.m^2]  (F450 larger than F330)
p.Iyy = 1.65e-2;    % pitch inertia (RealSense D435i forward mount)
p.Izz = 2.90e-2;    % yaw   inertia

% A2212 920KV + 9045 on 3S (11.1V):
%   Max thrust ~800g per motor at ~7500 RPM loaded
%   kT derived from: F_max = kT * omega_max^2
%   kT = (0.800 * 9.81) / (785)^2 = 1.2723e-5
p.kT        = 1.2723e-5;   % N/(rad/s)^2
p.kD        = 2.5445e-7;   % N.m/(rad/s)^2  (kD = kT * 0.020 for 9" prop)
p.max_omega = 785;          % rad/s  (~7500 RPM max loaded on 3S)
p.min_omega = 80;

% Verify
omega_h = sqrt(p.m*p.g / (4*p.kT));
TW      = 4*p.kT*p.max_omega^2 / (p.m*p.g);
hover_pct = omega_h/p.max_omega*100;
fprintf('Hover speed    : %.0f rad/s (%.0f RPM, %.0f%% throttle)\n', ...
        omega_h, omega_h*60/(2*pi), hover_pct);
fprintf('T/W ratio      : %.2f\n', TW);
fprintf('Max thrust     : %.0f g total\n\n', 4*p.kT*p.max_omega^2/p.g*1000);
assert(TW > 1.8, 'T/W too low');

% =========================================================================
%  PID GAINS  (tuned for 1333g, F450 dynamics)
% =========================================================================
% Altitude Z  [ArduPilot: PSC_POSZ_P, PSC_VELZ_P/I/D]
gains.alt_Kp = 1.60;  gains.alt_Ki = 0.00;  gains.alt_Kd = 1.50;
gains.alt_max_vel = 1.0;   % conservative for heavier drone

% Horizontal XY  [PSC_POSXY_P, PSC_VELXY_P/I/D]
gains.pos_Kp = 1.00;  gains.pos_Ki = 0.05;  gains.pos_Kd = 0.30;
gains.pos_max_angle = deg2rad(12);   % slightly less tilt — heavier drone

% Attitude roll/pitch  [ATC_RAT_RLL/PIT _P/I/D]
gains.att_Kp = 6.50;  gains.att_Ki = 0.04;  gains.att_Kd = 1.20;

% Yaw  [ATC_RAT_YAW_P/I/D]
gains.yaw_Kp = 3.00;  gains.yaw_Ki = 0.02;  gains.yaw_Kd = 0.30;

% Anti-windup
gains.int_limit_alt = 0.5;
gains.int_limit_pos = 0.3;
gains.int_limit_att = 0.2;

% =========================================================================
%  SIMULATION
% =========================================================================
dt    = 0.002;
t_end = 30.0;
t_vec = 0:dt:t_end;
N     = length(t_vec);

sp.pos = [0; 0; 1.0];
sp.yaw = 0;

x       = zeros(12,1);
pid_mem = zeros(18,1);

log.t      = t_vec;
log.state  = zeros(12,N);
log.motors = zeros(4,N);
log.cmd    = zeros(4,N);
log.error  = zeros(6,N);

fprintf('Simulating %.0fs at %.0fHz...\n', t_end, 1/dt);
tic;
for k = 1:N
    log.state(:,k) = x;
    [omega4, cmd, pid_mem, err] = cascaded_pid(x, sp, gains, p, dt, pid_mem);
    omega4 = max(p.min_omega, min(p.max_omega, omega4));
    log.motors(:,k) = omega4;
    log.cmd(:,k)    = cmd;
    log.error(:,k)  = err;

    k1 = quadrotor_dynamics(x,          omega4, p);
    k2 = quadrotor_dynamics(x+dt/2*k1,  omega4, p);
    k3 = quadrotor_dynamics(x+dt/2*k2,  omega4, p);
    k4 = quadrotor_dynamics(x+dt*k3,    omega4, p);
    x  = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    x(6) = mod(x(6)+pi, 2*pi) - pi;
end
fprintf('Done in %.2fs\n\n', toc);

% =========================================================================
%  UNIT TEST
% =========================================================================
ss      = t_vec >= 5.0;
alt_e   = (log.state(3,:) - sp.pos(3)) * 100;
xy_e    = sqrt(log.state(1,:).^2 + log.state(2,:).^2) * 100;
max_alt = max(abs(alt_e(ss)));
max_xy  = max(xy_e(ss));

fprintf('=== UNIT TEST ===\n');
fprintf('  Altitude error (t>5s): %.2fcm  [pass: <5cm]\n', max_alt);
fprintf('  XY drift      (t>5s): %.2fcm  [pass: <5cm]\n', max_xy);
if max_alt<5.0 && max_xy<5.0
    fprintf('  *** PASS ***\n\n');
else
    fprintf('  *** FAIL *** — try: alt_Kp→1.4, alt_Kd→1.8\n\n');
end

% =========================================================================
%  ARDUCOPTER EXPORT
% =========================================================================
fprintf('=== ArduPilot params (Mission Planner, Week 4) ===\n');
ap = {'ATC_RAT_RLL_P',gains.att_Kp; 'ATC_RAT_RLL_I',gains.att_Ki;
      'ATC_RAT_RLL_D',gains.att_Kd; 'ATC_RAT_PIT_P',gains.att_Kp;
      'ATC_RAT_PIT_I',gains.att_Ki; 'ATC_RAT_PIT_D',gains.att_Kd;
      'ATC_RAT_YAW_P',gains.yaw_Kp; 'PSC_POSZ_P',   gains.alt_Kp;
      'PSC_POSXY_P',  gains.pos_Kp;  'GPS_TYPE',     0};
for i=1:size(ap,1)
    fprintf('  %-20s = %g\n', ap{i,1}, ap{i,2});
end
fprintf('\n');

plot_S1_results(log, sp, gains, p);

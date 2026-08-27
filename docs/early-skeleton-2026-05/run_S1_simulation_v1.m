% =========================================================================
%  run_S1_simulation.m  —  Stage S1: Quadrotor Dynamics + Cascaded PID
%  Project : Indoor GPS-Denied Autonomous Drone (Minor Project 2025-26)
%  Week    : 1  (19 May – 25 May 2026)
%
%  HARDWARE (updated — visual SLAM, no LiDAR, F330 frame):
%    Frame   : F330  (330 mm wheelbase, ~50 cm tip-to-tip)
%    Motors  : 2306 2450KV × 4
%    Props   : 5045 CW/CCW
%    Battery : 4S 2200mAh 25C LiPo
%    Sensors : OAK-D Lite (stereo SLAM) + Pi Cam v2 (optical flow)
%    AUW     : ~1,025 g
%
%  MathWorks resource:
%    UAV Toolbox — multirotor + uavScenario (used fully from S2 onward)
%    awesome-matlab-robotics → UAV section
%    mathworks.com/matlabcentral/fileexchange/68788
%
%  PASS criterion: altitude error < ±5 cm for t > 5 s, XY drift < 5 cm
% =========================================================================
clear; clc; close all;
addpath(fileparts(mfilename('fullpath')));

fprintf('=== Stage S1: Quadrotor Dynamics + Cascaded PID ===\n');
fprintf('F330 | 2306 2450KV | 5045 props | 4S | AUW 1025g\n\n');

% =========================================================================
%  PHYSICAL PARAMETERS
% =========================================================================
p.m   = 1.025;      % AUW [kg]
p.g   = 9.81;
p.L   = 0.165;      % arm length [m]  (330mm/2)
p.Ixx = 8.0e-3;     % roll  inertia [kg m^2]
p.Iyy = 8.5e-3;     % pitch inertia (OAK-D Lite shifts CG slightly fwd)
p.Izz = 1.45e-2;    % yaw   inertia

% 2306 2450KV + 5045 on 4S:
%   hover F_per_motor = 1.025*9.81/4 = 2.514 N at ~793 rad/s
%   kT = 2.514 / 793^2 = 4.0e-6 N/(rad/s)^2
%   kD/kT ratio for 5" prop ~0.016 -> kD = 6.4e-8
p.kT        = 4.00e-6;
p.kD        = 6.40e-8;
p.max_omega = 1200;     % rad/s (~11460 RPM on 4S)
p.min_omega = 100;

% Verify thrust budget
omega_h = sqrt(p.m*p.g / (4*p.kT));
TW      = 4*p.kT*p.max_omega^2 / (p.m*p.g);
fprintf('Hover speed  : %.0f rad/s (%.0f RPM)\n', omega_h, omega_h*60/(2*pi));
fprintf('T/W ratio    : %.2f\n\n', TW);
assert(TW > 1.8, 'T/W too low for safe indoor flight');

% =========================================================================
%  PID GAINS  (ArduPilot parameter names in comments)
% =========================================================================
% Altitude Z  [PSC_POSZ_P, PSC_VELZ_P/I/D]
gains.alt_Kp = 1.80;  gains.alt_Ki = 0.35;  gains.alt_Kd = 1.10;
gains.alt_max_vel = 1.2;

% Horizontal XY  [PSC_POSXY_P, PSC_VELXY_P/I/D]
gains.pos_Kp = 1.10;  gains.pos_Ki = 0.06;  gains.pos_Kd = 0.35;
gains.pos_max_angle = deg2rad(15);

% Attitude roll/pitch  [ATC_RAT_RLL/PIT _P/I/D]
gains.att_Kp = 7.50;  gains.att_Ki = 0.04;  gains.att_Kd = 1.40;

% Yaw  [ATC_RAT_YAW_P/I/D]
gains.yaw_Kp = 3.50;  gains.yaw_Ki = 0.02;  gains.yaw_Kd = 0.35;

% Anti-windup limits
gains.int_limit_alt = 0.5;
gains.int_limit_pos = 0.3;
gains.int_limit_att = 0.2;

% =========================================================================
%  SIMULATION LOOP
% =========================================================================
dt    = 0.002;   % 500 Hz (Pixhawk IMU at 1 kHz; we run controller at 500 Hz)
t_end = 30.0;
t_vec = 0:dt:t_end;
N     = length(t_vec);

sp.pos = [0; 0; 1.0];   % hover target [x;y;z] metres
sp.yaw = 0;

x       = zeros(12,1);  % [pos; euler; vel; omega] — starts on ground at rest
pid_mem = zeros(18,1);

log.t      = t_vec;
log.state  = zeros(12,N);
log.motors = zeros(4,N);
log.cmd    = zeros(4,N);
log.error  = zeros(6,N);

fprintf('Simulating %.0f s at %.0f Hz...\n', t_end, 1/dt);
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
fprintf('Done in %.2f s (%.0fx realtime)\n\n', toc, t_end/toc);

% =========================================================================
%  UNIT TEST
% =========================================================================
ss      = t_vec >= 5.0;
alt_e   = (log.state(3,:) - sp.pos(3)) * 100;
xy_e    = sqrt(log.state(1,:).^2 + log.state(2,:).^2) * 100;
max_alt = max(abs(alt_e(ss)));
max_xy  = max(xy_e(ss));

fprintf('=== UNIT TEST ===\n');
fprintf('  Altitude error (t>5s): %.2f cm  [pass: <5 cm]\n', max_alt);
fprintf('  XY drift      (t>5s): %.2f cm  [pass: <5 cm]\n', max_xy);
if max_alt<5.0 && max_xy<5.0
    fprintf('  *** PASS ***\n\n');
else
    fprintf('  *** FAIL *** — reduce gains by 10%% and retry\n\n');
end

% =========================================================================
%  ARDUCOPTER EXPORT  (paste into Mission Planner → Full Parameter List)
% =========================================================================
fprintf('=== ArduPilot params for Week 4 ===\n');
ap = {'ATC_RAT_RLL_P',gains.att_Kp; 'ATC_RAT_RLL_I',gains.att_Ki;
      'ATC_RAT_RLL_D',gains.att_Kd; 'ATC_RAT_PIT_P',gains.att_Kp;
      'ATC_RAT_PIT_I',gains.att_Ki; 'ATC_RAT_PIT_D',gains.att_Kd;
      'ATC_RAT_YAW_P',gains.yaw_Kp; 'PSC_POSZ_P',gains.alt_Kp;
      'PSC_POSXY_P',gains.pos_Kp};
for i=1:size(ap,1)
    fprintf('  %-20s = %.4f\n', ap{i,1}, ap{i,2});
end
fprintf('\n');

plot_S1_results(log, sp, gains, p);

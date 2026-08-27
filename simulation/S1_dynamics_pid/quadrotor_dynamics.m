function dxdt = quadrotor_dynamics(x, omega4, p)
% QUADROTOR_DYNAMICS  —  6-DOF rigid-body model for an X-configuration quadrotor
%
%  dxdt = quadrotor_dynamics(x, omega4, p)
%
%  Inputs:
%    x      (12×1)  State vector:
%                   [x, y, z,             — position in world frame [m]
%                    phi, theta, psi,      — roll, pitch, yaw (ZYX Euler) [rad]
%                    dx, dy, dz,           — linear velocity in world frame [m/s]
%                    dphi, dtheta, dpsi]   — angular velocity in body frame [rad/s]
%
%    omega4 (4×1)  Motor angular speeds [rad/s]
%                   Motor layout (X-config, top view):
%                       M1(CCW)  M2(CW)
%                          \      /
%                          /      \
%                       M4(CW)  M3(CCW)
%                  M1=front-left, M2=front-right, M3=rear-right, M4=rear-left
%
%    p      struct  Physical parameters (see run_S1_simulation.m)
%
%  Output:
%    dxdt   (12×1)  State derivative
%
%  Reference: Mahony, Mueller & D'Andrea (2012) "Multirotor Aerial Vehicles"
%             IEEE Robotics & Automation Magazine
% -------------------------------------------------------------------------

% Unpack state
pos   = x(1:3);     % [x; y; z]
eul   = x(4:6);     % [phi; theta; psi]  roll, pitch, yaw
vel   = x(7:9);     % [dx; dy; dz] in world frame
omega = x(10:12);   % [p; q; r] body angular rates (using p,q,r = dphi,dtheta,dpsi)

phi   = eul(1);
theta = eul(2);
psi   = eul(3);

% -------------------------------------------------------------------------
%  Motor thrusts and torques
%  F_i = kT * omega_i^2      (each motor produces a force along +z body)
%  Q_i = kD * omega_i^2      (reaction torque, sign depends on spin direction)
%  CCW motors (M1, M3): positive yaw reaction
%  CW  motors (M2, M4): negative yaw reaction
% -------------------------------------------------------------------------
F = p.kT .* omega4.^2;          % [N] thrust from each motor (4×1)
Q = p.kD .* omega4.^2;          % [N·m] torque from each motor (4×1)

T_total = sum(F);                % Total thrust along body +z axis

% Body-frame torques (X-config mixing matrix)
% tau_phi   (roll)  = L*(F4 + F1 - F2 - F3)   — L = arm length
% tau_theta (pitch) = L*(F1 + F2 - F3 - F4)   ← watch sign convention
% tau_psi   (yaw)   = Q1 - Q2 + Q3 - Q4       (CCW positive)
tau_phi   = p.L * ( F(1) - F(2) - F(3) + F(4) );
tau_theta = p.L * ( F(1) + F(2) - F(3) - F(4) );
tau_psi   =       ( Q(1) - Q(2) + Q(3) - Q(4) );

% -------------------------------------------------------------------------
%  Rotation matrix: body → world  (ZYX Euler convention)
%  R = Rz(psi) * Ry(theta) * Rx(phi)
% -------------------------------------------------------------------------
cphi = cos(phi);   sphi = sin(phi);
cth  = cos(theta); sth  = sin(theta);
cpsi = cos(psi);   spsi = sin(psi);

R = [ cpsi*cth,  cpsi*sth*sphi - spsi*cphi,  cpsi*sth*cphi + spsi*sphi;
      spsi*cth,  spsi*sth*sphi + cpsi*cphi,  spsi*sth*cphi - cpsi*sphi;
      -sth,      cth*sphi,                   cth*cphi                  ];

% -------------------------------------------------------------------------
%  Translational dynamics  (Newton, world frame)
%  m*ddpos = R * [0; 0; T_total] + [0; 0; -m*g]
% -------------------------------------------------------------------------
thrust_world = R * [0; 0; T_total];
ddpos = (thrust_world + [0; 0; -p.m * p.g]) / p.m;

% -------------------------------------------------------------------------
%  Euler angle kinematics
%  Relates body angular rates [p; q; r] to Euler angle derivatives
%  [dphi; dtheta; dpsi] = W_inv * [p; q; r]
%
%  W_inv = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
%           0, cos(phi),            -sin(phi);
%           0, sin(phi)/cos(theta), cos(phi)/cos(theta)]
%
%  Singularity at theta = ±90° (gimbal lock) — not an issue for small angles
% -------------------------------------------------------------------------
if abs(cth) < 1e-6
    cth = 1e-6;   % prevent division by zero near 90° pitch (shouldn't happen)
end

W_inv = [1,  sphi*sth/cth,  cphi*sth/cth;
         0,  cphi,          -sphi;
         0,  sphi/cth,       cphi/cth    ];

deul = W_inv * omega;

% -------------------------------------------------------------------------
%  Rotational dynamics  (Euler equations in body frame)
%  I * domega = tau - omega × (I * omega)
% -------------------------------------------------------------------------
I     = diag([p.Ixx, p.Iyy, p.Izz]);
tau   = [tau_phi; tau_theta; tau_psi];
domega = I \ (tau - cross(omega, I * omega));

% -------------------------------------------------------------------------
%  Assemble derivative
% -------------------------------------------------------------------------
dxdt = [vel;        % dpos/dt  = velocity
        deul;       % deul/dt  = Euler angle rates
        ddpos;      % dvel/dt  = acceleration
        domega];    % domega/dt= angular acceleration
end

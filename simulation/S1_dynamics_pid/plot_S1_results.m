function plot_S1_results(log, sp, gains, p)
% PLOT_S1_RESULTS  —  Generates the four report figures for Stage S1
%
%  Figure 1: 3D trajectory
%  Figure 2: Position & altitude vs time  (main unit-test plot)
%  Figure 3: Euler angles vs time
%  Figure 4: Motor speeds vs time  (check saturation)
%
%  All plots use a clean style matching the report template.
% -------------------------------------------------------------------------

t   = log.t;
st  = log.state;      % 12×N
mot = log.motors;     % 4×N

% Derived signals
alt_err_cm = (st(3,:) - sp.pos(3)) * 100;   % [cm]
xy_drift_cm = sqrt(st(1,:).^2 + st(2,:).^2) * 100;
rpm = mot * 60 / (2*pi);                     % convert rad/s → RPM

% Colour scheme (matches report figures)
c_pos  = [0.20 0.45 0.72];   % blue
c_alt  = [0.00 0.60 0.40];   % green
c_err  = [0.85 0.33 0.10];   % orange
c_mot  = [0.49 0.18 0.56;    % purple   M1
           0.30 0.60 0.90;   % light blue M2
           0.00 0.60 0.40;   % green    M3
           0.85 0.33 0.10];  % orange   M4

set(0, 'DefaultAxesFontSize', 11, ...
       'DefaultLineLineWidth', 1.4, ...
       'DefaultAxesBox', 'off', ...
       'DefaultFigureColor', 'w');

% =========================================================================
%  FIGURE 1: 3D Trajectory
% =========================================================================
figure(1); clf;
plot3(st(1,:), st(2,:), st(3,:), 'Color', c_pos, 'LineWidth', 1.6);
hold on;
plot3(sp.pos(1), sp.pos(2), sp.pos(3), 'k*', 'MarkerSize', 12, ...
      'DisplayName', 'Target (0, 0, 1 m)');
plot3(0, 0, 0, 'ko', 'MarkerSize', 8, 'DisplayName', 'Start');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Stage S1 — 3D trajectory (30 s hover)');
legend('Drone path', 'Target', 'Start', 'Location', 'best');
grid on; axis equal;
view(-40, 25);

% =========================================================================
%  FIGURE 2: Position & Altitude  (the Unit-Test figure for the report)
% =========================================================================
figure(2); clf;
t2 = tiledlayout(3, 1, 'TileSpacing', 'compact');
title(t2, 'Stage S1 — Position hold (Week 1 unit test)', ...
      'FontSize', 12, 'FontWeight', 'normal');

% --- Altitude
nexttile;
plot(t, st(3,:)*100, 'Color', c_alt, 'DisplayName', 'Altitude');
hold on;
yline(sp.pos(3)*100, '--k', 'LineWidth', 1, 'DisplayName', 'Target 1 m');
yline((sp.pos(3)+0.05)*100, ':r', 'LineWidth', 0.8, 'DisplayName', '±5 cm band');
yline((sp.pos(3)-0.05)*100, ':r', 'LineWidth', 0.8, 'HandleVisibility', 'off');
xline(5, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, ...
      'Label', 'Steady state ▶', 'LabelVerticalAlignment', 'bottom');
ylabel('Altitude [cm]'); ylim([-5 130]); grid on;
legend('Location', 'southeast'); xlim([0 30]);

% --- XY drift
nexttile;
plot(t, st(1,:)*100, 'Color', c_pos, 'DisplayName', 'x');
hold on;
plot(t, st(2,:)*100, 'Color', c_err, 'DisplayName', 'y');
yline( 5, ':r', 'LineWidth', 0.8, 'DisplayName', '±5 cm');
yline(-5, ':r', 'LineWidth', 0.8, 'HandleVisibility', 'off');
ylabel('XY position [cm]'); grid on; legend('Location', 'best'); xlim([0 30]);

% --- Altitude error
nexttile;
plot(t, alt_err_cm, 'Color', c_err, 'DisplayName', 'Altitude error');
hold on;
yline( 5, ':r', 'LineWidth', 0.8, 'Label', '+5 cm limit');
yline(-5, ':r', 'LineWidth', 0.8, 'Label', '-5 cm limit');
xline(5, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8);
xlabel('Time [s]'); ylabel('Error [cm]'); grid on; xlim([0 30]);

% Mark steady-state region
steady_idx = t >= 5;
ss_max = max(abs(alt_err_cm(steady_idx)));
annotation('textbox', [0.72 0.08 0.25 0.06], ...
    'String', sprintf('SS max error: %.2f cm', ss_max), ...
    'EdgeColor', 'k', 'BackgroundColor', 'w', 'FontSize', 10);

% =========================================================================
%  FIGURE 3: Euler Angles
% =========================================================================
figure(3); clf;
t3 = tiledlayout(3, 1, 'TileSpacing', 'compact');
title(t3, 'Stage S1 — Euler angles during hover', ...
      'FontSize', 12, 'FontWeight', 'normal');

angle_labels = {'Roll \phi [deg]', 'Pitch \theta [deg]', 'Yaw \psi [deg]'};
for i = 1:3
    nexttile;
    plot(t, rad2deg(st(3+i,:)), 'Color', c_pos);
    yline(0, '--k', 'LineWidth', 0.8);
    ylabel(angle_labels{i}); grid on; xlim([0 30]);
    if i < 3, xticklabels([]); end
end
xlabel('Time [s]');

% =========================================================================
%  FIGURE 4: Motor Speeds
% =========================================================================
figure(4); clf;
motor_names = {'M1 FL (CCW)', 'M2 FR (CW)', 'M3 RR (CCW)', 'M4 RL (CW)'};
hold on;
for i = 1:4
    plot(t, rpm(i,:), 'Color', c_mot(i,:), 'DisplayName', motor_names{i});
end

% Hover RPM reference line
omega_hover = sqrt((p.m * p.g) / (4 * p.kT));
hover_rpm   = omega_hover * 60 / (2*pi);
yline(hover_rpm, '--k', 'LineWidth', 1, 'Label', sprintf('Hover RPM = %.0f', hover_rpm));
yline(p.max_omega * 60 / (2*pi), ':r', 'Label', 'Max RPM', 'LineWidth', 1);

xlabel('Time [s]'); ylabel('Motor speed [RPM]');
title('Stage S1 — Motor speeds (check: no saturation in steady state)');
legend('Location', 'best'); grid on; xlim([0 30]);

% =========================================================================
%  CONSOLE SUMMARY
% =========================================================================
fprintf('\n--- Plot summary ---\n');
fprintf('  Figure 1: 3D trajectory\n');
fprintf('  Figure 2: Position hold (SAVE THIS for report — unit test evidence)\n');
fprintf('  Figure 3: Euler angles (should stay within ±15 deg)\n');
fprintf('  Figure 4: Motor RPM (should not saturate after t > 3 s)\n');
fprintf('\n  Save Figure 2 as: simulation/results/S1_position_hold.png\n');
fprintf('  Command: saveas(figure(2), ''../results/S1_position_hold.png'')\n\n');
end

clear;
close all;

tspan = [0 4*pi];
g = 9.81;
A = 0.4;
w = 1;

a = 0.5;
L = 0.20;
base_offset = 0.1;
l2 = 0.30;
theta2_home = asin(L/l2);
l1 = (a - base_offset) - l2*cos(theta2_home);

[theta1_home, theta2_home, ~, ~] = plateAngleToIK(0, a, L, l1, l2, base_offset);

x0 = 0;
y0 = 0;
x0dot = 0.3;
y0dot = 0;
phi_x0 = 0;
phi_y0 = 0;

Kp_x = 10;
Kd_x = 4;
Kp_y = 5.5;
Kd_y = 1.95;
Ki_x = 0.02;
Ki_y = 0.02;
Ix = 0;
Iy = 0;
Imax = 0.5;

dt = 0.001;
t = (tspan(1):dt:tspan(2))';
N = length(t);
Ts_control = 0.03;
control_steps = round(Ts_control/dt);

tau_servo = 0.03;

x = zeros(N,1);
y = zeros(N,1);
xdot = zeros(N,1);
ydot = zeros(N,1);
phi_x = zeros(N,1);
phi_y = zeros(N,1);
phi_dot_x = zeros(N,1);
phi_dot_y = zeros(N,1);
phi_cmd_x = zeros(N,1);
phi_cmd_y = zeros(N,1);
xddot = zeros(N,1);
yddot = zeros(N,1);
theta1_x = zeros(N,1);
theta2_x = zeros(N,1);
theta1_y = zeros(N,1);
theta2_y = zeros(N,1);
theta1_cmd_x = zeros(N,1);
theta2_cmd_x = zeros(N,1);
theta1_cmd_y = zeros(N,1);
theta2_cmd_y = zeros(N,1);
theta1_dot_x_log = zeros(1, N); theta2_dot_x_log = zeros(1, N);
theta1_dot_y_log = zeros(1, N); theta2_dot_y_log = zeros(1, N);

sat_pts_x1 = NaN(1, N); sat_pts_x2 = NaN(1, N);
sat_pts_y1 = NaN(1, N); sat_pts_y2 = NaN(1, N);

control_effort_x1 = 0;
control_effort_x2 = 0;
control_effort_y1 = 0;
control_effort_y2 = 0;
theta_dot_limit = deg2rad(60);
sat_count_x1 = 0;
sat_count_x2 = 0;
sat_count_y1 = 0;
sat_count_y2 = 0;

x(1) = x0;
y(1) = y0;
xdot(1) = x0dot;
ydot(1) = y0dot;
phi_x(1) = phi_x0;
phi_y(1) = phi_y0;

phi_cmd_x_current = phi_x0;
phi_cmd_y_current = phi_y0;
theta1_cmd_x_current = 0;
theta2_cmd_x_current = 0;
theta1_cmd_y_current = 0;
theta2_cmd_y_current = 0;

options = optimoptions('fsolve', 'Display','off');

for k = 2:N-1
    if mod(k-2, control_steps) == 0
        xref = A*sin(w*t(k));
        yref = 0.5*A*sin(2*w*t(k));
        xdot_ref = A*w*cos(w*t(k));
        ydot_ref = A*w*cos(2*w*t(k));
        xdotdot_ref = -A*w^2*sin(w*t(k));
        ydotdot_ref = -2*A*w^2*sin(2*w*t(k));

        ex = x(k-1) - xref;
        ey = y(k-1) - yref;

        Ix = Ix + ex*Ts_control;
        Iy = Iy + ey*Ts_control;
        Ix = max(min(Ix, Imax), -Imax);
        Iy = max(min(Iy, Imax), -Imax);

        exdot = xdot(k-1) - xdot_ref;
        eydot = ydot(k-1) - ydot_ref;

        xdotdot_des = xdotdot_ref - Kd_x*exdot - Kp_x*ex - Ki_x*Ix;
        ydotdot_des = ydotdot_ref - Kd_y*eydot - Kp_y*ey - Ki_y*Iy;

        z0 = [phi_cmd_x_current; phi_cmd_y_current];
        z_sol = fsolve(@(z) inverse_residual_discrete( ...
            z, x(k-1), y(k-1), phi_x(k-1), phi_y(k-1), Ts_control, g, ...
            xdotdot_des, ydotdot_des), z0, options);

        phi_cmd_x_current = z_sol(1);
        phi_cmd_y_current = z_sol(2);

        [theta1_cmd_abs_x, theta2_cmd_abs_x, ~, ~] = ...
            plateAngleToIK(phi_cmd_x_current, a, L, l1, l2, base_offset);
        [theta1_cmd_abs_y, theta2_cmd_abs_y, ~, ~] = ...
            plateAngleToIK(phi_cmd_y_current, a, L, l1, l2, base_offset);

        theta1_cmd_x_current = theta1_cmd_abs_x - theta1_home;
        theta2_cmd_x_current = theta2_cmd_abs_x - theta2_home;
        theta1_cmd_y_current = theta1_cmd_abs_y - theta1_home;
        theta2_cmd_y_current = theta2_cmd_abs_y - theta2_home;
    end

    phi_cmd_x(k) = phi_cmd_x_current;
    phi_cmd_y(k) = phi_cmd_y_current;
    theta1_cmd_x(k) = theta1_cmd_x_current;
    theta2_cmd_x(k) = theta2_cmd_x_current;
    theta1_cmd_y(k) = theta1_cmd_y_current;
    theta2_cmd_y(k) = theta2_cmd_y_current;

    desired_theta1_dot_x = (theta1_cmd_x_current - theta1_x(k-1)) / tau_servo;
    desired_theta2_dot_x = (theta2_cmd_x_current - theta2_x(k-1)) / tau_servo;
    desired_theta1_dot_y = (theta1_cmd_y_current - theta1_y(k-1)) / tau_servo;
    desired_theta2_dot_y = (theta2_cmd_y_current - theta2_y(k-1)) / tau_servo;

    theta1_dot_x = max(min(desired_theta1_dot_x, theta_dot_limit), -theta_dot_limit);
    theta2_dot_x = max(min(desired_theta2_dot_x, theta_dot_limit), -theta_dot_limit);
    theta1_dot_y = max(min(desired_theta1_dot_y, theta_dot_limit), -theta_dot_limit);
    theta2_dot_y = max(min(desired_theta2_dot_y, theta_dot_limit), -theta_dot_limit);

    control_effort_x1 = control_effort_x1 + (theta1_dot_x^2) * dt;
    control_effort_x2 = control_effort_x2 + (theta2_dot_x^2) * dt;
    control_effort_y1 = control_effort_y1 + (theta1_dot_y^2) * dt;
    control_effort_y2 = control_effort_y2 + (theta2_dot_y^2) * dt;

    if abs(desired_theta1_dot_x) >= theta_dot_limit
        sat_count_x1 = sat_count_x1 + 1;
        sat_pts_x1(k) = theta1_dot_x;
    end

    if abs(desired_theta2_dot_x) >= theta_dot_limit
        sat_count_x2 = sat_count_x2 + 1;
        sat_pts_x2(k) = theta2_dot_x;
    end

    if abs(desired_theta1_dot_y) >= theta_dot_limit
        sat_count_y1 = sat_count_y1 + 1;
        sat_pts_y1(k) = theta1_dot_y;
    end

    if abs(desired_theta2_dot_y) >= theta_dot_limit
        sat_count_y2 = sat_count_y2 + 1;
        sat_pts_y2(k) = theta2_dot_y;
    end

    theta1_dot_x_log(k) = theta1_dot_x;
    theta2_dot_x_log(k) = theta2_dot_x;
    theta1_dot_y_log(k) = theta1_dot_y;
    theta2_dot_y_log(k) = theta2_dot_y;

    theta1_x(k) = theta1_x(k-1) + theta1_dot_x*dt;
    theta2_x(k) = theta2_x(k-1) + theta2_dot_x*dt;
    theta1_y(k) = theta1_y(k-1) + theta1_dot_y*dt;
    theta2_y(k) = theta2_y(k-1) + theta2_dot_y*dt;

    theta1_abs_x = theta1_x(k) + theta1_home;
    theta2_abs_x = theta2_x(k) + theta2_home;
    theta1_abs_y = theta1_y(k) + theta1_home;
    theta2_abs_y = theta2_y(k) + theta2_home;

    [phi_x(k), ~, ~] = thetaToPlateAngle(theta1_abs_x, theta2_abs_x, l1, l2, base_offset, L);
    [phi_y(k), ~, ~] = thetaToPlateAngle(theta1_abs_y, theta2_abs_y, l1, l2, base_offset, L);

    phi_dot_x(k) = (phi_x(k) - phi_x(k-1))/dt;
    phi_dot_y(k) = (phi_y(k) - phi_y(k-1))/dt;

    xddot(k) = 5/7 * ( ...
          x(k-1)*phi_dot_x(k)^2 ...
        + y(k-1)*phi_dot_x(k)*phi_dot_y(k) ...
        - g*sin(phi_x(k)));
    yddot(k) = 5/7 * ( ...
          y(k-1)*phi_dot_y(k)^2 ...
        + x(k-1)*phi_dot_x(k)*phi_dot_y(k) ...
        - g*sin(phi_y(k)));

    xdot(k) = xdot(k-1) + xddot(k)*dt;
    ydot(k) = ydot(k-1) + yddot(k)*dt;
    x(k) = x(k-1) + xdot(k)*dt;
    y(k) = y(k-1) + ydot(k)*dt;
end

x(end) = x(end-1);
y(end) = y(end-1);
xdot(end) = xdot(end-1);
ydot(end) = ydot(end-1);
phi_x(end) = phi_x(end-1);
phi_y(end) = phi_y(end-1);
phi_cmd_x(end) = phi_cmd_x(end-1);
phi_cmd_y(end) = phi_cmd_y(end-1);
theta1_x(end) = theta1_x(end-1);
theta2_x(end) = theta2_x(end-1);
theta1_y(end) = theta1_y(end-1);
theta2_y(end) = theta2_y(end-1);
theta1_cmd_x(end) = theta1_cmd_x(end-1);
theta2_cmd_x(end) = theta2_cmd_x(end-1);
theta1_cmd_y(end) = theta1_cmd_y(end-1);
theta2_cmd_y(end) = theta2_cmd_y(end-1);

X = [x, y, xdot, ydot, phi_x, phi_y];
t_max = t(end);
max_pi_multiplier = floor(t_max / pi);
pi_ticks = 0:pi:(max_pi_multiplier * pi);

if abs(pi_ticks(end) - t_max) > 1e-4
    pi_ticks = [pi_ticks, t_max];
end

pi_labels = cell(1, length(pi_ticks));
for i = 1:length(pi_ticks)
    val = pi_ticks(i);
    multiplier = round(val / pi * 100) / 100;

    if val == 0
        pi_labels{i} = '0';
    elseif multiplier == 1
        pi_labels{i} = '\pi';
    elseif floor(multiplier) == multiplier
        pi_labels{i} = [num2str(multiplier), '\pi'];
    else
        pi_labels{i} = [num2str(multiplier), '\pi'];
    end
end

idx_half = round(length(t) / 2);

arrow_scale = 0.088;
xref_plot = A*sin(w*t);
yref_plot = 0.5*A*sin(2*w*t);

figure('Color','w','Position',[100 100 1000 450]);

tiledlayout(1,2,'TileSpacing','compact','Padding','tight');

fontSizeAxis  = 12;
fontSizeLabel = 14;
fontSizeTitle = 13;
fontSizeLeg   = 11;

ax1 = nexttile;

plot(xref_plot(1:idx_half), yref_plot(1:idx_half), 'k--', 'LineWidth', 1.2);
hold on;
plot(X(1:idx_half,1), X(1:idx_half,2), 'b-', 'LineWidth', 2);

arrow_idx_1 = round(linspace(idx_half*0.1, idx_half*0.9, 5));

for idx = arrow_idx_1
    dx = X(idx+1,1) - X(idx,1);
    dy = X(idx+1,2) - X(idx,2);

    len = sqrt(dx^2 + dy^2);

    if len > 0
        dx_norm = (dx / len) * arrow_scale;
        dy_norm = (dy / len) * arrow_scale;

        quiver(X(idx,1), X(idx,2), dx_norm, dy_norm, 0, 'k', ...
               'LineWidth', 1.5, ...
               'MaxHeadSize', 1.5, ...
               'AutoScale', 'off');
    end
end

grid on;
axis equal;
xlim([-0.55 0.55]);
ylim([-0.55 0.55]);

xlabel("x Position [m]", 'FontSize', fontSizeLabel);
ylabel("y Position [m]", 'FontSize', fontSizeLabel);
title('Cycle 1: Transient Response (0 \rightarrow 2\pi rad)', ...
      'FontSize', fontSizeTitle);

legend('Reference', 'Actual Tracking', ...
       'Location', 'southwest', ...
       'FontSize', fontSizeLeg);

set(ax1, 'FontSize', fontSizeAxis, 'LineWidth', 1);

ax2 = nexttile;

plot(xref_plot(idx_half:end), yref_plot(idx_half:end), 'k--', 'LineWidth', 1.2);
hold on;
plot(X(idx_half:end,1), X(idx_half:end,2), ...
     'Color', [0, 0.5, 0], 'LineWidth', 2);

arrow_idx_2 = round(linspace(idx_half + (length(t)-idx_half)*0.1, length(t)-5, 5));

for idx = arrow_idx_2
    dx = X(idx+1,1) - X(idx,1);
    dy = X(idx+1,2) - X(idx,2);

    len = sqrt(dx^2 + dy^2);

    if len > 0
        dx_norm = (dx / len) * arrow_scale;
        dy_norm = (dy / len) * arrow_scale;

        quiver(X(idx,1), X(idx,2), dx_norm, dy_norm, 0, 'k', ...
               'LineWidth', 1.5, ...
               'MaxHeadSize', 1.5, ...
               'AutoScale', 'off');
    end
end

grid on;
axis equal;
xlim([-0.55 0.55]);
ylim([-0.55 0.55]);

xlabel("x Position [m]", 'FontSize', fontSizeLabel);
ylabel("y Position [m]", 'FontSize', fontSizeLabel);
title('Cycle 2: Steady-State Tracking (2\pi \rightarrow 4\pi rad)', ...
      'FontSize', fontSizeTitle);

legend('Reference', 'Actual Tracking', ...
       'Location', 'southwest', ...
       'FontSize', fontSizeLeg);

set(ax2, 'FontSize', fontSizeAxis, 'LineWidth', 1);

figure('Color', 'w', ...
       'Name', 'Ball Trajectory Timelines and Plate Tilt Angles', ...
       'Position', [100 100 1000 650]);

tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

fontSizeAxis  = 11;
fontSizeLabel = 12;
fontSizeTitle = 12;
fontSizeLeg   = 10;

ax1 = nexttile;
plot(t, X(:,1), 'LineWidth', 2); hold on;
plot(t, xref_plot, '--', 'LineWidth', 2);
grid on;
legend("x","x_{ref}", 'Location', 'best', 'FontSize', fontSizeLeg);
ylabel("x Position [m]", 'FontSize', fontSizeLabel);
title("X Tracking Performance", 'FontSize', fontSizeTitle);
xlim([0 t_max]);
xticks(pi_ticks);
xticklabels(pi_labels);
set(ax1, 'FontSize', fontSizeAxis, 'LineWidth', 1);

ax2 = nexttile;
plot(t, X(:,2), 'LineWidth', 2); hold on;
plot(t, yref_plot, '--', 'LineWidth', 2);
grid on;
legend("y","y_{ref}", 'Location', 'best', 'FontSize', fontSizeLeg);
ylabel("y Position [m]", 'FontSize', fontSizeLabel);
title("Y Tracking Performance", 'FontSize', fontSizeTitle);
xlim([0 t_max]);
xticks(pi_ticks);
xticklabels(pi_labels);
set(ax2, 'FontSize', fontSizeAxis, 'LineWidth', 1);

ax3 = nexttile;
plot(t, rad2deg(X(:,5)), 'LineWidth', 2, 'Color', [0.2 0.6 0.8]);
grid on;
legend("\phi_x", 'Location', 'best', 'FontSize', fontSizeLeg);
xlabel("Time [rad]", 'FontSize', fontSizeLabel);
ylabel("Plate Angle [deg]", 'FontSize', fontSizeLabel);
title("Plate Angle \phi_x vs Time", 'FontSize', fontSizeTitle);
xlim([0 t_max]);
xticks(pi_ticks);
xticklabels(pi_labels);
set(ax3, 'FontSize', fontSizeAxis, 'LineWidth', 1);

ax4 = nexttile;
plot(t, rad2deg(X(:,6)), 'LineWidth', 2, 'Color', [0.8 0.4 0.2]);
grid on;
legend("\phi_y", 'Location', 'best', 'FontSize', fontSizeLeg);
xlabel("Time [rad]", 'FontSize', fontSizeLabel);
ylabel("Plate Angle [deg]", 'FontSize', fontSizeLabel);
title("Plate Angle \phi_y vs Time", 'FontSize', fontSizeTitle);
xlim([0 t_max]);
xticks(pi_ticks);
xticklabels(pi_labels);
set(ax4, 'FontSize', fontSizeAxis, 'LineWidth', 1);

figure('Color', 'w', ...
       'Name', 'Servo Actuator Comprehensive Analysis (Angles and Velocities)', ...
       'Position', [100 100 1050 900]);

tiledlayout(4,2,'TileSpacing','compact','Padding','compact');

fontSizeAxis  = 9;
fontSizeLabel = 10;
fontSizeTitle = 10;
fontSizeLeg   = 7;

ax1 = nexttile;
plot(t, rad2deg(theta1_cmd_x), '--', 'LineWidth', 2); hold on;
plot(t, rad2deg(theta1_x), 'LineWidth', 2);
grid on;
legend("\theta_{1x,cmd}", "\theta_{1x,actual}", 'Location', 'best', 'FontSize', fontSizeLeg);
ylabel("Angle [deg]", 'FontSize', fontSizeLabel);
title("Theta 1 X: Command vs Actual", 'FontSize', fontSizeTitle);
xlim([0 t_max]); xticks(pi_ticks); xticklabels(pi_labels);
set(ax1, 'FontSize', fontSizeAxis, 'LineWidth', 1);

ax2 = nexttile;
plot(t, theta1_dot_x_log, 'LineWidth', 1.5, 'Color', [0.1 0.5 0.3]); hold on;
yline(theta_dot_limit, 'r--', 'LineWidth', 1.2);
yline(-theta_dot_limit, 'r--', 'LineWidth', 1.2);
plot(t, sat_pts_x1, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 4);
grid on;
ylabel('Velocity [rad/s]', 'FontSize', fontSizeLabel);
title(['\theta_1 Dot X - Saturation Steps: ', num2str(sat_count_x1)], 'FontSize', fontSizeTitle);
xlim([0 t_max]); xticks(pi_ticks); xticklabels(pi_labels);
set(ax2, 'FontSize', fontSizeAxis, 'LineWidth', 1);

ax3 = nexttile;
plot(t, rad2deg(theta2_cmd_x), '--', 'LineWidth', 2); hold on;
plot(t, rad2deg(theta2_x), 'LineWidth', 2);
grid on;
legend("\theta_{2x,cmd}", "\theta_{2x,actual}", 'Location', 'best', 'FontSize', fontSizeLeg);
ylabel("Angle [deg]", 'FontSize', fontSizeLabel);
title("Theta 2 X: Command vs Actual", 'FontSize', fontSizeTitle);
xlim([0 t_max]); xticks(pi_ticks); xticklabels(pi_labels);
set(ax3, 'FontSize', fontSizeAxis, 'LineWidth', 1);

ax4 = nexttile;
plot(t, theta2_dot_x_log, 'LineWidth', 1.5, 'Color', [0.1 0.4 0.6]); hold on;
yline(theta_dot_limit, 'r--', 'LineWidth', 1.2);
yline(-theta_dot_limit, 'r--', 'LineWidth', 1.2);
plot(t, sat_pts_x2, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 4);
grid on;
ylabel('Velocity [rad/s]', 'FontSize', fontSizeLabel);
title(['\theta_2 Dot X - Saturation Steps: ', num2str(sat_count_x2)], 'FontSize', fontSizeTitle);
xlim([0 t_max]); xticks(pi_ticks); xticklabels(pi_labels);
set(ax4, 'FontSize', fontSizeAxis, 'LineWidth', 1);

ax5 = nexttile;
plot(t, rad2deg(theta1_cmd_y), '--', 'LineWidth', 2); hold on;
plot(t, rad2deg(theta1_y), 'LineWidth', 2);
grid on;
legend("\theta_{1y,cmd}", "\theta_{1y,actual}", 'Location', 'best', 'FontSize', fontSizeLeg);
ylabel("Angle [deg]", 'FontSize', fontSizeLabel);
title("Theta 1 Y: Command vs Actual", 'FontSize', fontSizeTitle);
xlim([0 t_max]); xticks(pi_ticks); xticklabels(pi_labels);
set(ax5, 'FontSize', fontSizeAxis, 'LineWidth', 1);

ax6 = nexttile;
plot(t, theta1_dot_y_log, 'LineWidth', 1.5, 'Color', [0.5 0.2 0.5]); hold on;
yline(theta_dot_limit, 'r--', 'LineWidth', 1.2);
yline(-theta_dot_limit, 'r--', 'LineWidth', 1.2);
plot(t, sat_pts_y1, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 4);
grid on;
ylabel('Velocity [rad/s]', 'FontSize', fontSizeLabel);
title(['\theta_1 Dot Y - Saturation Steps: ', num2str(sat_count_y1)], 'FontSize', fontSizeTitle);
xlim([0 t_max]); xticks(pi_ticks); xticklabels(pi_labels);
set(ax6, 'FontSize', fontSizeAxis, 'LineWidth', 1);

ax7 = nexttile;
plot(t, rad2deg(theta2_cmd_y), '--', 'LineWidth', 2); hold on;
plot(t, rad2deg(theta2_y), 'LineWidth', 2);
grid on;
legend("\theta_{2y,cmd}", "\theta_{2y,actual}", 'Location', 'best', 'FontSize', fontSizeLeg);
xlabel("Time [rad]", 'FontSize', fontSizeLabel);
ylabel("Angle [deg]", 'FontSize', fontSizeLabel);
title("Theta 2 Y: Command vs Actual", 'FontSize', fontSizeTitle);
xlim([0 t_max]); xticks(pi_ticks); xticklabels(pi_labels);
set(ax7, 'FontSize', fontSizeAxis, 'LineWidth', 1);

ax8 = nexttile;
plot(t, theta2_dot_y_log, 'LineWidth', 1.5, 'Color', [0.6 0.3 0.1]); hold on;
yline(theta_dot_limit, 'r--', 'LineWidth', 1.2);
yline(-theta_dot_limit, 'r--', 'LineWidth', 1.2);
plot(t, sat_pts_y2, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 4);
grid on;
xlabel("Time [rad]", 'FontSize', fontSizeLabel);
ylabel('Velocity [rad/s]', 'FontSize', fontSizeLabel);
title(['\theta_2 Dot Y - Saturation Steps: ', num2str(sat_count_y2)], 'FontSize', fontSizeTitle);
xlim([0 t_max]); xticks(pi_ticks); xticklabels(pi_labels);
set(ax8, 'FontSize', fontSizeAxis, 'LineWidth', 1);

ex = X(:,1) - xref_plot;
ey = X(:,2) - yref_plot;
idx = t > 2;
rms_error_x = sqrt(mean(ex(idx).^2));
rms_error_y = sqrt(mean(ey(idx).^2));

theta1_dot_x_hist = gradient(theta1_x, t);
theta2_dot_x_hist = gradient(theta2_x, t);
theta1_dot_y_hist = gradient(theta1_y, t);
theta2_dot_y_hist = gradient(theta2_y, t);
servo_activity_x = trapz(t, theta1_dot_x_hist.^2 + theta2_dot_x_hist.^2);
servo_activity_y = trapz(t, theta1_dot_y_hist.^2 + theta2_dot_y_hist.^2);

total_sim_steps = length(t);
sat_percentage_x1 = (sat_count_x1 / total_sim_steps) * 100;
sat_percentage_x2 = (sat_count_x2 / total_sim_steps) * 100;
sat_percentage_y1 = (sat_count_y1 / total_sim_steps) * 100;
sat_percentage_y2 = (sat_count_y2 / total_sim_steps) * 100;

total_rms_error = sqrt(mean((x - xref_plot).^2 + (y - yref_plot).^2));
fprintf('--- TUNING --- Total RMS: %.4f m | Kp_x: %.2f | Kd_x: %.2f | Ki_x: %.2f || Kp_y: %.2f | Kd_y: %.2f | Ki_y: %.2f\n', total_rms_error, Kp_x, Kd_x, Ki_x, Kp_y, Kd_y, Ki_y);

fprintf("\n===== X DOF Metrics =====\n");
fprintf("RMS tracking error x: %.6f m\n", rms_error_x);
fprintf('X-Axis Motor 1 (Omuz) Effort: %f\n', control_effort_x1);
fprintf('X-Axis Motor 2 (Dirsek) Effort: %f\n', control_effort_x2);
fprintf("\n===== Y DOF Metrics =====\n");
fprintf("RMS tracking error y: %.6f m\n", rms_error_y);
fprintf('Y-Axis Motor 1 (Omuz) Effort: %f\n', control_effort_y1);
fprintf('Y-Axis Motor 2 (Dirsek) Effort: %f\n', control_effort_y2);
fprintf("\n===== SATURATION (RATE LIMIT) METRICS =====\n");
fprintf("X Axis Motor 1 Saturation: %d steps (%.2f%% of total time)\n", sat_count_x1, sat_percentage_x1);
fprintf("X Axis Motor 2 Saturation: %d steps (%.2f%% of total time)\n", sat_count_x2, sat_percentage_x2);
fprintf("Y Axis Motor 1 Saturation: %d steps (%.2f%% of total time)\n", sat_count_y1, sat_percentage_y1);
fprintf("Y Axis Motor 2 Saturation: %d steps (%.2f%% of total time)\n", sat_count_y2, sat_percentage_y2);

launch3DAnimationPlayer(t, X, theta1_x, theta2_x, theta1_y, theta2_y, ...
    a, L, l1, l2, base_offset);

function F = inverse_residual_discrete(z,x,y,phi_x_prev,phi_y_prev,dt,g,xdotdot_des,ydotdot_des)
    phi_cmd_x = z(1);
    phi_cmd_y = z(2);
    phi_dot_x = (phi_cmd_x - phi_x_prev)/dt;
    phi_dot_y = (phi_cmd_y - phi_y_prev)/dt;
    xdotdot_model = 5/7 * ( ...
          x*phi_dot_x^2 ...
        + y*phi_dot_x*phi_dot_y ...
        - g*sin(phi_cmd_x));
    ydotdot_model = 5/7 * ( ...
          y*phi_dot_y^2 ...
        + x*phi_dot_x*phi_dot_y ...
        - g*sin(phi_cmd_y));
    F = [xdotdot_model - xdotdot_des;
         ydotdot_model - ydotdot_des];
end

function [theta1, theta2, x2, y2] = plateAngleToIK(phi, a, L, l1, l2, base_offset)
    x2 = a*cos(phi);
    y2 = L + a*sin(phi);
    x_rel = x2 - base_offset;
    y_rel = y2;
    D = (x_rel^2 + y_rel^2 - l1^2 - l2^2)/(2*l1*l2);
    D = max(min(D,1),-1);
    theta2 = acos(D);
    theta1 = atan2(y_rel,x_rel) - atan2(l2*sin(theta2), l1 + l2*cos(theta2));
end

function [phi, x2, y2] = thetaToPlateAngle(theta1, theta2, l1, l2, base_offset, L)
    x2 = base_offset + l1*cos(theta1) + l2*cos(theta1 + theta2);
    y2 = l1*sin(theta1) + l2*sin(theta1 + theta2);
    phi = atan2(y2 - L, x2);
end

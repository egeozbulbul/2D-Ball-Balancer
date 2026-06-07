clear;
clc;
close all;

tspan = [0 4*pi];
g = 9.81;
A = 0.4;
w = 1;

a = 0.5;
L = 0.20;
base_offset = 0.1;
l2 = 0.30;
theta2_home_temp = asin(L/l2);
l1 = (a - base_offset) - l2*cos(theta2_home_temp);

[theta1_home, theta2_home, ~, ~] = plateAngleToIK(0, a, L, l1, l2, base_offset);

x0 = 0;
y0 = 0;
x0dot = 0.3;
y0dot = 0;
phi_x0 = 0;
phi_y0 = 0;

Kp_x = 10;
Kd_x = 4;
Ki_x = 0.02;

Kp_y = 5.5;
Kd_y = 1.95;
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
theta_dot_limit = deg2rad(60);

x = zeros(N,1);
y = zeros(N,1);
xdot = zeros(N,1);
ydot = zeros(N,1);

phi_x = zeros(N,1);
phi_y = zeros(N,1);
phi_dot_x = zeros(N,1);
phi_dot_y = zeros(N,1);

theta1_x = zeros(N,1);
theta2_x = zeros(N,1);
theta1_y = zeros(N,1);
theta2_y = zeros(N,1);

xddot_des_log = NaN(N,1);
yddot_des_log = NaN(N,1);

xddot_rec_log = NaN(N,1);
yddot_rec_log = NaN(N,1);

res_x_log = NaN(N,1);
res_y_log = NaN(N,1);
res_norm_log = NaN(N,1);

phi_cmd_x_log = NaN(N,1);
phi_cmd_y_log = NaN(N,1);

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

options = optimoptions('fsolve', ...
    'Display', 'off', ...
    'FunctionTolerance', 1e-12, ...
    'StepTolerance', 1e-12);

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

        exdot = xdot(k-1) - xdot_ref;
        eydot = ydot(k-1) - ydot_ref;

        Ix = Ix + ex*Ts_control;
        Iy = Iy + ey*Ts_control;

        Ix = max(min(Ix, Imax), -Imax);
        Iy = max(min(Iy, Imax), -Imax);

        xdotdot_des = xdotdot_ref - Kd_x*exdot - Kp_x*ex - Ki_x*Ix;
        ydotdot_des = ydotdot_ref - Kd_y*eydot - Kp_y*ey - Ki_y*Iy;

        z0 = [phi_cmd_x_current; phi_cmd_y_current];

        z_sol = fsolve(@(z) inverse_residual_discrete( ...
            z, x(k-1), y(k-1), phi_x(k-1), phi_y(k-1), ...
            Ts_control, g, xdotdot_des, ydotdot_des), z0, options);

        phi_cmd_x_current = z_sol(1);
        phi_cmd_y_current = z_sol(2);

        phi_dot_cmd_x = (phi_cmd_x_current - phi_x(k-1)) / Ts_control;
        phi_dot_cmd_y = (phi_cmd_y_current - phi_y(k-1)) / Ts_control;

        xddot_rec = 5/7 * ( ...
              x(k-1)*phi_dot_cmd_x^2 ...
            + y(k-1)*phi_dot_cmd_x*phi_dot_cmd_y ...
            - g*sin(phi_cmd_x_current));

        yddot_rec = 5/7 * ( ...
              y(k-1)*phi_dot_cmd_y^2 ...
            + x(k-1)*phi_dot_cmd_x*phi_dot_cmd_y ...
            - g*sin(phi_cmd_y_current));

        res_x = xddot_rec - xdotdot_des;
        res_y = yddot_rec - ydotdot_des;
        res_norm = norm([res_x; res_y]);

        xddot_des_log(k) = xdotdot_des;
        yddot_des_log(k) = ydotdot_des;

        xddot_rec_log(k) = xddot_rec;
        yddot_rec_log(k) = yddot_rec;

        res_x_log(k) = res_x;
        res_y_log(k) = res_y;
        res_norm_log(k) = res_norm;

        phi_cmd_x_log(k) = phi_cmd_x_current;
        phi_cmd_y_log(k) = phi_cmd_y_current;

        [theta1_cmd_abs_x, theta2_cmd_abs_x, ~, ~] = ...
            plateAngleToIK(phi_cmd_x_current, a, L, l1, l2, base_offset);

        [theta1_cmd_abs_y, theta2_cmd_abs_y, ~, ~] = ...
            plateAngleToIK(phi_cmd_y_current, a, L, l1, l2, base_offset);

        theta1_cmd_x_current = theta1_cmd_abs_x - theta1_home;
        theta2_cmd_x_current = theta2_cmd_abs_x - theta2_home;

        theta1_cmd_y_current = theta1_cmd_abs_y - theta1_home;
        theta2_cmd_y_current = theta2_cmd_abs_y - theta2_home;
    end

    desired_theta1_dot_x = (theta1_cmd_x_current - theta1_x(k-1)) / tau_servo;
    desired_theta2_dot_x = (theta2_cmd_x_current - theta2_x(k-1)) / tau_servo;

    desired_theta1_dot_y = (theta1_cmd_y_current - theta1_y(k-1)) / tau_servo;
    desired_theta2_dot_y = (theta2_cmd_y_current - theta2_y(k-1)) / tau_servo;

    theta1_dot_x = max(min(desired_theta1_dot_x, theta_dot_limit), -theta_dot_limit);
    theta2_dot_x = max(min(desired_theta2_dot_x, theta_dot_limit), -theta_dot_limit);

    theta1_dot_y = max(min(desired_theta1_dot_y, theta_dot_limit), -theta_dot_limit);
    theta2_dot_y = max(min(desired_theta2_dot_y, theta_dot_limit), -theta_dot_limit);

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

    xddot_actual = 5/7 * ( ...
          x(k-1)*phi_dot_x(k)^2 ...
        + y(k-1)*phi_dot_x(k)*phi_dot_y(k) ...
        - g*sin(phi_x(k)));

    yddot_actual = 5/7 * ( ...
          y(k-1)*phi_dot_y(k)^2 ...
        + x(k-1)*phi_dot_x(k)*phi_dot_y(k) ...
        - g*sin(phi_y(k)));

    xdot(k) = xdot(k-1) + xddot_actual*dt;
    ydot(k) = ydot(k-1) + yddot_actual*dt;

    x(k) = x(k-1) + xdot(k)*dt;
    y(k) = y(k-1) + ydot(k)*dt;
end

x(end) = x(end-1);
y(end) = y(end-1);
xdot(end) = xdot(end-1);
ydot(end) = ydot(end-1);
phi_x(end) = phi_x(end-1);
phi_y(end) = phi_y(end-1);

idx_valid = ~isnan(res_norm_log);
t_valid = t(idx_valid);

figure('Color', 'w', ...
       'Name', 'Numerical Feedback Linearization Residual Norm', ...
       'Position', [150 150 1000 380]);

semilogy(t_valid, res_norm_log(idx_valid), 'LineWidth', 1.6);
grid on;

xlabel('Time [rad]');
ylabel('||F(z^*)||');
title('Residual Norm of Numerical Feedback Linearization');

xlim([0 4*pi]);
xticks(0:pi:4*pi);
xticklabels({'0', '\pi', '2\pi', '3\pi', '4\pi'});

fprintf('\n============================================================\n');
fprintf('NUMERICAL INVERSE MAPPING VALIDATION\n');
fprintf('============================================================\n');
fprintf('Maximum residual norm: %.4e\n', max(res_norm_log(idx_valid)));
fprintf('Mean residual norm: %.4e\n', mean(res_norm_log(idx_valid)));
fprintf('Median residual norm: %.4e\n', median(res_norm_log(idx_valid)));
fprintf('Minimum residual norm: %.4e\n', min(res_norm_log(idx_valid)));

function F = inverse_residual_discrete(z, x, y, phi_x_prev, phi_y_prev, dt, g, xdotdot_des, ydotdot_des)

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

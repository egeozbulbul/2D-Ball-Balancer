clear;
clc;
close all;

%% PID frequency-domain analysis
% Closed-loop tracking/error FRF and disturbance-to-error sensitivity FRF.
% Required toolboxes: Optimization Toolbox and System Identification Toolbox.


%% Physical constants
g = 9.81;

%% IK geometry parameters
a = 0.5;
L = 0.20;
base_offset = 0.1;
l2 = 0.30;
theta2_home_temp = asin(L/l2);
l1 = (a - base_offset) - l2*cos(theta2_home_temp);

%% Home configuration
[theta1_home, theta2_home, ~, ~] = plateAngleToIK(0, a, L, l1, l2, base_offset);

%% Scenario 1 PID gains
Kp_x = 10;
Kd_x = 4;
Ki_x = 0.02;

Kp_y = 5.5;
Kd_y = 1.95;
Ki_y = 0.02;

Imax = 0.5;

%% Frequency-domain identification initial conditions
x0 = 0;
y0 = 0;
x0dot = 0;
y0dot = 0;
phi_x0 = 0;
phi_y0 = 0;

%% Simulation settings
dt = 0.001;
Tsim = 120;
t = (0:dt:Tsim)';
N = length(t);

Ts_control = 0.03;
control_steps = round(Ts_control/dt);

%% Servo dynamics
tau_servo = 0.03;
theta_dot_limit = deg2rad(60);

%% Chirp settings
A_chirp = 0.01;     % small-signal amplitude [m]
f0 = 0.02;          % initial frequency [Hz]
f1 = 0.8;          % final frequency [Hz]

%% Analytical linear chirp and its derivatives

k_chirp = (f1 - f0)/Tsim;

phase = 2*pi*(f0*t + 0.5*k_chirp*t.^2);
phase_dot = 2*pi*(f0 + k_chirp*t);
phase_ddot = 2*pi*k_chirp;

chirp_signal = A_chirp * sin(phase);

chirp_dot = A_chirp * cos(phase) .* phase_dot;

chirp_ddot = A_chirp * ( ...
    -sin(phase) .* phase_dot.^2 ...
    + cos(phase) .* phase_ddot );

%% Identification settings
t_skip = 10;        % remove early transient from identification [s]
ds = 10;            % downsample factor for identification
Ts_id = dt * ds;

idx_id = find(t > t_skip);
idx_id = idx_id(1:ds:end);

xref_x_test = chirp_signal;
xdot_ref_x_test = chirp_dot;
xddot_ref_x_test = chirp_ddot;

yref_x_test = zeros(N,1);
ydot_ref_x_test = zeros(N,1);
yddot_ref_x_test = zeros(N,1);

fprintf("\nRunning PID closed-loop chirp test: X-axis reference...\n");

out_x = simulatePIDChirpTest( ...
    t, dt, control_steps, Ts_control, ...
    g, a, L, l1, l2, base_offset, ...
    theta1_home, theta2_home, ...
    tau_servo, theta_dot_limit, ...
    x0, y0, x0dot, y0dot, phi_x0, phi_y0, ...
    Kp_x, Kd_x, Ki_x, Kp_y, Kd_y, Ki_y, Imax, ...
    xref_x_test, yref_x_test, ...
    xdot_ref_x_test, ydot_ref_x_test, ...
    xddot_ref_x_test, yddot_ref_x_test);

%% Identification data for X-axis
data_x = iddata(out_x.x(idx_id), xref_x_test(idx_id), Ts_id);
data_x = detrend(data_x);

Gfrf_x = spafdr(data_x);
%% Error FRF for X-axis: E_x / X_ref
ex_test = xref_x_test - out_x.x;

data_ex = iddata(ex_test(idx_id), xref_x_test(idx_id), Ts_id);
data_ex = detrend(data_ex);

Gfrf_ex = spafdr(data_ex);

xref_y_test = zeros(N,1);
xdot_ref_y_test = zeros(N,1);
xddot_ref_y_test = zeros(N,1);

yref_y_test = chirp_signal;
ydot_ref_y_test = chirp_dot;
yddot_ref_y_test = chirp_ddot;

fprintf("Running PID closed-loop chirp test: Y-axis reference...\n");

out_y = simulatePIDChirpTest( ...
    t, dt, control_steps, Ts_control, ...
    g, a, L, l1, l2, base_offset, ...
    theta1_home, theta2_home, ...
    tau_servo, theta_dot_limit, ...
    x0, y0, x0dot, y0dot, phi_x0, phi_y0, ...
    Kp_x, Kd_x, Ki_x, Kp_y, Kd_y, Ki_y, Imax, ...
    xref_y_test, yref_y_test, ...
    xdot_ref_y_test, ydot_ref_y_test, ...
    xddot_ref_y_test, yddot_ref_y_test);

%% Identification data for Y-axis
data_y = iddata(out_y.y(idx_id), yref_y_test(idx_id), Ts_id);
data_y = detrend(data_y);

Gfrf_y = spafdr(data_y);
%% Error FRF for Y-axis: E_y / Y_ref
ey_test = yref_y_test - out_y.y;

data_ey = iddata(ey_test(idx_id), yref_y_test(idx_id), Ts_id);
data_ey = detrend(data_ey);

Gfrf_ey = spafdr(data_ey);

w_min = 2*pi*f0;
w_max = 2*pi*f1;

%% Closed-loop tracking FRF: output/reference
figure("Color", "w", "Name", "PID - Empirical Closed-Loop Tracking FRF");

bodeplot(Gfrf_x, Gfrf_y, {w_min, w_max});
grid on;

legend("T_{cl,x} = X/X_{ref}", ...
       "T_{cl,y} = Y/Y_{ref}", ...
       "Location", "best");

title("PID: Empirical Closed-Loop Tracking Frequency Response");

%% Tracking error FRF: error/reference
figure("Color", "w", "Name", "PID - Empirical Tracking Error FRF");

bodeplot(Gfrf_ex, Gfrf_ey, {w_min, w_max});
grid on;

legend("E_x/X_{ref}", ...
       "E_y/Y_{ref}", ...
       "Location", "best");

title("PID: Empirical Tracking Error Frequency Response");

[mag_x, ~, w_x] = bode(Gfrf_x);
[mag_y, ~, w_y] = bode(Gfrf_y);

mag_x = squeeze(mag_x);
mag_y = squeeze(mag_y);
w_x = squeeze(w_x);
w_y = squeeze(w_y);

valid_x = (w_x >= w_min) & (w_x <= w_max);
valid_y = (w_y >= w_min) & (w_y <= w_max);

w_x = w_x(valid_x);
mag_x = mag_x(valid_x);

w_y = w_y(valid_y);
mag_y = mag_y(valid_y);

dc_gain_x_approx = mag_x(1);
dc_gain_y_approx = mag_y(1);

bw_x_hz = estimateBandwidthFromFRF(w_x, mag_x);
bw_y_hz = estimateBandwidthFromFRF(w_y, mag_y);
[max_tracking_gain_x, idx_peak_tx] = max(mag_x);
[max_tracking_gain_y, idx_peak_ty] = max(mag_y);

peak_tracking_freq_x_hz = w_x(idx_peak_tx)/(2*pi);
peak_tracking_freq_y_hz = w_y(idx_peak_ty)/(2*pi);

[mag_ex, ~, w_ex] = bode(Gfrf_ex);
[mag_ey, ~, w_ey] = bode(Gfrf_ey);

mag_ex = squeeze(mag_ex);
mag_ey = squeeze(mag_ey);

w_ex = squeeze(w_ex);
w_ey = squeeze(w_ey);

valid_ex = (w_ex >= w_min) & (w_ex <= w_max);
valid_ey = (w_ey >= w_min) & (w_ey <= w_max);

w_ex_valid = w_ex(valid_ex);
mag_ex_valid = mag_ex(valid_ex);

w_ey_valid = w_ey(valid_ey);
mag_ey_valid = mag_ey(valid_ey);

low_freq_error_gain_x = mag_ex_valid(1);
low_freq_error_gain_y = mag_ey_valid(1);

[max_error_gain_x, idx_peak_ex] = max(mag_ex_valid);
[max_error_gain_y, idx_peak_ey] = max(mag_ey_valid);

peak_error_freq_x_hz = w_ex_valid(idx_peak_ex)/(2*pi);
peak_error_freq_y_hz = w_ey_valid(idx_peak_ey)/(2*pi);

fprintf("\n============================================================\n");
fprintf("PID SCENARIO 1 - EMPIRICAL TRACKING ERROR FRF\n");
fprintf("============================================================\n");

fprintf("\n--- X-axis error FRF: E_x / X_ref ---\n");
fprintf("Approx. low-frequency error gain: %.4f\n", low_freq_error_gain_x);
fprintf("Maximum error gain in tested range: %.4f\n", max_error_gain_x);
fprintf("Peak error frequency: %.4f Hz\n", peak_error_freq_x_hz);

fprintf("\n--- Y-axis error FRF: E_y / Y_ref ---\n");
fprintf("Approx. low-frequency error gain: %.4f\n", low_freq_error_gain_y);
fprintf("Maximum error gain in tested range: %.4f\n", max_error_gain_y);
fprintf("Peak error frequency: %.4f Hz\n", peak_error_freq_y_hz);

fprintf("\n============================================================\n");
fprintf("PID SCENARIO 1 - EMPIRICAL CLOSED-LOOP TRACKING FRF\n");
fprintf("============================================================\n");

fprintf("\n--- X-axis tracking FRF: T_cl_x ≈ X / X_ref ---\n");
fprintf("Approx. low-frequency gain: %.4f\n", dc_gain_x_approx);
fprintf("Approx. -3 dB bandwidth: %.4f Hz\n", bw_x_hz);
fprintf("X test max |x|: %.5f m\n", max(abs(out_x.x)));
fprintf("X test max |y|: %.5f m\n", max(abs(out_x.y)));
fprintf("X test saturation x1: %d steps (%.4f%%)\n", ...
    out_x.sat_count_x1, out_x.sat_percentage_x1);
fprintf("X test saturation x2: %d steps (%.4f%%)\n", ...
    out_x.sat_count_x2, out_x.sat_percentage_x2);
fprintf("X test saturation y1: %d steps (%.4f%%)\n", ...
    out_x.sat_count_y1, out_x.sat_percentage_y1);
fprintf("X test saturation y2: %d steps (%.4f%%)\n", ...
    out_x.sat_count_y2, out_x.sat_percentage_y2);
fprintf("Maximum tracking gain in tested range: %.4f\n", max_tracking_gain_x);
fprintf("Peak tracking-gain frequency: %.4f Hz\n", peak_tracking_freq_x_hz);

fprintf("\n--- Y-axis tracking FRF: T_cl_y ≈ Y / Y_ref ---\n");
fprintf("Approx. low-frequency gain: %.4f\n", dc_gain_y_approx);
fprintf("Approx. -3 dB bandwidth: %.4f Hz\n", bw_y_hz);
fprintf("Y test max |x|: %.5f m\n", max(abs(out_y.x)));
fprintf("Y test max |y|: %.5f m\n", max(abs(out_y.y)));
fprintf("Y test saturation x1: %d steps (%.4f%%)\n", ...
    out_y.sat_count_x1, out_y.sat_percentage_x1);
fprintf("Y test saturation x2: %d steps (%.4f%%)\n", ...
    out_y.sat_count_x2, out_y.sat_percentage_x2);
fprintf("Y test saturation y1: %d steps (%.4f%%)\n", ...
    out_y.sat_count_y1, out_y.sat_percentage_y1);
fprintf("Y test saturation y2: %d steps (%.4f%%)\n", ...
    out_y.sat_count_y2, out_y.sat_percentage_y2);
fprintf("Maximum tracking gain in tested range: %.4f\n", max_tracking_gain_y);
fprintf("Peak tracking-gain frequency: %.4f Hz\n", peak_tracking_freq_y_hz);

%% Disturbance-to-error sensitivity analysis
%% Physical constants
g = 9.81;

%% IK geometry parameters
a = 0.5;
L = 0.20;
base_offset = 0.1;
l2 = 0.30;
theta2_home_temp = asin(L/l2);
l1 = (a - base_offset) - l2*cos(theta2_home_temp);

%% Home configuration
[theta1_home, theta2_home, ~, ~] = plateAngleToIK(0, a, L, l1, l2, base_offset);

%% Scenario 1 PID gains
Kp_x = 10;
Kd_x = 4;
Ki_x = 0.02;

Kp_y = 5.5;
Kd_y = 1.95;
Ki_y = 0.02;

Imax = 0.5;

%% Frequency-domain identification initial conditions
x0 = 0;
y0 = 0;
x0dot = 0;
y0dot = 0;
phi_x0 = 0;
phi_y0 = 0;

%% Simulation settings
dt = 0.001;
Tsim = 120;
t = (0:dt:Tsim)';
N = length(t);

Ts_control = 0.03;
control_steps = round(Ts_control/dt);

%% Servo dynamics
tau_servo = 0.03;
theta_dot_limit = deg2rad(60);

%% Disturbance chirp settings
D_acc = 0.02;       % acceleration disturbance amplitude [m/s^2]
f0 = 0.02;          % initial frequency [Hz]
f1 = 0.80;          % final frequency [Hz]

%% Analytical linear chirp disturbance

k_chirp = (f1 - f0)/Tsim;

phase = 2*pi*(f0*t + 0.5*k_chirp*t.^2);

disturbance_signal = D_acc * sin(phase);

%% Identification settings
t_skip = 10;        % remove early transient from identification [s]
ds = 10;            % downsample factor
Ts_id = dt * ds;

idx_id = find(t > t_skip);
idx_id = idx_id(1:ds:end);

w_min = 2*pi*f0;
w_max = 2*pi*f1;

xref_dx_test = zeros(N,1);
yref_dx_test = zeros(N,1);

xdot_ref_dx_test = zeros(N,1);
ydot_ref_dx_test = zeros(N,1);

xddot_ref_dx_test = zeros(N,1);
yddot_ref_dx_test = zeros(N,1);

dist_x_test = disturbance_signal;
dist_y_test = zeros(N,1);

fprintf("\nRunning PID disturbance sensitivity test: X-axis acceleration disturbance...\n");

out_dx = simulatePIDDisturbanceTest( ...
    t, dt, control_steps, Ts_control, ...
    g, a, L, l1, l2, base_offset, ...
    theta1_home, theta2_home, ...
    tau_servo, theta_dot_limit, ...
    x0, y0, x0dot, y0dot, phi_x0, phi_y0, ...
    Kp_x, Kd_x, Ki_x, Kp_y, Kd_y, Ki_y, Imax, ...
    xref_dx_test, yref_dx_test, ...
    xdot_ref_dx_test, ydot_ref_dx_test, ...
    xddot_ref_dx_test, yddot_ref_dx_test, ...
    dist_x_test, dist_y_test);

ex_dist_test = xref_dx_test - out_dx.x;

data_sdx = iddata(ex_dist_test(idx_id), dist_x_test(idx_id), Ts_id);
data_sdx = detrend(data_sdx);

Gfrf_sdx = spafdr(data_sdx);

xref_dy_test = zeros(N,1);
yref_dy_test = zeros(N,1);

xdot_ref_dy_test = zeros(N,1);
ydot_ref_dy_test = zeros(N,1);

xddot_ref_dy_test = zeros(N,1);
yddot_ref_dy_test = zeros(N,1);

dist_x_test = zeros(N,1);
dist_y_test = disturbance_signal;

fprintf("Running PID disturbance sensitivity test: Y-axis acceleration disturbance...\n");

out_dy = simulatePIDDisturbanceTest( ...
    t, dt, control_steps, Ts_control, ...
    g, a, L, l1, l2, base_offset, ...
    theta1_home, theta2_home, ...
    tau_servo, theta_dot_limit, ...
    x0, y0, x0dot, y0dot, phi_x0, phi_y0, ...
    Kp_x, Kd_x, Ki_x, Kp_y, Kd_y, Ki_y, Imax, ...
    xref_dy_test, yref_dy_test, ...
    xdot_ref_dy_test, ydot_ref_dy_test, ...
    xddot_ref_dy_test, yddot_ref_dy_test, ...
    dist_x_test, dist_y_test);

ey_dist_test = yref_dy_test - out_dy.y;

data_sdy = iddata(ey_dist_test(idx_id), dist_y_test(idx_id), Ts_id);
data_sdy = detrend(data_sdy);

Gfrf_sdy = spafdr(data_sdy);

figure("Color", "w", "Name", "PID - Empirical Disturbance Sensitivity FRF");

bodeplot(Gfrf_sdx, Gfrf_sdy, {w_min, w_max});
grid on;

legend("S_{d,x} = E_x/D_{acc,x}", ...
       "S_{d,y} = E_y/D_{acc,y}", ...
       "Location", "best");

title("PID: Empirical Disturbance-to-Error Sensitivity Frequency Response");

[mag_sdx, ~, w_sdx] = bode(Gfrf_sdx);
[mag_sdy, ~, w_sdy] = bode(Gfrf_sdy);

mag_sdx = squeeze(mag_sdx);
mag_sdy = squeeze(mag_sdy);

w_sdx = squeeze(w_sdx);
w_sdy = squeeze(w_sdy);

valid_sdx = (w_sdx >= w_min) & (w_sdx <= w_max);
valid_sdy = (w_sdy >= w_min) & (w_sdy <= w_max);

w_sdx_valid = w_sdx(valid_sdx);
mag_sdx_valid = mag_sdx(valid_sdx);

w_sdy_valid = w_sdy(valid_sdy);
mag_sdy_valid = mag_sdy(valid_sdy);

low_freq_sens_x = mag_sdx_valid(1);
low_freq_sens_y = mag_sdy_valid(1);

[max_sens_x, idx_peak_sdx] = max(mag_sdx_valid);
[max_sens_y, idx_peak_sdy] = max(mag_sdy_valid);

peak_sens_freq_x_hz = w_sdx_valid(idx_peak_sdx)/(2*pi);
peak_sens_freq_y_hz = w_sdy_valid(idx_peak_sdy)/(2*pi);

fprintf("\n============================================================\n");
fprintf("PID SCENARIO 1 - EMPIRICAL DISTURBANCE SENSITIVITY FRF\n");
fprintf("============================================================\n");

fprintf("\n--- X-axis disturbance sensitivity: E_x / D_acc,x ---\n");
fprintf("Approx. low-frequency sensitivity: %.6f s^2\n", low_freq_sens_x);
fprintf("Maximum sensitivity in tested range: %.6f s^2\n", max_sens_x);
fprintf("Peak sensitivity frequency: %.4f Hz\n", peak_sens_freq_x_hz);
fprintf("X disturbance test max |x|: %.5f m\n", max(abs(out_dx.x)));
fprintf("X disturbance test max |y|: %.5f m\n", max(abs(out_dx.y)));
fprintf("X disturbance test saturation x1: %d steps (%.4f%%)\n", ...
    out_dx.sat_count_x1, out_dx.sat_percentage_x1);
fprintf("X disturbance test saturation x2: %d steps (%.4f%%)\n", ...
    out_dx.sat_count_x2, out_dx.sat_percentage_x2);
fprintf("X disturbance test saturation y1: %d steps (%.4f%%)\n", ...
    out_dx.sat_count_y1, out_dx.sat_percentage_y1);
fprintf("X disturbance test saturation y2: %d steps (%.4f%%)\n", ...
    out_dx.sat_count_y2, out_dx.sat_percentage_y2);

fprintf("\n--- Y-axis disturbance sensitivity: E_y / D_acc,y ---\n");
fprintf("Approx. low-frequency sensitivity: %.6f s^2\n", low_freq_sens_y);
fprintf("Maximum sensitivity in tested range: %.6f s^2\n", max_sens_y);
fprintf("Peak sensitivity frequency: %.4f Hz\n", peak_sens_freq_y_hz);
fprintf("Y disturbance test max |x|: %.5f m\n", max(abs(out_dy.x)));
fprintf("Y disturbance test max |y|: %.5f m\n", max(abs(out_dy.y)));
fprintf("Y disturbance test saturation x1: %d steps (%.4f%%)\n", ...
    out_dy.sat_count_x1, out_dy.sat_percentage_x1);
fprintf("Y disturbance test saturation x2: %d steps (%.4f%%)\n", ...
    out_dy.sat_count_x2, out_dy.sat_percentage_x2);
fprintf("Y disturbance test saturation y1: %d steps (%.4f%%)\n", ...
    out_dy.sat_count_y1, out_dy.sat_percentage_y1);
fprintf("Y disturbance test saturation y2: %d steps (%.4f%%)\n", ...
    out_dy.sat_count_y2, out_dy.sat_percentage_y2);

fprintf("\nNote:\n");
fprintf("These FRFs are empirical disturbance-to-error sensitivity estimates.\n");
fprintf("The input is acceleration disturbance [m/s^2], and the output is tracking error [m].\n");
fprintf("Therefore, the sensitivity unit is approximately s^2.\n");

%% Local functions

function out = simulatePIDDisturbanceTest( ...
    t, dt, control_steps, Ts_control, ...
    g, a, L, l1, l2, base_offset, ...
    theta1_home, theta2_home, ...
    tau_servo, theta_dot_limit, ...
    x0, y0, x0dot, y0dot, phi_x0, phi_y0, ...
    Kp_x, Kd_x, Ki_x, Kp_y, Kd_y, Ki_y, Imax, ...
    xref, yref, xdot_ref, ydot_ref, xddot_ref, yddot_ref, ...
    dist_x, dist_y)

    N = length(t);

    %% Preallocate states
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

    %% Initial conditions
    x(1) = x0;
    y(1) = y0;
    xdot(1) = x0dot;
    ydot(1) = y0dot;
    phi_x(1) = phi_x0;
    phi_y(1) = phi_y0;

    %% Initial commands
    phi_cmd_x_current = phi_x0;
    phi_cmd_y_current = phi_y0;

    theta1_cmd_x_current = 0;
    theta2_cmd_x_current = 0;
    theta1_cmd_y_current = 0;
    theta2_cmd_y_current = 0;

    Ix = 0;
    Iy = 0;

    %% Saturation counters
    sat_count_x1 = 0;
    sat_count_x2 = 0;
    sat_count_y1 = 0;
    sat_count_y2 = 0;

    options = optimoptions("fsolve", "Display", "off");

    %% Simulation loop
    for k = 2:N-1

        if mod(k-2, control_steps) == 0

            ex = x(k-1) - xref(k);
            ey = y(k-1) - yref(k);

            exdot = xdot(k-1) - xdot_ref(k);
            eydot = ydot(k-1) - ydot_ref(k);

            Ix = Ix + ex * Ts_control;
            Iy = Iy + ey * Ts_control;

            Ix = max(min(Ix, Imax), -Imax);
            Iy = max(min(Iy, Imax), -Imax);

            xdotdot_des = xddot_ref(k) - Kd_x*exdot - Kp_x*ex - Ki_x*Ix;
            ydotdot_des = yddot_ref(k) - Kd_y*eydot - Kp_y*ey - Ki_y*Iy;

            z0 = [phi_cmd_x_current; phi_cmd_y_current];

            z_sol = fsolve(@(z) inverse_residual_discrete( ...
                z, x(k-1), y(k-1), phi_x(k-1), phi_y(k-1), ...
                Ts_control, g, xdotdot_des, ydotdot_des), z0, options);

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

        %% Servo dynamics with rate limit
        desired_theta1_dot_x = (theta1_cmd_x_current - theta1_x(k-1)) / tau_servo;
        desired_theta2_dot_x = (theta2_cmd_x_current - theta2_x(k-1)) / tau_servo;
        desired_theta1_dot_y = (theta1_cmd_y_current - theta1_y(k-1)) / tau_servo;
        desired_theta2_dot_y = (theta2_cmd_y_current - theta2_y(k-1)) / tau_servo;

        theta1_dot_x = max(min(desired_theta1_dot_x, theta_dot_limit), -theta_dot_limit);
        theta2_dot_x = max(min(desired_theta2_dot_x, theta_dot_limit), -theta_dot_limit);
        theta1_dot_y = max(min(desired_theta1_dot_y, theta_dot_limit), -theta_dot_limit);
        theta2_dot_y = max(min(desired_theta2_dot_y, theta_dot_limit), -theta_dot_limit);

        if abs(desired_theta1_dot_x) >= theta_dot_limit
            sat_count_x1 = sat_count_x1 + 1;
        end
        if abs(desired_theta2_dot_x) >= theta_dot_limit
            sat_count_x2 = sat_count_x2 + 1;
        end
        if abs(desired_theta1_dot_y) >= theta_dot_limit
            sat_count_y1 = sat_count_y1 + 1;
        end
        if abs(desired_theta2_dot_y) >= theta_dot_limit
            sat_count_y2 = sat_count_y2 + 1;
        end

        theta1_x(k) = theta1_x(k-1) + theta1_dot_x * dt;
        theta2_x(k) = theta2_x(k-1) + theta2_dot_x * dt;

        theta1_y(k) = theta1_y(k-1) + theta1_dot_y * dt;
        theta2_y(k) = theta2_y(k-1) + theta2_dot_y * dt;

        %% Forward kinematics: joint angles to plate angles
        theta1_abs_x = theta1_x(k) + theta1_home;
        theta2_abs_x = theta2_x(k) + theta2_home;

        theta1_abs_y = theta1_y(k) + theta1_home;
        theta2_abs_y = theta2_y(k) + theta2_home;

        [phi_x(k), ~, ~] = thetaToPlateAngle(theta1_abs_x, theta2_abs_x, l1, l2, base_offset, L);
        [phi_y(k), ~, ~] = thetaToPlateAngle(theta1_abs_y, theta2_abs_y, l1, l2, base_offset, L);

        phi_dot_x(k) = (phi_x(k) - phi_x(k-1)) / dt;
        phi_dot_y(k) = (phi_y(k) - phi_y(k-1)) / dt;

        %% Nonlinear ball dynamics
        xddot = 5/7 * ( ...
              x(k-1)*phi_dot_x(k)^2 ...
            + y(k-1)*phi_dot_x(k)*phi_dot_y(k) ...
            - g*sin(phi_x(k)));

        yddot = 5/7 * ( ...
              y(k-1)*phi_dot_y(k)^2 ...
            + x(k-1)*phi_dot_x(k)*phi_dot_y(k) ...
            - g*sin(phi_y(k)));

        %% External acceleration disturbance injection
        xddot = xddot + dist_x(k);
        yddot = yddot + dist_y(k);

        xdot(k) = xdot(k-1) + xddot * dt;
        ydot(k) = ydot(k-1) + yddot * dt;

        x(k) = x(k-1) + xdot(k) * dt;
        y(k) = y(k-1) + ydot(k) * dt;
    end

    %% Fill last values
    x(end) = x(end-1);
    y(end) = y(end-1);
    xdot(end) = xdot(end-1);
    ydot(end) = ydot(end-1);
    phi_x(end) = phi_x(end-1);
    phi_y(end) = phi_y(end-1);

    %% Output structure
    out.x = x;
    out.y = y;
    out.xdot = xdot;
    out.ydot = ydot;
    out.phi_x = phi_x;
    out.phi_y = phi_y;

    out.sat_count_x1 = sat_count_x1;
    out.sat_count_x2 = sat_count_x2;
    out.sat_count_y1 = sat_count_y1;
    out.sat_count_y2 = sat_count_y2;

    out.sat_percentage_x1 = 100 * sat_count_x1 / N;
    out.sat_percentage_x2 = 100 * sat_count_x2 / N;
    out.sat_percentage_y1 = 100 * sat_count_y1 / N;
    out.sat_percentage_y2 = 100 * sat_count_y2 / N;
end

function out = simulatePIDChirpTest( ...
    t, dt, control_steps, Ts_control, ...
    g, a, L, l1, l2, base_offset, ...
    theta1_home, theta2_home, ...
    tau_servo, theta_dot_limit, ...
    x0, y0, x0dot, y0dot, phi_x0, phi_y0, ...
    Kp_x, Kd_x, Ki_x, Kp_y, Kd_y, Ki_y, Imax, ...
    xref, yref, xdot_ref, ydot_ref, xddot_ref, yddot_ref)

    N = length(t);

    %% Preallocate states
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
    %% Saturation counters for frequency-domain test
    sat_count_x1 = 0;
    sat_count_x2 = 0;
    sat_count_y1 = 0;
    sat_count_y2 = 0;
    %% Initial conditions
    x(1) = x0;
    y(1) = y0;
    xdot(1) = x0dot;
    ydot(1) = y0dot;
    phi_x(1) = phi_x0;
    phi_y(1) = phi_y0;

    %% Initial commands
    phi_cmd_x_current = phi_x0;
    phi_cmd_y_current = phi_y0;

    theta1_cmd_x_current = 0;
    theta2_cmd_x_current = 0;
    theta1_cmd_y_current = 0;
    theta2_cmd_y_current = 0;

    Ix = 0;
    Iy = 0;

    options = optimoptions("fsolve", "Display", "off");

    %% Simulation loop
    for k = 2:N-1

        if mod(k-2, control_steps) == 0

            ex = x(k-1) - xref(k);
            ey = y(k-1) - yref(k);

            exdot = xdot(k-1) - xdot_ref(k);
            eydot = ydot(k-1) - ydot_ref(k);

            Ix = Ix + ex * Ts_control;
            Iy = Iy + ey * Ts_control;

            Ix = max(min(Ix, Imax), -Imax);
            Iy = max(min(Iy, Imax), -Imax);

            xdotdot_des = xddot_ref(k) - Kd_x*exdot - Kp_x*ex - Ki_x*Ix;
            ydotdot_des = yddot_ref(k) - Kd_y*eydot - Kp_y*ey - Ki_y*Iy;

            z0 = [phi_cmd_x_current; phi_cmd_y_current];

            z_sol = fsolve(@(z) inverse_residual_discrete( ...
                z, x(k-1), y(k-1), phi_x(k-1), phi_y(k-1), ...
                Ts_control, g, xdotdot_des, ydotdot_des), z0, options);

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

        %% Servo dynamics with rate limit
        desired_theta1_dot_x = (theta1_cmd_x_current - theta1_x(k-1)) / tau_servo;
        desired_theta2_dot_x = (theta2_cmd_x_current - theta2_x(k-1)) / tau_servo;
        desired_theta1_dot_y = (theta1_cmd_y_current - theta1_y(k-1)) / tau_servo;
        desired_theta2_dot_y = (theta2_cmd_y_current - theta2_y(k-1)) / tau_servo;

        theta1_dot_x = max(min(desired_theta1_dot_x, theta_dot_limit), -theta_dot_limit);
        theta2_dot_x = max(min(desired_theta2_dot_x, theta_dot_limit), -theta_dot_limit);
        theta1_dot_y = max(min(desired_theta1_dot_y, theta_dot_limit), -theta_dot_limit);
        theta2_dot_y = max(min(desired_theta2_dot_y, theta_dot_limit), -theta_dot_limit);

        %% Saturation detection
        if abs(desired_theta1_dot_x) >= theta_dot_limit
            sat_count_x1 = sat_count_x1 + 1;
        end

        if abs(desired_theta2_dot_x) >= theta_dot_limit
            sat_count_x2 = sat_count_x2 + 1;
        end

        if abs(desired_theta1_dot_y) >= theta_dot_limit
            sat_count_y1 = sat_count_y1 + 1;
        end

        if abs(desired_theta2_dot_y) >= theta_dot_limit
            sat_count_y2 = sat_count_y2 + 1;
        end

        theta1_x(k) = theta1_x(k-1) + theta1_dot_x * dt;
        theta2_x(k) = theta2_x(k-1) + theta2_dot_x * dt;

        theta1_y(k) = theta1_y(k-1) + theta1_dot_y * dt;
        theta2_y(k) = theta2_y(k-1) + theta2_dot_y * dt;

        %% Forward kinematics: joint angles to plate angles
        theta1_abs_x = theta1_x(k) + theta1_home;
        theta2_abs_x = theta2_x(k) + theta2_home;

        theta1_abs_y = theta1_y(k) + theta1_home;
        theta2_abs_y = theta2_y(k) + theta2_home;

        [phi_x(k), ~, ~] = thetaToPlateAngle(theta1_abs_x, theta2_abs_x, l1, l2, base_offset, L);
        [phi_y(k), ~, ~] = thetaToPlateAngle(theta1_abs_y, theta2_abs_y, l1, l2, base_offset, L);

        phi_dot_x(k) = (phi_x(k) - phi_x(k-1)) / dt;
        phi_dot_y(k) = (phi_y(k) - phi_y(k-1)) / dt;

        %% Nonlinear ball dynamics
        xddot = 5/7 * ( ...
              x(k-1)*phi_dot_x(k)^2 ...
            + y(k-1)*phi_dot_x(k)*phi_dot_y(k) ...
            - g*sin(phi_x(k)));

        yddot = 5/7 * ( ...
              y(k-1)*phi_dot_y(k)^2 ...
            + x(k-1)*phi_dot_x(k)*phi_dot_y(k) ...
            - g*sin(phi_y(k)));

        xdot(k) = xdot(k-1) + xddot * dt;
        ydot(k) = ydot(k-1) + yddot * dt;

        x(k) = x(k-1) + xdot(k) * dt;
        y(k) = y(k-1) + ydot(k) * dt;
    end

    %% Fill last values
    x(end) = x(end-1);
    y(end) = y(end-1);
    xdot(end) = xdot(end-1);
    ydot(end) = ydot(end-1);
    phi_x(end) = phi_x(end-1);
    phi_y(end) = phi_y(end-1);

    %% Output structure
    out.x = x;
    out.y = y;
    out.xdot = xdot;
    out.ydot = ydot;
    out.phi_x = phi_x;
    out.phi_y = phi_y;
    %% Saturation outputs
    out.sat_count_x1 = sat_count_x1;
    out.sat_count_x2 = sat_count_x2;
    out.sat_count_y1 = sat_count_y1;
    out.sat_count_y2 = sat_count_y2;

    out.sat_percentage_x1 = 100 * sat_count_x1 / N;
    out.sat_percentage_x2 = 100 * sat_count_x2 / N;
    out.sat_percentage_y1 = 100 * sat_count_y1 / N;
    out.sat_percentage_y2 = 100 * sat_count_y2 / N;
end

function F = inverse_residual_discrete(z, x, y, phi_x_prev, phi_y_prev, dt, g, xdotdot_des, ydotdot_des)

    phi_cmd_x = z(1);
    phi_cmd_y = z(2);

    phi_dot_x = (phi_cmd_x - phi_x_prev) / dt;
    phi_dot_y = (phi_cmd_y - phi_y_prev) / dt;

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

    D = (x_rel^2 + y_rel^2 - l1^2 - l2^2) / (2*l1*l2);
    D = max(min(D, 1), -1);

    theta2 = acos(D);

    theta1 = atan2(y_rel, x_rel) - atan2(l2*sin(theta2), l1 + l2*cos(theta2));
end

function [phi, x2, y2] = thetaToPlateAngle(theta1, theta2, l1, l2, base_offset, L)

    x2 = base_offset + l1*cos(theta1) + l2*cos(theta1 + theta2);
    y2 = l1*sin(theta1) + l2*sin(theta1 + theta2);

    phi = atan2(y2 - L, x2);
end

function bw_hz = estimateBandwidthFromFRF(w_rad, mag)

    if isempty(w_rad) || isempty(mag) || all(isnan(mag))
        bw_hz = NaN;
        return;
    end

    mag = abs(mag);
    low_gain = mag(1);
    threshold = low_gain / sqrt(2);

    idx = find(mag <= threshold, 1, "first");

    if isempty(idx)
        bw_hz = NaN;
    else
        bw_hz = w_rad(idx) / (2*pi);
    end
end

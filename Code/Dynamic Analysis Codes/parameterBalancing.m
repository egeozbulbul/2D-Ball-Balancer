% 2D Ball Balancer Optimization (All in SI: meter, kg, m/s^2, N)
clc; clear;

% Fixed parameters (SI units!)
m_ball_ = 0.024;        % kg (24 grams)
L_ = 0.100;             % m  (100 mm)
r_ball_ = 0.009;        % m  (9 mm)
g_ = 9.81;              % m/s^2

% Controlled parameters: [m_platform, m_rack, OS, r_p] (all SI!)
% Lower bounds (kg, kg, m, m)
lb = [0.09, 0.05, 0.01, 0.03];       
ub = [0.31, 0.10, 0.04, 0.10];    

x0 = [0.15, 0.10, 0.025, 0.05];    % Initial guess (kg, kg, m, m)

% Only the variance of newly derived parameters will be minimized
objective = @(x) var(computeNewParams(x, m_ball_, L_, r_ball_, g_));

options = optimoptions('fmincon','Display','none','Algorithm','sqp');
[x_opt, ~] = fmincon(objective, x0, [], [], [], [], lb, ub, [], options);

% Print the results properly
controlled_names = {'m_platform', 'm_rack', 'OS', 'r_p'};
fprintf('\nBest controlled parameters:\n');
fprintf('m_platform = %.2f grams\n', x_opt(1)*1000);
fprintf('m_rack     = %.2f grams\n', x_opt(2)*1000);
fprintf('OS         = %.2f mm\n', x_opt(3)*1000);
fprintf('r_p        = %.2f mm\n', x_opt(4)*1000);

% Calculate derived parameters
params = computeParams(x_opt, m_ball_, L_, r_ball_, g_);
new_params = computeNewParams(x_opt, m_ball_, L_, r_ball_, g_);
derived_old = {'A', 'B', 'C', 'D', 'E', 'F', 'H', 'I', 'L1'};
derived_new = {'A_1', 'B_1', 'C_1', 'D_1', 'E_1', 'F_1', 'E_2', 'H_1', 'C_2', 'F_2', 'I_1', 'L_1_'};

fprintf('\nClassical derived parameters:\n');
for i = 1:9
    fprintf('%s = %.6f\n', derived_old{i}, params(i));
end

fprintf('\nNewly derived parameters:\n');
for i = 1:12
    fprintf('%s = %.6f\n', derived_new{i}, new_params(i));
end

% Helper functions:
function params = computeParams(x, m_ball_, L_, r_ball_, g_)
    m_platform = x(1);
    m_rack     = x(2);
    OS         = x(3);
    r_p        = x(4);

    A  = 7/5 * m_ball_;
    B  = g_ * m_rack * r_p;
    C  = -g_ * m_ball_;
    D  = ((L_ - OS)^2) / (r_p^2);
    E  = g_*m_ball_*L_ + (L_*g_*m_platform)/2;
    F  = -g_*m_ball_*r_p*r_ball_/(L_-OS);
    H  = m_platform*(L_-OS)^2 * ( ((L_/2 - OS)^2) + L_^2/12 ) / (r_p^2);
    I  = g_*m_ball_*r_ball_/(L_-OS);
    L1 = m_ball_ * g_ * (L_-OS) / r_p;

    params = [A, B, C, D, E, F, H, I, L1];
end

function new_params = computeNewParams(x, m_ball_, L, r_ball, g_)
    p = computeParams(x, m_ball_, L, r_ball, g_);
    A = p(1); B = p(2); C = p(3); D = p(4); E = p(5); F = p(6); H = p(7); I = p(8); L1 = p(9);

    A_1   = A;
    B_1   = B / A;
    C_1   = C / sqrt(D);
    D_1   = D;
    E_1   = E / sqrt(D);
    F_1   = F / sqrt(D);
    E_2   = E / (D^(3/2));
    H_1   = H / (2*D^3);
    C_2   = C / (D^(3/2));
    F_2   = F / (D^(3/2));
    I_1   = I / sqrt(D);
    L_1_  = L1 / sqrt(D);

    new_params = [A_1, B_1, C_1, D_1, E_1, F_1, E_2, H_1, C_2, F_2, I_1, L_1_];
end

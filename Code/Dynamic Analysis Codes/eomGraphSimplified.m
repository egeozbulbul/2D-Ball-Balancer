clear;
clc;

% Basic parameters
m_ball_     = 0.024; %kg
m_platform_ = 0.227; %kg
m_rack_     = 0.05;  %kg
L_          = 0.1;   %m
OS_         = 0.04;  %m
r_p_        = 0.064; %m
r_ball_     = 0.009; %m
g_          = 9.81;  %m/s^2
b_          = 0.1;   %400N/m
mu_         = 0.4;
mu_r_       = 0.001;
p_air_      = 1.19;  %kg/m^3
C_d_        = 0.55;

A_ball_ = pi * r_ball_^2;


A_ = 7/5 * m_ball_;
B_ = g_ * m_rack_ * r_p_;
C_ = -g_ * m_ball_;
D_ = (L_ - OS_)^2 / r_p_^2;
E_ = g_ * m_ball_ * L_ + L_ * g_ * m_platform_ / 2;
F_ = -g_ * m_ball_ * r_ball_ * r_p_ / (L_ - OS_);
H_ = m_platform_ * ((L_/2 - OS_)^2 + L_^2/12) / r_p_^2;
I_ = g_ * m_ball_ * r_ball_ / (L_ - OS_);
K_ = g_ * m_ball_ * (L_ - OS_);
L_1_ = m_ball_ * g_ * (L_- OS_/r_p_);
d_ = 0.5 * p_air_ * C_d_ * A_ball_;

x_ = 0.058;
xdot_ = -0.1;
xdotdot_ = -2;
phidot_ = -0.4;

Gain_x = 1;
Gain_xdot = 1;
Gain_xdotdot = 1;

Den     = D_;
sqrtDen = sqrt(Den);

f = @(p) ...
      Gain_xdotdot * A_ * xdotdot_ ...
    + B_ * p ...
    + C_ * Gain_x * x_ * phidot_ / sqrtDen ...
    + E_ * phidot_ / sqrtDen ...
    + F_ * p * phidot_ / sqrtDen ...
    + C_ * Gain_x * x_ * p / sqrtDen ...
    + F_ * p^2 / sqrtDen ...
    + b_ *  Gain_xdot * xdot_ ...
    + mu_   * L_1_ * sign(xdot_) / sqrtDen;

figure(1)
phi_plot_deg = linspace(-15,15,500) * (180/pi);
y_f = arrayfun(@(phi_deg) f(phi_deg * pi/180), phi_plot_deg);
plot(phi_plot_deg, y_f), grid on, xlabel('\phi (degree)'), ylabel('f(\phi)')

phi_min_deg = -200;
phi_max_deg = 200;

phi_min_rad = phi_min_deg * pi/180;
phi_max_rad = phi_max_deg * pi/180;

phi_root = fzero(f, [phi_min_rad phi_max_rad]);
phi_root_deg = phi_root * (180/pi);

% --- Normalize with 360 (convert to smallest equivalent angle)
phi_root_deg_norm = phi_root_deg;
phi_root_deg_norm = mod(phi_root_deg + 180, 360) - 180;

% Second step: Check if it is still within ±90
if phi_root_deg_norm >= -90 && phi_root_deg_norm <= 90
    disp(['Root phi: ', num2str(phi_root_deg_norm), ' degree (within servo range)'])
else
    disp(['Root phi: ', num2str(phi_root_deg_norm), ' degree (UNREACHABLE - outside servo limit!)'])
end

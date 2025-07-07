clear; clc;

% 1) x variable (0-100 range): this is a fixed interval!
x_vals        = 0:0.01:0.1;           % From 0 to 0.1 with step 0.01 (11 points)
xdot_vals     = -0.25:0.01:0.25;     
xdotdot_vals  = -8:0.1:8;   
phidot_vals   = -0.65:0.1:0.65;           

% Result matrix: [x, xdot, xdotdot, phidot]
result = zeros(length(x_vals), length(xdot_vals), length(xdotdot_vals), length(phidot_vals));

% --- Before the loop:
m_ball_     = 0.024; %kg
m_platform_ = 0.227; %kg
m_rack_     = 0.05;  %kg
L_          = 0.1;   %m
OS_         = 0.04;  %m
r_p_        = 0.064; %m
r_ball_     = 0.009; %m
g_          = 9.81;  %m/s^2
b_          = 0.1;   
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

for i = 1:length(x_vals)
    for j = 1:length(xdot_vals)
        for k = 1:length(xdotdot_vals)
            for l = 1:length(phidot_vals)
                x_ = x_vals(i);
                xdot_ = xdot_vals(j);
                xdotdot_PID_ = xdotdot_vals(k);
                phidot_ = phidot_vals(l);

                Den     = D_;
                sqrtDen = sqrt(D_);

                f = @(p) ...
                      A_ * xdotdot_PID_ ...
                    + B_ * p ...
                    + C_ * x_ * phidot_ ./ sqrtDen ...
                    + E_ * phidot_ ./ sqrtDen ...
                    + C_ * x_ * p ./ sqrtDen ...
                    + F_ * p.^2 ./ sqrtDen ...
                    + b_ * xdot_ ...
                    + mu_   * L_1_ * sign(xdot_) ./ sqrtDen ;

                % Grid scanning parameters
                N_scan = 400;                      % Number of steps (increase if needed!)
                phi_scan = linspace(-5*pi, 5*pi, N_scan);
                f_vals = arrayfun(f, phi_scan);
                root_found = false;

                for m = 1:N_scan-1
                    if f_vals(m)*f_vals(m+1) < 0     % If there is a sign change
                        try
                            phi_root = fzero(f, [phi_scan(m) phi_scan(m+1)]);
                            phi_deg = phi_root * 180/pi;
                            phi_deg_norm = mod(phi_deg + 180, 360) - 180;

                            % Classify for the first found root
                            if abs(phi_deg_norm) <= 180
                                result(i,j,k,l) = 1;
                            else
                                result(i,j,k,l) = -1;
                            end
                            root_found = true;
                            break    % Exit after first root is found
                        catch
                            % If fzero fails, continue
                        end
                    end
                end

                if ~root_found
                    result(i,j,k,l) = 0;    % If no sign change, no root!
                end

            end
        end
    end
end

% Guaranteed Safe Set Calculation (intersection of safe points for all x)
safe_matrix = ones(length(xdot_vals), length(xdotdot_vals), length(phidot_vals)); % all safe at the beginning

for i = 1:length(x_vals)
    cur_safe = squeeze(result(i,:,:,:) == 1);  
    safe_matrix = safe_matrix & cur_safe;
end

[XDOT, XDDOT, PHIDOT] = ndgrid(xdot_vals, xdotdot_vals, phidot_vals);

idx_safe = find(safe_matrix);
idx_unsafe = find(~safe_matrix);

figure;
hold on
scatter3(XDOT(idx_safe), XDDOT(idx_safe), PHIDOT(idx_safe), 70, [1 1 1], 'filled');   % White
scatter3(XDOT(idx_unsafe), XDDOT(idx_unsafe), PHIDOT(idx_unsafe), 70, [0 0 0], 'filled'); % Black

% --- CONTOUR/ISOSURFACE BLOCK ---
safe_matrix_double = double(safe_matrix);
fv = isosurface(XDOT, XDDOT, PHIDOT, safe_matrix_double, 0.5);

if ~isempty(fv.vertices)
    p = patch(fv);
    p.FaceColor = [0.5 0 0.13]; % Reddish
    p.EdgeColor = 'none';
    p.FaceAlpha = 0.90;
end
% --------------------------------

hold off
xlabel('xdot'); ylabel('xdotdot\_PID'); zlabel('phidot');
title('Guaranteed Safe Set: White = Root Exists, Black = No Root, Red = Boundary/Contour');
grid on
view(3)
camlight; lighting gouraud

disp(['Number of guaranteed safe combinations: ', num2str(length(idx_safe))]);

[idx_xdot, idx_xdotdot, idx_phidot] = ind2sub(size(safe_matrix), idx_unsafe);

empty_points = [xdot_vals(idx_xdot)', xdotdot_vals(idx_xdotdot)', phidot_vals(idx_phidot)'];

disp('Points outside the guaranteed safe set (not equal to 1):');
disp('  xdot      xdotdot_PID  phidot');
disp(empty_points);

if isempty(empty_points)
    disp('All points are guaranteed safe!');
end

save('safe_boundary_data_simplified.mat', 'safe_matrix', 'xdot_vals', 'xdotdot_vals', 'phidot_vals');


%%

% 1,0,-1 scatter for x_desired

x_desired = 0.1;
[~, fixed_i] = min(abs(x_vals - x_desired));

[XDOT, XDDOT, PHIDOT] = ndgrid(xdot_vals, xdotdot_vals, phidot_vals);

R = squeeze(result(fixed_i,:,:,:));
R = R(:);
XDOT = XDOT(:);
XDDOT = XDDOT(:);
PHIDOT = PHIDOT(:);

figure;
scatter3(XDOT, XDDOT, PHIDOT, 50, R, 'filled');
xlabel('xdot'), ylabel('xdotdot\_PID'), zlabel('phidot')
title(['Root suitability: x = ', num2str(x_vals(fixed_i)), ' mm'])
colorbar
grid on

clc;
load("Datas/x_vs_time_datas/fullEom.mat")

%% Drag Term %0.03
fprintf('\n--- Drag Term Disabled ---\n');
load("Datas/x_vs_time_datas/dragTerm.mat")
time = fullEom(1,:);    
x_full = fullEom(2,:);  
x_drag = dragTerm(2,:); 
rel_error = 100 * abs(x_full - x_drag) ./ abs(x_full);
figure;
plot(time, rel_error, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Relative Error (%)');
title('Relative Percentage Error: Full EOM vs Drag Term Disabled');
grid on;
mean_rel_error = mean(rel_error);
std_rel_error = std(rel_error);
fprintf('Mean Relative Error: %.4f%%\n', mean_rel_error);
fprintf('Standard Deviation: %.4f\n', std_rel_error);

%% Rolling Resistance Term %0.71
fprintf('\n--- Rolling Resistance Term Disabled ---\n');
load("Datas/x_vs_time_datas/rollingResistanceTerm.mat")
time = fullEom(1,:);  
x_full = fullEom(2,:);  
x_rolling = rollingResistanceTerm(2,:); 
rel_error = 100 * abs(x_full - x_rolling) ./ abs(x_full);
figure;
plot(time, rel_error, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Relative Error (%)');
title('Relative Percentage Error: Full EOM vs Rolling Resistance Term Disabled');
grid on;
mean_rel_error = mean(rel_error);
std_rel_error = std(rel_error);
fprintf('Mean Relative Error: %.4f%%\n', mean_rel_error);
fprintf('Standard Deviation: %.4f\n', std_rel_error);

%% Coulomb Friction Term 
fprintf('\n--- Coulomb Friction Term Disabled ---\n');
load("Datas/x_vs_time_datas/coloumbTerm.mat")
N = size(coloumbTerm, 2);
time = fullEom(1,1:N);         
x_full = fullEom(2,1:N);       
x_coloumb = coloumbTerm(2,:);  
rel_error = 100 * abs(x_full - x_coloumb) ./ abs(x_full);
figure;
plot(time, rel_error, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Relative Error (%)');
title('Relative Percentage Error: Full EOM vs Coulomb Term Disabled');
grid on;
mean_rel_error = mean(rel_error);
std_rel_error = std(rel_error);
fprintf('Mean Relative Error: %.4f%%\n', mean_rel_error);
fprintf('Standard Deviation: %.4f\n', std_rel_error);

%% Viscosity Term
fprintf('\n--- Viscosity Term Disabled ---\n');
load("Datas/x_vs_time_datas/viscosityTerm.mat")
N = size(viscosityTerm, 2);
time = fullEom(1,1:N);         
x_full = fullEom(2,1:N);       
x_viscosity = viscosityTerm(2,:);
rel_error = 100 * abs(x_full - x_viscosity) ./ abs(x_full);
figure;
plot(time, rel_error, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Relative Error (%)');
title('Relative Percentage Error: Full EOM vs Viscosity Term Disabled');
grid on;
mean_rel_error = mean(rel_error);
std_rel_error = std(rel_error);
fprintf('Mean Relative Error: %.4f%%\n', mean_rel_error);
fprintf('Standard Deviation: %.4f\n', std_rel_error);

%% -F*phi^3*phidot/32 Term %0.0003
fprintf('\n--- -F*phi^3*phidot/32 Term Disabled ---\n');
load("Datas/x_vs_time_datas/F_phi_3_phidot_32.mat")
N = size(F_phi_3_phidot_32, 2);
time = fullEom(1,1:N);
x_full = fullEom(2,1:N);
x_F1 = F_phi_3_phidot_32(2,:);
rel_error = 100 * abs(x_full - x_F1) ./ abs(x_full);
figure;
plot(time, rel_error, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Relative Error (%)');
title('Relative Percentage Error: Full EOM vs F1 Term Disabled');
grid on;
mean_rel_error = mean(rel_error);
std_rel_error = std(rel_error);
fprintf('Mean Relative Error: %.4f%%\n', mean_rel_error);
fprintf('Standard Deviation: %.4f\n', std_rel_error);

%% -E*phi^2*phidot/32 Term %0.22
fprintf('\n--- -E*phi^2*phidot/32 Term Disabled ---\n');
load("Datas/x_vs_time_datas/E_phi_2_phidot_32.mat")
N = size(E_phi_2_phidot_32, 2);
time = fullEom(1,1:N);
x_full = fullEom(2,1:N);
x_E1 = E_phi_2_phidot_32(2,:);
rel_error = 100 * abs(x_full - x_E1) ./ abs(x_full);
figure;
plot(time, rel_error, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Relative Error (%)');
title('Relative Percentage Error: Full EOM vs E1 Term Disabled');
grid on;
mean_rel_error = mean(rel_error);
std_rel_error = std(rel_error);
fprintf('Mean Relative Error: %.4f%%\n', mean_rel_error);
fprintf('Standard Deviation: %.4f\n', std_rel_error);

%% -C*x*phi^2*phidot/32 Term %0.018
fprintf('\n--- -C*x*phi^2*phidot/32 Term Disabled ---\n');
load("Datas/x_vs_time_datas/C_x_phi_2_phidot_32.mat")
N = size(C_x_phi_2_phidot_32, 2);
time = fullEom(1,1:N);
x_full = fullEom(2,1:N);
x_C1 = C_x_phi_2_phidot_32(2,:);
rel_error = 100 * abs(x_full - x_C1) ./ abs(x_full);
figure;
plot(time, rel_error, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Relative Error (%)');
title('Relative Percentage Error: Full EOM vs C1 Term Disabled');
grid on;
mean_rel_error = mean(rel_error);
std_rel_error = std(rel_error);
fprintf('Mean Relative Error: %.4f%%\n', mean_rel_error);
fprintf('Standard Deviation: %.4f\n', std_rel_error);

%% C*x*phi/sqrt Term 
fprintf('\n--- C*x*phi/sqrt Term Disabled ---\n');
load("Datas/x_vs_time_datas/C_x_phi_sqrt.mat")
N = size(C_x_phi_sqrt, 2);
time = fullEom(1,1:N);
x_full = fullEom(2,1:N);
x_C2 = C_x_phi_sqrt(2,:);
rel_error = 100 * abs(x_full - x_C2) ./ abs(x_full);
figure;
plot(time, rel_error, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Relative Error (%)');
title('Relative Percentage Error: Full EOM vs C2 Term Disabled');
grid on;
mean_rel_error = mean(rel_error);
std_rel_error = std(rel_error);
fprintf('Mean Relative Error: %.4f%%\n', mean_rel_error);
fprintf('Standard Deviation: %.4f\n', std_rel_error);

%% F*phi^2/sqrt Term
fprintf('\n--- F*phi^2/sqrt Term Disabled ---\n');
load("Datas/x_vs_time_datas/F_phi_2_sqrt.mat")
N = size(F_phi_2_sqrt, 2);
time = fullEom(1,1:N);
x_full = fullEom(2,1:N);
x_F2 = F_phi_2_sqrt(2,:);
rel_error = 100 * abs(x_full - x_F2) ./ abs(x_full);
figure;
plot(time, rel_error, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Relative Error (%)');
title('Relative Percentage Error: Full EOM vs F2 Term Disabled');
grid on;
mean_rel_error = mean(rel_error);
std_rel_error = std(rel_error);
fprintf('Mean Relative Error: %.4f%%\n', mean_rel_error);
fprintf('Standard Deviation: %.4f\n', std_rel_error);

%% B*phi Term
fprintf('\n--- B*phi Term Disabled ---\n');
load("Datas/x_vs_time_datas/B_phi.mat")
N = size(B_phi, 2);
time = fullEom(1,1:N);
x_full = fullEom(2,1:N);
x_B = B_phi(2,:);
rel_error = 100 * abs(x_full - x_B) ./ abs(x_full);
figure;
plot(time, rel_error, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Relative Error (%)');
title('Relative Percentage Error: Full EOM vs B Term Disabled');
grid on;
mean_rel_error = mean(rel_error);
std_rel_error = std(rel_error);
fprintf('Mean Relative Error: %.4f%%\n', mean_rel_error);
fprintf('Standard Deviation: %.4f\n', std_rel_error);

%% C*x*phidot/sqrt Term
fprintf('\n--- C*x*phidot/sqrt Term Disabled ---\n');
load("Datas/x_vs_time_datas/C_x_phidot_sqrt.mat")
N = size(C_x_phidot_sqrt, 2);
time = fullEom(1,1:N);
x_full = fullEom(2,1:N);
x_C3 = C_x_phidot_sqrt(2,:);
rel_error = 100 * abs(x_full - x_C3) ./ abs(x_full);
figure;
plot(time, rel_error, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Relative Error (%)');
title('Relative Percentage Error: Full EOM vs C3 Term Disabled');
grid on;
mean_rel_error = mean(rel_error);
std_rel_error = std(rel_error);
fprintf('Mean Relative Error: %.4f%%\n', mean_rel_error);
fprintf('Standard Deviation: %.4f\n', std_rel_error);

%% I*phi^2*phidot/sqrt %0.05
fprintf('\n--- I*phi^2*phidot/sqrt Term Disabled ---\n');
load("Datas/x_vs_time_datas/I_phi_2_phidot_sqrt.mat")
N = size(I_phi_2_phidot_sqrt, 2);
time = fullEom(1,1:N);
x_full = fullEom(2,1:N);
x_I = I_phi_2_phidot_sqrt(2,:);
rel_error = 100 * abs(x_full - x_I) ./ abs(x_full);
figure;
plot(time, rel_error, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Relative Error (%)');
title('Relative Percentage Error: Full EOM vs I Term Disabled');
grid on;
mean_rel_error = mean(rel_error);
std_rel_error = std(rel_error);
fprintf('Mean Relative Error: %.4f%%\n', mean_rel_error);
fprintf('Standard Deviation: %.4f\n', std_rel_error);

%% E*phidot/sqrt Term
fprintf('\n--- E*phidot/sqrt Term Disabled ---\n');
load("Datas/x_vs_time_datas/E_phidot_sqrt.mat")
N = size(E_phidot_sqrt, 2);
time = fullEom(1,1:N);
x_full = fullEom(2,1:N);
x_E2 = E_phidot_sqrt(2,:);
rel_error = 100 * abs(x_full - x_E2) ./ abs(x_full);
figure;
plot(time, rel_error, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Relative Error (%)');
title('Relative Percentage Error: Full EOM vs E2 Term Disabled');
grid on;
mean_rel_error = mean(rel_error);
std_rel_error = std(rel_error);
fprintf('Mean Relative Error: %.4f%%\n', mean_rel_error);
fprintf('Standard Deviation: %.4f\n', std_rel_error);

%% F*phi*phidot/sqrt Term %0.037
fprintf('\n--- F*phi*phidot/sqrt Term Disabled ---\n');
load("Datas/x_vs_time_datas/F_phi_phidot_sqrt.mat")
N = size(F_phi_phidot_sqrt, 2);
time = fullEom(1,1:N);
x_full = fullEom(2,1:N);
x_F3 = F_phi_phidot_sqrt(2,:);
rel_error = 100 * abs(x_full - x_F3) ./ abs(x_full);
figure;
plot(time, rel_error, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Relative Error (%)');
title('Relative Percentage Error: Full EOM vs F3 Term Disabled');
grid on;
mean_rel_error = mean(rel_error);
std_rel_error = std(rel_error);
fprintf('Mean Relative Error: %.4f%%\n', mean_rel_error);
fprintf('Standard Deviation: %.4f\n', std_rel_error);

%% H/2*phi^3*phidot/3 Term %0.0046
fprintf('\n--- H/2*phi^3*phidot/3.mat Term Disabled ---\n');
load("Datas/x_vs_time_datas/H_2_phi_3_phidot_3.mat")
N = size(H_2_phi_3_phidot_3, 2);
time = fullEom(1,1:N);
x_full = fullEom(2,1:N);
x_H1 = H_2_phi_3_phidot_3(2,:);
rel_error = 100 * abs(x_full - x_H1) ./ abs(x_full);
figure;
plot(time, rel_error, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Relative Error (%)');
title('Relative Percentage Error: Full EOM vs H1 Term Disabled');
grid on;
mean_rel_error = mean(rel_error);
std_rel_error = std(rel_error);
fprintf('Mean Relative Error: %.4f%%\n', mean_rel_error);
fprintf('Standard Deviation: %.4f\n', std_rel_error);

%% All Terms With A Relative Percentage Error Smaller Than 1%
fprintf('\n--- All Terms With A Relative Error Smaller Than 1 Percent Disabled ---\n');
load("Datas/x_vs_time_datas/all_terms_smaller_than_1.mat")
N = size(all_terms_smaller_than_1, 2);
time = fullEom(1,1:N);
x_full = fullEom(2,1:N);
x_Simplified1 = all_terms_smaller_than_1(2,:);
rel_error = 100 * abs(x_full - x_Simplified1) ./ abs(x_full);
figure;
plot(time, rel_error, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Relative Error (%)');
title('Relative Percentage Error: Full EOM vs Simplified1 Term Disabled');
grid on;
mean_rel_error = mean(rel_error);
std_rel_error = std(rel_error);
fprintf('Mean Relative Error: %.4f%%\n', mean_rel_error);
fprintf('Standard Deviation: %.4f\n', std_rel_error);

%% All Terms With A Relative Percentage Error Smaller Than 1% + Denominator Simplification
fprintf('\n--- All Terms With A Relative Error Smaller Than 1 Percent Disabled and Denominator Simplification ---\n');
load("Datas/x_vs_time_datas/denominator_simplification.mat")
N = size(denominator_simplification, 2);
time = fullEom(1,1:N);
x_full = fullEom(2,1:N);
x_Simplified2 = denominator_simplification(2,:);
rel_error = 100 * abs(x_full - x_Simplified2) ./ abs(x_full);
figure;
plot(time, rel_error, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Relative Error (%)');
title('Relative Percentage Error: Full EOM vs Simplified2 Term Disabled');
grid on;
mean_rel_error = mean(rel_error);
std_rel_error = std(rel_error);
fprintf('Mean Relative Error: %.4f%%\n', mean_rel_error);
fprintf('Standard Deviation: %.4f\n', std_rel_error);

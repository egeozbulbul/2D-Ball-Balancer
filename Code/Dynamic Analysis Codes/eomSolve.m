% Simülasyon süresi ve başlangıç koşulları
tspan = [0 10];
y0 = [0; 0; 0.1; 0];   % x, x_dot, phi, phi_dot

% Çözüm
[t, y] = ode45(@eomFunction, tspan, y0);

figure;
plot(t, y(:,3), 'b', 'LineWidth', 2)
hold on
plot(t, y(:,1), 'r', 'LineWidth', 2)
xlabel('Time (s)')
legend('\phi_x(t)', 'x(t)')
title('\phi_x ve x zamanla değişimi')
grid on

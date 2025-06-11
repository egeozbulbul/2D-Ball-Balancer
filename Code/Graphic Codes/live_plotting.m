
port = "/dev/tty.usbmodem142101"; 
baud = 9600;
s = serialport(port, baud);
flush(s);

x = [];
y = [];
t = [];


figure(1); clf; % x step
hx1 = plot(nan, nan, 'bo', 'MarkerFaceColor', 'b'); hold on;
hx2 = plot(nan, nan, 'b-', 'LineWidth',2);
yline(50, '--k', 'Target (50 mm)');
xlabel('Time (s)'); ylabel('x position (mm)');
title('Step Response (x axis) - Live Cubic Spline');
grid on;

figure(2); clf; % y step
hy1 = plot(nan, nan, 'ro', 'MarkerFaceColor', 'r'); hold on;
hy2 = plot(nan, nan, 'r-', 'LineWidth',2);
yline(50, '--k', 'Target (50 mm)');
xlabel('Time (s)'); ylabel('y position (mm)');
title('Step Response (y axis) - Live Cubic Spline');
grid on;

figure(3); clf; % Trajectory
ht1 = plot(nan, nan, 'k.', 'DisplayName', 'Discrete data'); hold on;
ht2 = plot(nan, nan, '-', 'Color', [0.4 0.6 1], 'LineWidth', 2, 'DisplayName', 'Cubic Spline');
hstart = plot(nan, nan, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
htarget = plot(50, 50, 'p', 'MarkerSize', 18, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'DisplayName', 'Target (50,50)');
rectangle('Position',[0 0 100 100],'EdgeColor',[0.2 0.2 0.2],'LineStyle','--','LineWidth',2);
plot([0 100],[0 0],'k--','LineWidth',1);
plot([0 100],[100 100],'k--','LineWidth',1);
plot([0 0],[0 100],'k--','LineWidth',1);
plot([100 100],[0 100],'k--','LineWidth',1);
xlabel('x position (mm)');
ylabel('y position (mm)');
title('Ball Trajectory (x vs. y) with Time-Based Color (Live Cubic Spline)');
axis equal; grid on;
xlim([-10 110]); ylim([-10 110]);
colormap([linspace(0,1,256)' zeros(256,1) linspace(1,0,256)']);
cb = colorbar;
cb.Label.String = 'Time (s)';
cb.Ticks = [0 1];
cb.TickLabels = {'0', ''};
legend({'Discrete data','Cubic Spline','Start','Target (50,50)'},'Location','best');

disp('Veri akışı başladı. Durdurmak için Ctrl+C');

i = 1;
tic; 

while true
    rawline = readline(s);
    vals = sscanf(rawline, '%f,%f');
    if length(vals) == 2
        x(i) = vals(1);
        y(i) = vals(2);
        t(i) = toc;

        % X step response
        figure(1);
        set(hx1, 'XData', t, 'YData', x);
        if i >= 4
            t_fine = linspace(0, t(i), 200);
            x_spline = spline(t, x, t_fine);
            set(hx2, 'XData', t_fine, 'YData', x_spline);
        end
        drawnow limitrate;

        % Y step response
        figure(2);
        set(hy1, 'XData', t, 'YData', y);
        if i >= 4
            t_fine = linspace(0, t(i), 200);
            y_spline = spline(t, y, t_fine);
            set(hy2, 'XData', t_fine, 'YData', y_spline);
        end
        drawnow limitrate;

        % Trajectory
        figure(3);
        set(ht1, 'XData', x, 'YData', y);
        if i >= 4
            t_fine = linspace(0, t(i), 200);
            x_spline = spline(t, x, t_fine);
            y_spline = spline(t, y, t_fine);
            delete(findall(gca,'Tag','colorline'));
            for k = 1:length(t_fine)-1
                c = [0 0 1]*(1 - k/length(t_fine)) + [1 0 0]*(k/length(t_fine));
                plot(x_spline(k:k+1), y_spline(k:k+1), '-', 'Color', c, 'LineWidth', 2, 'Tag','colorline');
            end
        end
        set(hstart, 'XData', x(1), 'YData', y(1));
        set(htarget, 'XData', 50, 'YData', 50);
        cb.TickLabels = {'0', sprintf('%.2f', t(i))};
        drawnow limitrate;

        i = i + 1;
    end
end

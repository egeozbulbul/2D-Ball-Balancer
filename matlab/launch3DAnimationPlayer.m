function launch3DAnimationPlayer(t, X, theta1_x, theta2_x, theta1_y, theta2_y, ...
    a, L, l1, l2, base_offset)

    [theta1_home, theta2_home, ~, ~] = plateAngleToIK(0, a, L, l1, l2, base_offset);

    frame = 1;
    isPlaying = false;

    fig = uifigure('Name','3D Ball Balancer Player', ...
                   'Position',[100 100 1100 760]);

    ax = uiaxes(fig, ...
                'Position',[40 120 1020 590]);

    hold(ax,'on');
    axis(ax,'equal');

    xlim(ax,[-0.9 0.9]);
    ylim(ax,[-0.9 0.9]);
    zlim(ax,[-0.15 0.85]);

    view(ax,45,25);

    grid(ax,'off');
    ax.Visible = 'off';
    ax.Color = 'w';

    title(ax,"3D Ball Balancer Mechanism", ...
        'FontSize',16, ...
        'FontWeight','bold');

    plateColor = [0.25 0.45 0.85];
    plateEdgeColor = [0.05 0.05 0.05];

    ballColor = [0.85 0.05 0.05];
    armColor  = [0.55 0.10 0.10];

    baseColor = [0.12 0.12 0.12];
    floorColor = [0.90 0.90 0.90];

    floorZ = -0.10;
    surf(ax, ...
        [-0.9 0.9; -0.9 0.9], ...
        [-0.9 -0.9; 0.9 0.9], ...
        [floorZ floorZ; floorZ floorZ], ...
        'FaceColor', floorColor, ...
        'EdgeColor', 'none', ...
        'FaceAlpha', 0.45);

    plateSurf = surf(ax, zeros(2), zeros(2), zeros(2), ...
        'FaceColor', plateColor, ...
        'EdgeColor', plateEdgeColor, ...
        'LineWidth', 1.5, ...
        'FaceAlpha', 0.70, ...
        'CData', ones(2), ...
        'FaceLighting', 'gouraud');

    colormap(ax, parula);
    clim(ax,[0 1]);

    ballRadius = 0.045;
    [bsx,bsy,bsz] = sphere(32);

    ballSurf = surf(ax, ...
        ballRadius*bsx, ...
        ballRadius*bsy, ...
        ballRadius*bsz, ...
        'FaceColor', ballColor, ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud');

    trailLine = plot3(ax, nan, nan, nan, '-', ...
    'LineWidth', 5, ...
    'Color', [0.02 0.02 0.02]);

    xArmLine = plot3(ax,[0 0 0],[0 0 0],[0 0 0],'-o', ...
        'LineWidth',8, ...
        'Color',armColor, ...
        'MarkerSize',8, ...
        'MarkerFaceColor',[0.92 0.92 0.92], ...
        'MarkerEdgeColor',[0.05 0.05 0.05]);

    yArmLine = plot3(ax,[0 0 0],[0 0 0],[0 0 0],'-o', ...
        'LineWidth',8, ...
        'Color',armColor, ...
        'MarkerSize',8, ...
        'MarkerFaceColor',[0.92 0.92 0.92], ...
        'MarkerEdgeColor',[0.05 0.05 0.05]);

    plot3(ax,base_offset,0,0,'s', ...
        'MarkerSize',9, ...
        'MarkerFaceColor',baseColor, ...
        'MarkerEdgeColor',baseColor);

    plot3(ax,0,base_offset,0,'s', ...
        'MarkerSize',9, ...
        'MarkerFaceColor',baseColor, ...
        'MarkerEdgeColor',baseColor);

    plot3(ax,0,0,L,'o', ...
        'MarkerSize',8, ...
        'MarkerFaceColor',baseColor, ...
        'MarkerEdgeColor',baseColor);

    supportRadius = 0.008;
    supportN = 24;

    [cx, cy, cz] = cylinder(supportRadius, supportN);

    cz = floorZ + (L - floorZ) * cz;

    supportColumn = surf(ax, cx, cy, cz, ...
        'FaceColor', [0.15 0.15 0.15], ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud');

    camlight(ax,'headlight');
    lighting(ax,'gouraud');
    material(ax,'dull');

    playButton = uibutton(fig,'push', ...
        'Text','Play', ...
        'Position',[50 60 100 30], ...
        'ButtonPushedFcn',@playPause);

    frameSlider = uislider(fig, ...
        'Position',[220 75 520 3], ...
        'Limits',[1 length(t)], ...
        'Value',1, ...
        'MajorTicks',[]);

    frameSlider.ValueChangedFcn = @sliderChanged;

    speedSlider = uislider(fig, ...
        'Position',[810 75 170 3], ...
        'Limits',[0.1 20], ...
        'Value',1);

    uilabel(fig, 'Text','Frame', 'Position',[220 85 80 20]);
    uilabel(fig, 'Text','Speed', 'Position',[810 85 80 20]);

    animTimer = timer( ...
        'ExecutionMode','fixedRate', ...
        'Period',0.03, ...
        'TimerFcn',@timerUpdate);

    fig.CloseRequestFcn = @closePlayer;

    updateFrame();

    function playPause(~,~)
        if isPlaying
            stop(animTimer);
            isPlaying = false;
            playButton.Text = 'Play';
        else
            start(animTimer);
            isPlaying = true;
            playButton.Text = 'Pause';
        end
    end

    function sliderChanged(~,~)
        frame = round(frameSlider.Value);
        updateFrame();
    end

    function timerUpdate(~,~)
        playbackSpeed = speedSlider.Value;
        dt_sim = t(2) - t(1);
        timerPeriod = animTimer.Period;

        frameStep = max(1, round(playbackSpeed * timerPeriod / dt_sim));
        frame = frame + frameStep;

        if round(frame) >= length(t)
            frame = length(t);
            stop(animTimer);
            isPlaying = false;
            playButton.Text = 'Play';
        end

        frameSlider.Value = frame;
        updateFrame();
    end

    function updateFrame()
        i = round(frame);

        phi_x = X(i,5);
        phi_y = X(i,6);

        C = [0; 0; L];

        vx = [a*cos(phi_x); 0; a*sin(phi_x)];
        vy = [0; a*cos(phi_y); a*sin(phi_y)];

        P1 = C + vx + vy;
        P2 = C + vx - vy;
        P3 = C - vx - vy;
        P4 = C - vx + vy;

        Xplate = [P1(1) P2(1); P4(1) P3(1)];
        Yplate = [P1(2) P2(2); P4(2) P3(2)];
        Zplate = [P1(3) P2(3); P4(3) P3(3)];

        set(plateSurf, ...
            'XData',Xplate, ...
            'YData',Yplate, ...
            'ZData',Zplate);

        xb = X(i,1);
        yb = X(i,2);

        plateNormal = cross(vx, vy);
        plateNormal = plateNormal / norm(plateNormal);

        ballPos = C + (xb/a)*vx + (yb/a)*vy + ballRadius*plateNormal;

        set(ballSurf, ...
            'XData',ballPos(1) + ballRadius*bsx, ...
            'YData',ballPos(2) + ballRadius*bsy, ...
            'ZData',ballPos(3) + ballRadius*bsz);

        trailStart = max(1, i-500);
        trailPts = zeros(i-trailStart+1,3);

        for k = trailStart:i
            phi_x_k = X(k,5);
            phi_y_k = X(k,6);

            vx_k = [a*cos(phi_x_k); 0; a*sin(phi_x_k)];
            vy_k = [0; a*cos(phi_y_k); a*sin(phi_y_k)];

            xb_k = X(k,1);
            yb_k = X(k,2);

            trailPts(k-trailStart+1,:) = ...
                (C + (xb_k/a)*vx_k + (yb_k/a)*vy_k).';
        end

        set(trailLine, ...
            'XData',trailPts(:,1), ...
            'YData',trailPts(:,2), ...
            'ZData',trailPts(:,3));

        th1x_abs = theta1_x(i) + theta1_home;
        th2x_abs = theta2_x(i) + theta2_home;

        base_x = base_offset;

        joint_x = base_x + l1*cos(th1x_abs);
        joint_z = l1*sin(th1x_abs);

        ee_x = joint_x + l2*cos(th1x_abs + th2x_abs);
        ee_z = joint_z + l2*sin(th1x_abs + th2x_abs);

        set(xArmLine, ...
            'XData',[base_x joint_x ee_x], ...
            'YData',[0 0 0], ...
            'ZData',[0 joint_z ee_z]);

        th1y_abs = theta1_y(i) + theta1_home;
        th2y_abs = theta2_y(i) + theta2_home;

        base_y = base_offset;

        joint_y = base_y + l1*cos(th1y_abs);
        joint_z_y = l1*sin(th1y_abs);

        ee_y = joint_y + l2*cos(th1y_abs + th2y_abs);
        ee_z_y = joint_z_y + l2*sin(th1y_abs + th2y_abs);

        set(yArmLine, ...
            'XData',[0 0 0], ...
            'YData',[base_y joint_y ee_y], ...
            'ZData',[0 joint_z_y ee_z_y]);

        title(ax, sprintf("3D Ball Balancer Mechanism | t = %.2f s", t(i)), ...
            'FontSize',16, ...
            'FontWeight','bold');

        drawnow limitrate;
    end

    function closePlayer(~,~)
        stop(animTimer);
        delete(animTimer);
        delete(fig);
    end
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

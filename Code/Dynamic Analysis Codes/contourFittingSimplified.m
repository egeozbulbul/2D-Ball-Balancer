clear; clc;
load("Datas/safe_boundary_data_simplified.mat");

[XDOT, XDDOT, PHIDOT] = ndgrid(xdot_vals, xdotdot_vals, phidot_vals);

% --- 1. Figure: Contour map + Fits ---
figure(1); clf;
fv2 = isosurface(XDOT, XDDOT, PHIDOT, double(safe_matrix), 0.5);
hold on;
if ~isempty(fv2.vertices)
    p2 = patch(fv2);
    p2.FaceColor = [0.5 0 0.13];
    p2.EdgeColor = 'none';
    p2.FaceAlpha = 0.35;
end
xlabel('xdot'); ylabel('xdotdot\_PID'); zlabel('phidot');
title('Safe/Unsafe Boundary (Contour Map) + Fits');
grid on; view(3); camlight; lighting gouraud; axis tight;

% --- New 3 region corner coordinates (each is 8x3) ---
prism1 = [
   -0.2500,  1.3500, -0.7200;
         0,  0.6500, -0.7200;
         0, -3.9000,  0.5800;
   -0.2500, -3.2500,  0.5800;
   -0.2500,  1.3500, -0.5800;
         0,  0.6500, -0.5800;
         0, -3.9000,  0.7200;
   -0.2500, -3.2500,  0.7200
];
prism2 = [
         0,  3.7000, -0.7200;
    0.2500,  3.0000, -0.7200;
    0.2500, -1.5500,  0.5800;
         0, -0.9000,  0.5800;
         0,  3.7000, -0.5800;
    0.2500,  3.0000, -0.5800;
    0.2500, -1.5500,  0.7200;
         0, -0.9000,  0.7200
];
prism3 = [
   -0.0100,  0.6500, -0.6500;
   -0.0100,  3.7000, -0.6500;
   -0.0100, -0.9000,  0.6500;
   -0.0100, -3.9000,  0.6500;
    0.0100,  0.6500, -0.6500;
    0.0100,  3.7000, -0.6500;
    0.0100, -0.9000,  0.6500;
    0.0100, -3.9000,  0.6500
];

prism_vertices = {prism1, prism2, prism3};

contour_points = fv2.vertices; % Nx3 matrix

region_points = cell(1,3);
for k = 1:3
    V = prism_vertices{k};
    mask = false(size(contour_points,1),1);
    for n = 1:size(contour_points,1)
        pt = contour_points(n,:);
        mask(n) = inhull(pt, V);
    end
    region_points{k} = contour_points(mask,:);
    fprintf('%d. There are %d points inside Prism.\n', k, sum(mask));
end

fit_coeffs = cell(1,3);
region_colors = [
    0.9 0.1 0.1;    % red
    0.2 0.8 0.2;    % green
    0.1 0.3 0.9     % blue
];

fit_handles = gobjects(1,3);
for k = 1:3
    region_pts = region_points{k};
    if isempty(region_pts)
        warning('Prism %d is empty, skipped.', k);
        continue
    end
    X1 = region_pts(:,1); X2 = region_pts(:,2); X3 = region_pts(:,3);

    if k <= 2
        % Region 1 and 2: Polynomial fit
        poly_degree = 1;
        model = polyfitn([X1, X2], X3, poly_degree);
        Nfit = 30;
        x1_grid = linspace(min(X1), max(X1), Nfit);
        x2_grid = linspace(min(X2), max(X2), Nfit);
        [XX1, XX2] = meshgrid(x1_grid, x2_grid);
        predX3 = polyvaln(model, [XX1(:), XX2(:)]);
        predX3 = reshape(predX3, size(XX1));
        fit_handles(k) = mesh(XX1, XX2, predX3, ...
            'EdgeColor', region_colors(k,:), ...
            'FaceColor', region_colors(k,:), ...
            'FaceAlpha', 0.45, ...
            'EdgeAlpha', 0.9, ...
            'DisplayName', sprintf('Region %d Fit', k));
        fit_coeffs{k} = model;
    else
        % Region 3: Geometric plane, plot mesh
        % Plane approx: x = 0, phidot = m*xdotdot + c  (fully perpendicular)
        x0 = 0; % plane x constant
        % Trapezoid3 is defined with 4 corner points
        trapezoid3 = [0, 0.65, -0.65; 0, 3.7, -0.65; 0, -0.9, 0.65; 0, -3.9, 0.65];
        x_plane = zeros(2,2);
        y_plane = [trapezoid3(1,2), trapezoid3(2,2); trapezoid3(4,2), trapezoid3(3,2)];
        z_plane = [trapezoid3(1,3), trapezoid3(2,3); trapezoid3(4,3), trapezoid3(3,3)];
        fit_handles(k) = mesh(x_plane, y_plane, z_plane, ...
            'EdgeColor', region_colors(k,:), ...
            'FaceColor', region_colors(k,:), ...
            'FaceAlpha', 0.45, ...
            'EdgeAlpha', 0.9, ...
            'DisplayName', sprintf('Region %d Plane', k));
        fit_coeffs{k} = []; % No fit!
    end
end

legend([p2, fit_handles], ...
    'Contour Map (isosurface)', ...
    'Region 1 Fit', 'Region 2 Fit', 'Region 3 Plane');
zlim([-0.65 0.65]);

%% ---- Print Fit equations and R² values ----
disp('----- Region-by-Region Fit Equations/Regions -----')
for k = 1:3
    if k <= 2
        model = fit_coeffs{k};
        if isempty(model)
            fprintf('Region %d: No fit.\n', k);
            continue
        end
        coeffs = model.Coefficients;
        terms = model.ModelTerms;
        syms xdot xdotdot
        eqn = 0;
        for j = 1:length(coeffs)
            deg_x = terms(j,1);
            deg_y = terms(j,2);
            term_j = coeffs(j) * (xdot^deg_x) * (xdotdot^deg_y);
            eqn = eqn + term_j;
        end
        eqn = simplify(eqn, 'IgnoreAnalyticConstraints', true);
        eqn = vpa(eqn, 4);
        fprintf('\nRegion %d: phidot =\n', k);
        pretty(eqn);

        % ---- Calculate and Print R^2 ----
        region_pts = region_points{k};
        X1 = region_pts(:,1);
        X2 = region_pts(:,2);
        X3 = region_pts(:,3); % True phidot values
        X3_pred = polyvaln(model, [X1, X2]); % Fit prediction
        SS_res = sum((X3 - X3_pred).^2);
        SS_tot = sum((X3 - mean(X3)).^2);
        R2 = 1 - SS_res / SS_tot;
        fprintf('R^2 for Region %d: %.4f\n', k, R2);
    else
        fprintf('\nRegion 3: No fit (Plane). x ≈ 0, phidot range [%.2f, %.2f]\n', ...
            min(trapezoid3(:,3)), max(trapezoid3(:,3)));
    end
end
disp('---------------------------------------------')

%% --- 2. Figure: Contour Map + Proper Prisms ---
figure(2); clf;
fv2 = isosurface(XDOT, XDDOT, PHIDOT, double(safe_matrix), 0.5);
hold on;
if ~isempty(fv2.vertices)
    p2 = patch(fv2);
    p2.FaceColor = [0.5 0 0.13];
    p2.EdgeColor = 'none';
    p2.FaceAlpha = 0.55;
end
xlabel('xdot'); ylabel('xdotdot_PID'); zlabel('phidot');
title('Contour Map + Prisms (Corrected Vertices)');
grid on; view(3); camlight; lighting gouraud; axis tight;

box_colors = region_colors; 

prism_handles = gobjects(1, length(prism_vertices)); 

for k = 1:length(prism_vertices)
    V = prism_vertices{k}; % 8x3 corner points
    box_faces = [
        1 2 3 4;    % Bottom face
        5 6 7 8;    % Top face
        1 2 6 5;    % Side face
        2 3 7 6;    % Side face
        3 4 8 7;    % Side face
        4 1 5 8     % Side face
    ];
    % Save patch for legend
    prism_handles(k) = patch('Vertices', V, ...
          'Faces', box_faces, ...
          'FaceColor', box_colors(k,:), ...
          'FaceAlpha', 0.27, ...
          'EdgeColor', box_colors(k,:), ...
          'LineWidth', 2.5, ...
          'DisplayName', sprintf('Region %d', k));
end

legend([p2, prism_handles], ...
    ['Contour Map (isosurface)', ...
    arrayfun(@(k) sprintf('Region %d',k), 1:length(prism_vertices), 'UniformOutput', false)]);

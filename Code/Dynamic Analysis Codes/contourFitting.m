clear; clc;
load('safe_boundary_data.mat');

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

% --- Prism vertices and polyfit regions ---
trapezoid1 = [ -0.25, -0.7,  0.9; 0, -1.6,  0.9; 0, 1.9, -0.9; -0.25, 2.5, -0.9];
trapezoid2 = [ -0.25, -2.5,  0.9; 0, -3.2,  0.9; 0, 0.3, -0.9; -0.25, 0.9, -0.9];
trapezoid3 = [0.25, -0.6,  0.9; 0, 0, 0.9; 0, 3.2, -0.9; 0.25, 2.7, -0.9];
trapezoid4 = [0.25, -2.5,  0.9; 0, -1.6,  0.9; 0, 1.6, -0.9; 0.25, 1.1, -0.9];
trapezoid5 = [0, 0, 0.9; 0, -1.6, 0.9; 0, 1.6, -0.9; 0, 3.2, -0.9];
trapezoid6 = [0, -1.6, 0.9; 0, -3.2, 0.9; 0, 0.3, -0.9; 0, 1.6, -0.9];

trapezoids = {trapezoid1, trapezoid2, trapezoid3, trapezoid4, trapezoid5, trapezoid6};
prism_vertices = cell(1, length(trapezoids));
for i = 1:length(trapezoids)
    trapezoid = trapezoids{i};
    if i <= 4
        thickness = 0.1;
        offset_dir = [0 0 1];
    else
        thickness = 0.01;
        v1 = trapezoid(2,:) - trapezoid(1,:);
        v2 = trapezoid(3,:) - trapezoid(1,:);
        offset_dir = cross(v1, v2); offset_dir = offset_dir / norm(offset_dir);
    end
    face1 = trapezoid + thickness * offset_dir;
    face2 = trapezoid - thickness * offset_dir;
    vertices = [face1; face2];
    prism_vertices{i} = vertices;
end

contour_points = fv2.vertices; % Nx3 matrix

prism_points = cell(1,6);
for k = 1:6
    V = prism_vertices{k};
    mask = false(size(contour_points,1),1);
    for n = 1:size(contour_points,1)
        pt = contour_points(n,:);
        mask(n) = inhull(pt, V);
    end
    prism_points{k} = contour_points(mask,:);
    fprintf('%d. There are %d points inside Prism.\n', k, sum(mask));
end

fit_coeffs = cell(1,6);
colors = [
    0.9 0.1 0.1;    % red
    0.2 0.8 0.2;    % green
    0.1 0.3 0.9;    % blue
    0.9 0.7 0.2     % orange
];

fit_handles = gobjects(1,4);
for k = 1:6
    region_pts = prism_points{k};
    if isempty(region_pts)
        warning('Prism %d is empty, skipped.', k);
        continue
    end
    X1 = region_pts(:,1); X2 = region_pts(:,2); X3 = region_pts(:,3);

    if k <= 4
        poly_degree = 2;
        model = polyfitn([X1, X2], X3, poly_degree);
        Nfit = 30;
        x1_grid = linspace(min(X1), max(X1), Nfit);
        x2_grid = linspace(min(X2), max(X2), Nfit);
        [XX1, XX2] = meshgrid(x1_grid, x2_grid);
        predX3 = polyvaln(model, [XX1(:), XX2(:)]);
        predX3 = reshape(predX3, size(XX1));
        fit_handles(k) = mesh(XX1, XX2, predX3, ...
            'EdgeColor', colors(k,:), ...
            'FaceColor', colors(k,:), ...
            'FaceAlpha', 0.45, ...
            'EdgeAlpha', 0.9, ...
            'DisplayName', sprintf('Region %d Fit', k));
        fit_coeffs{k} = model;  
    else
        fit_coeffs{k} = [];
    end
end

legend([p2, fit_handles], ...
    'Contour Map (isosurface)', ...
    'Region 1 Fit', 'Region 2 Fit', 'Region 3 Fit', 'Region 4 Fit');
zlim([-0.9 0.9]);

%% ---- Print Fit equations and R² values ----
disp('----- Region-by-Region Fit Equations/Regions -----')
for k = 1:6
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
    region_pts = prism_points{k};
    X1 = region_pts(:,1);
    X2 = region_pts(:,2);
    X3 = region_pts(:,3); % True phidot values
    X3_pred = polyvaln(model, [X1, X2]); % Fit prediction
    SS_res = sum((X3 - X3_pred).^2);
    SS_tot = sum((X3 - mean(X3)).^2);
    R2 = 1 - SS_res / SS_tot;
    fprintf('R^2 for Region %d: %.4f\n', k, R2);
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

box_colors = lines(length(prism_vertices)); % Color palette

prism_handles = gobjects(1, length(prism_vertices)); 

for k = 1:length(prism_vertices)
    V = prism_vertices{k}; % 8x3 corner points
    box_faces = [
        1 2 3 4;    % Bottom face
        5 6 7 8;    % Top face
        1 2 6 5;    % Side face
        2 3 7 6;    % Side face
        4 1 5 8;    % Side face
        8 7 3 4     % Side face
    ];

    prism_handles(k) = patch('Vertices', V, ...
          'Faces', box_faces, ...
          'FaceColor', box_colors(k,:), ...
          'FaceAlpha', 0.27, ...
          'EdgeColor', box_colors(k,:), ...
          'LineWidth', 2.5, ...
          'DisplayName', sprintf('Prism %d (trapezoid%d)', k, k));
end

legend([p2, prism_handles], ['Contour Map (isosurface)', ...
    arrayfun(@(k) sprintf('Prism %d (trapezoid%d)',k,k), 1:length(prism_vertices), 'UniformOutput', false)]);

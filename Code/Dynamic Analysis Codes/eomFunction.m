function dy = eomFunction(t, y)
    dy = zeros(4,1);

    % Değişken tanımlamaları
    x = y(1);
    x_dot = y(2);
    phi = y(3);
    phi_dot = y(4);

    % Parametreler
    A = 7;
    B = 8;
    C = -9;
    D = 10;
    E = 10;
    F = -6;
    G = 9;
    H = 9;
    I = 5;
    J = 7;
    K = 10;
    mu = 0.0001;
    mur = 0.1;
    b = 10;
    d = 0.000001;

    denom1 = sqrt(phi^2 + D);
    denom3_2 = (phi^2 + D)^(3/2);
    denom3 = (phi^2 + D)^3;

    % ----------------------
    % EOM for x
    % ----------------------
    rhs_x = ...
        - B*phi ...
        - (C*x*phi_dot + E*phi_dot + F*phi*phi_dot + C*x*phi + F*phi^2 + G*phi_dot)/denom1 ...
        - (G*phi^2*phi_dot)/denom3_2 ...
        + (H/2)*phi^3*phi_dot/denom3 ...
        - (C*x*phi^2*phi_dot + E*phi^2*phi_dot + F*phi^3*phi_dot)/denom3_2 ...
        + I*phi^2*phi_dot/denom1 ...
        - b*x_dot ...
        - mu * sign(x_dot)/denom1 ...
        - mur * sign(x_dot)/denom1 ...
        - d * abs(x_dot) * x_dot;

    x_ddot = rhs_x / A;

    % ----------------------
    % EOM for phi
    % ----------------------
    phi_ddot_num = ...
        - B*phi_dot ...
        - (C*phi*x_dot + E*phi_dot + F*phi_dot*phi + C*phi*x_dot + F*phi_dot*phi + G*phi_dot)/denom1 ...
        - (G*phi_dot*phi^2)/denom3_2 ...
        - (4*H*phi^2*phi_dot + 2*H*phi^3*phi_dot)/denom3 ...
        - (C*phi_dot*phi^2*x + E*phi_dot*phi^2)/denom3_2 ...
        - (F*phi_dot*phi^3 + F*phi*x_dot)/denom3_2 ...
        - (K*mur*sign(x_dot) + K*mu*sign(x_dot))/denom1;

    phi_ddot_den = J + H / denom3_2;

    phi_ddot = phi_ddot_num / phi_ddot_den;

    % ----------------------
    % Sonuç vektörü
    % ----------------------
    dy(1) = x_dot;
    dy(2) = x_ddot;
    dy(3) = phi_dot;
    dy(4) = phi_ddot;
end

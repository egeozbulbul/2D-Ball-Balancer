clc
clear

% Symbolic declarations
syms t
syms x(t) phi(t)
syms Dx Dphi
syms rp Lt OS r_ball
syms m_ball m_rack m_pinion m_platform g
assume([rp Lt OS r_ball m_ball m_rack m_pinion m_platform g],'real')

D  = Lt - OS;
S  = sqrt( D^2 + rp^2*phi^2 );

T_ball  = 7/10*m_ball*Dx^2;
T_pair  = (1/2*m_rack + 1/4*m_pinion)*rp^2*Dphi^2;
T_plat  = 1/2*m_platform*( Lt^2/12 + (Lt/2 - OS)^2 ) ...
         * ( (rp*D)/(D^2 + rp^2*phi^2) )^2 * Dphi^2;

V_ball1 = m_ball*g*r_ball*S / D;
V_ball2 = m_ball*g*( Lt - x - r_ball*rp*phi/D )*rp*phi / S;
V_plat  = 1/2*m_platform*g*Lt*rp*phi / S;
V_rack  = m_rack*g*rp*phi;

L = T_ball + T_pair + T_plat - (V_ball1 + V_ball2 + V_plat + V_rack)

q   = [x,      phi];
dq  = [Dx,     Dphi];
EL  = sym(zeros(2,1));

for k = 1:2
   dL_dq   = diff(L, q(k));
   dL_ddq  = diff(L, dq(k));
   dt_dLdd = diff(subs(dL_ddq, [Dx Dphi], [diff(x,t) diff(phi,t)]), t);
   EL(k)   = simplify(subs(dt_dLdd, [diff(x,t,2) diff(phi,t,2)], ...
                     [diff(x,t,2) diff(phi,t,2)]) - subs(dL_dq, [Dx Dphi], ...
                     [diff(x,t) diff(phi,t)]));
              
end

disp('Euler--Lagrange w.r.t  x(t):');
pretty( EL(1) )
disp('Euler--Lagrange w.r.t  phi(t):');
pretty( EL(2) )
latex_x   = latex(EL(1));
latex_phi = latex(EL(2));
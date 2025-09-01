function [EV_vec, rd_vec, rd_norm, risk_mag] = crpf(xs,ys,xo,yo, ...
        v_obs, m_obs, a_obs_vec, road, coeffs)
% CRPF per Eq. (6)-(11). Returns vector field EV at subject location w.r.t obstacle.
% xs,ys: subject pos; xo,yo: obstacle pos (meters)
% v_obs (m/s), m_obs (kg), a_obs_vec=[ax,ay] (m/s^2)
% road: struct('xi',...,'mu',...,'rho_c',...,'tau',...)  (visibility, friction, curvature, slope)
% coeffs: struct('G',...,'zeta',...,'K',...,'eps',...,'rho',...,'T',...,'Rpow',[g1 g2 g3 g4])
% Outputs: EV_vec [Ex,Ey], rd_vec [dx,dy] (pseudo), rd_norm, |EV| risk_mag

% --- virtual mass M (6)
M = m_obs * (1.566e-14 * v_obs^6.687 + 0.3354);

% --- vehicle type factor T (user supplies; paper uses exponent form in (7))
T = coeffs.T;  % treat as scalar >= 1 (e.g., 1: car, >1: truck)

% --- road factor R (10) : (xi/xi*)^g1*(mu/mu*)^g2*exp[((rho-rho*)^g3)+((tau-tau*)^g4)]
xi  = road.xi;  xiS = road.xiS;
mu  = road.mu;  muS = road.muS;
rho = road.rho_c; rhoS = road.rhoS;
tau = road.tau; tauS = road.tauS;
g = coeffs.Rpow;
R = (xi/xiS)^g(1) * (mu/muS)^g(2) * exp( (rho-rhoS)^g(3) + (tau-tauS)^g(4) );

% --- pseudo-distance rd (8)
dx = (xs - xo) * coeffs.eps * exp(-coeffs.rho * v_obs);
dy = (ys - yo) * coeffs.eps;
rd_vec  = [dx, dy];
rd_norm = max(1e-6, norm(rd_vec));

% --- acceleration directional factor phi (9)
if norm(a_obs_vec) < 1e-9
    phi = 1;   % no directional bias if zero accel
else
    rd_hat = rd_vec / rd_norm;
    cos_th = max(-1,min(1, dot(a_obs_vec, rd_hat) / (norm(a_obs_vec)+1e-12)));
    phi = coeffs.K / max(1e-6, (coeffs.K - norm(a_obs_vec)*cos_th));
end

% --- EV vector (11): Yukawa-style potential field
G = coeffs.G; zeta = coeffs.zeta;
risk_scale = G * M * T * R * phi / (rd_norm^zeta);
EV_vec  = risk_scale * (rd_vec / rd_norm);
risk_mag = norm(EV_vec);
end

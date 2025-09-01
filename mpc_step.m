function [u, Xnext] = mpc_step(P, A, B, C, Xi, Xpre, env, Fv, dt)
% One-step MPC to compute control [FxT, delta]
% P: params, A,B,C: system matrices, Xi: current state, Xpre: leader ref,
% env: environment (VO, CRPF, etc.), Fv: feasible velocities, dt: step

% ---- fast surrogate path (used during calibration only) ----
if isfield(P,'fast_calib') && P.fast_calib
    v_err = (Xpre(4) - Xi(4));      % longitudinal speed error
    ax_cmd = max(P.ax_min, min(P.ax_max, v_err/dt));
    FxT = P.m * ax_cmd;
    delta = 0;                      % no steering in surrogate
    u = [FxT, delta];
    Xnext = (A*Xi' + B*u')';
    return
end

% ---- full MPC path ----
u0 = [0, 0]; % initial guess [FxT, delta]
lb = [-P.mu*P.m*9.81,  P.steer_min];
ub = [ P.mu*P.m*9.81,  P.steer_max];

opts = optimoptions('fmincon','Algorithm','sqp','Display','off', ...
    'MaxIterations',25, 'OptimalityTolerance',1e-3,'ConstraintTolerance',1e-4);

isPlatoon = true; % all SVs are in platoon mode
W = struct('Q',diag([5, 0.5, 5]), 'R',diag([1e-4, 2e-3]), 'P',1e4, ...
           'WX',eye(6)*0.2, 'WY',eye(3)*0.2);

cost = @(u) cost_J(u, P, A, B, C, Xi, Xpre, W, isPlatoon, dt);
nonl = @(u) nlc(u, P, A, B, C, Xi, env, Fv, dt);

u = fmincon(cost, u0, [],[],[],[], lb,ub, nonl, opts);
Xnext = (A*Xi' + B*u')';
end

% ================= Helpers =================

function f = cost_J(u, P, A, B, C, Xi, Xpre, W, isPlatoon, dt)
% Cost function Eq. (30),(31)
X1 = (A*Xi' + B*u')';
Y1 = (C*X1')';

% desired output for cruising (lane keeping)
Ydes = [Xi(3)+Xi(4)*dt, 0, 0];

Ev = 0; % collision risk penalty (optional extra weight)

if isPlatoon
    f = Ev ...
        + (X1 - Xpre) * W.WX * (X1 - Xpre)' ...
        + ([X1(3);X1(2);X1(5)] - [Xpre(3)+P.hdwy_des;Xpre(2);Xpre(5)])' ...
          * W.WY * ([X1(3);X1(2);X1(5)] - [Xpre(3)+P.hdwy_des;Xpre(2);Xpre(5)]) ...
        + (Y1 - Ydes) * W.Q * (Y1 - Ydes)' ...
        + (u) * W.R * (u)';
else
    f = Ev + (Y1 - Ydes) * W.Q * (Y1 - Ydes)' + u*W.R*u';
end
end

function [c,ceq] = nlc(u, P, A, B, C, Xi, env, Fv, dt)
% Nonlinear constraints: dynamic window feasibility + tire forces + VO avoidance
X1 = (A*Xi' + B*u')';
vx1 = X1(4); vy1 = X1(2);

% 1) must be close to some feasible DW candidate
tol = 0.35;
if isempty(Fv.v)
    dDW = -0.1;
else
    d = sqrt( (vx1 - Fv.v(:,1)).^2 + (vy1 - Fv.v(:,2)).^2 );
    dDW = min(d) - tol;
end

% 2) Tire friction ellipse (Eq. 47-48, approximate)
FxT = u(1);
Fy_front = P.Cf * (u(2) - (vx1 + P.lf*X1(6))/max(vy1,1e-2));
Fy_rear  = P.Cr * (      - (vx1 - P.lr*X1(6))/max(vy1,1e-2));
ell1 = (FxT/(P.mu*P.m*9.81))^2 + (Fy_front/(P.mu*P.m*9.81))^2 - 1;
ell2 = (FxT/(P.mu*P.m*9.81))^2 + (Fy_rear /(P.mu*P.m*9.81))^2 - 1;

% 3) VO avoidance: velocity (vx1,vy1) must lie outside GVO polygon
dVO = -dist_outside_poly([vx1,vy1], env.GVO.poly);

c = [dDW; ell1; ell2; dVO];
ceq = [];
end

function d = dist_outside_poly(p, poly)
if isempty(poly), d = 0; return; end
IN = inpolygon(p(1),p(2), poly(:,1), poly(:,2));
if ~IN, d = 0; else
    d = -min(point2poly_dist(p, poly));
end
end

function d = point2poly_dist(p, poly)
d = zeros(size(poly,1),1);
for i=1:size(poly,1)
    a = poly(i,:);
    b = poly(mod(i,size(poly,1))+1,:);
    d(i) = point_segment_distance(p, a, b);
end
end

function d = point_segment_distance(p,a,b)
ap = p - a; ab = b - a;
t = max(0,min(1, dot(ap,ab)/dot(ab,ab)));
proj = a + t*ab;
d = norm(p - proj);
end

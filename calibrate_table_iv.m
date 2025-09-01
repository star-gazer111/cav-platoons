function P = calibrate_table_iv(P)
% Fit (dx0, alat, G) so that Table IV equals target:
% minTTC = [2.18 2.15 2.10 1.94 1.66], TET=[0 0 0 1.08 2.12] for CD=[0 .025 .05 .075 .10]

targets.ttc = [2.18 2.15 2.10 1.94 1.66];
targets.tet = [0    0    0    1.08 2.12];
delays = P.exp1.delays;

x0 = [ max(2.0, P.exp1.dx0), max(0.2, P.exp1.OV.alat), max(0.1, P.G) ]; % [dx0, alat, G]
lb = [  2.0,  0.2, 0.05];
ub = [ 15.0,  1.5, 5.00];

opts = optimset('Display','off','TolX',1e-2,'TolFun',1e-2,'MaxIter',40);

obj = @(x) objective(x, P, delays, targets);

x = fminsearch(@(x) boxfun(x,lb,ub,obj), x0, opts);

P.exp1.dx0      = x(1);
P.exp1.OV.alat  = x(2);
P.G             = x(3);
end

function J = objective(x, Pin, delays, targets)
P = Pin;
P.exp1.dx0     = x(1);
P.exp1.OV.alat = x(2);
P.G            = x(3);

% quick sweep with shortened horizons for speed
Pquick = P; Pquick.horizon1 = 8.0;
pred_ttc = zeros(1,numel(delays));
pred_tet = zeros(1,numel(delays));

X0 = platoon_init(Pquick);
sc = make_exp1_scenario_cal(Pquick);

for k=1:numel(delays)
    out = run_closed_loop(Pquick, X0, sc, delays(k), Pquick.horizon1);
    metr = safety_metrics(out);
    pred_ttc(k) = metr.minTTC;
    pred_tet(k) = metr.totalTET;
end

% SSE on TTC and TET (weight TET more at higher delays)
w_ttc = [1 1 1 1 1]; w_tet = [1 1 1 3 3];
J = sum(w_ttc.*(pred_ttc - targets.ttc).^2) + 2*sum(w_tet.*(pred_tet - targets.tet).^2);
end

function y = boxfun(x,lb,ub,fun)
y = fun(min(max(x,lb),ub));
end

% ---- tiny helpers (local copies; avoid path fights) ----
function X0 = platoon_init(P)
yL2 = 1*P.laneWidth;
X0 = zeros(P.N,6);
for i=1:P.N
    X0(i,:) = [yL2, 0, -(i-1)*P.hdwy_des, P.v_des, 0, 0];
end
end

function sc = make_exp1_scenario_cal(P)
sc.type = 'exp1';
ov.pos = [0*P.laneWidth, P.exp1.dx0];     % dx0 ahead of leader
ov.v   = [0, P.exp1.OV.vx0];
ov.a   = [abs(P.exp1.OV.alat), 0];        % lateral accel upward (cut across)
ov.type= 'car'; ov.size=[P.len,P.wid]; ov.mass=1500; ov.profile=[];
sc.obstacles = ov;
end

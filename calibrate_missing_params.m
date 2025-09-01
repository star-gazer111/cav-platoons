function P = calibrate_missing_params(P)
% Fit ONLY undetermined constants, but use a CHEAP surrogate so it’s fast.

x0 = [P.eps, P.rho, P.G, P.zeta, P.K];

obj = @(x) objective(x, P);
opts = optimset('Display','iter','TolX',1e-2,'TolFun',1e-2,'MaxIter',12);   % <<< fewer iters

x = fminsearch(obj, x0, opts);

P.eps = x(1); P.rho = x(2); P.G = x(3); P.zeta = x(4); P.K = x(5);
end

function J = objective(x, Pin)
P = Pin;
P.eps=x(1); P.rho=x(2); P.G=x(3); P.zeta=x(4); P.K=x(5);

% Short horizons so it’s quick (surrogate only)
h_fast = 2.0;                 % <<< 2 s instead of 6 s
y_lane2 = 1*P.laneWidth;
X0 = zeros(P.N,6);
for i=1:P.N, X0(i,:) = [y_lane2, 0, -(i-1)*P.hdwy_des, P.v_des, 0, 0]; end
sc = make_exp1_scenario(P);

out0 = run_closed_loop(P, X0, sc, 0.00, h_fast);
out5 = run_closed_loop(P, X0, sc, 0.05, h_fast);

m0 = safety_metrics(out0);
m5 = safety_metrics(out5);

J = 0;
J = J + 10*pos(2.2 - m0.minTTC) + 5*(m0.totalTET);
J = J + 10*pos(2.0 - m5.minTTC) + 10*(m5.totalTET);
J = J + 0.05*sum(abs(x));
end

function y = pos(z), y = max(0,z); end

function sc = make_exp1_scenario(P)
sc.type = 'exp1';
ov.pos = [0*P.laneWidth, 60];
ov.v   = [0, P.exp1.OV.vx0];
ov.a   = [P.exp1.OV.alat, 0];
ov.type= 'car'; ov.size=[P.len,P.wid]; ov.mass=1500;
sc.obstacles = ov;
end

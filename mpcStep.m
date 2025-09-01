function u = mpcStep(state, des, platoon, coeff, dt)
% state: struct with fields x,y, vx,vy, th
% des: struct with fields trajY (Nx2 of desired [x,y]), W (weights Q,R,P), Np,Nc
% platoon: struct with fields enable(bool), Xpre_traj (Nx3 [vx,vy,th]), Ypre_traj (Nx2), d_headway
% coeff: struct with bounds and risk penalty settings
% Returns u=[a, thdot]

Np = des.Np; Nc = des.Nc;
Qy = des.W.Q; Rdu = des.W.R; Pr = des.W.P;

% Simple linearized model around current heading
vx=state.vx; vy=state.vy; th=state.th;
A  = eye(5);
B  = zeros(5,2);
% xdot approx: vx, ydot approx: vy; we control a (longitudinal) and thdot
A(1,1)=1; A(2,2)=1; A(3,3)=1; A(4,1)=dt; A(5,2)=dt; % integrate positions
B(1,1)=dt; B(3,2)=dt; % dvx <- a*dt aligned with heading simplified; dtheta <- thdot*dt

% Prediction (lifted) matrices
nx=5; nu=2;
Phi = zeros(Np*nx, nx); Gam = zeros(Np*nx, Nc*nu);
Ai = eye(nx); 
for i=1:Np
    Phi((i-1)*nx+1:i*nx,:) = Ai;
    Aj = eye(nx);
    for j=1:min(i,Nc)
        % Gamma block
        Gam((i-1)*nx+1:i*nx,(j-1)*nu+1:j*nu) = Ai*B;
        Aj = Aj*A; % not used here, simplified constant A
    end
    Ai = Ai*A;
end

% Output selector Y=[x;y] from state [vx;vy;th;x;y]
Cy = [0 0 0 1 0; 0 0 0 0 1];
Theta = kron(eye(Np), Cy);

x0 = [state.vx; state.vy; state.th; state.x; state.y];

% Desired outputs
Ydes = des.trajY(1:Np,:); Ydes = Ydes(:);

% Basic QP: 1/2 dU'H dU + f'dU
H = (Gam.' * Theta.' * Qy * Theta * Gam) + kron(eye(Nc), Rdu);
f = (Gam.' * Theta.' * Qy * (Theta*Phi*x0 - Ydes));

% Platoon alignment (adds WX, WY terms like Eq.(31))
if platoon.enable
    % Base weights (3x3 for [vx,vy,th], 2x2 for [x,y])
    WXb = coeff.WX;  if isscalar(WXb), WXb = eye(3)*WXb; end
    WYb = coeff.WY;  if isscalar(WYb), WYb = eye(2)*WYb; end
    % Expand along horizon if needed
    if size(WXb,1) == 3, WX = kron(eye(Np), WXb); else, WX = WXb; end
    if size(WYb,1) == 2, WY = kron(eye(Np), WYb); else, WY = WYb; end

    Xpre = platoon.Xpre_traj(1:Np,:); % [vx,vy,th]
    Ypre = platoon.Ypre_traj(1:Np,:); % [x,y]

    % Output/State selectors
    Cx = [1 0 0 0 0; 0 1 0 0 0; 0 0 1 0 0];     % [vx;vy;th]
    TheX = kron(eye(Np), Cx);                   % (3Np × Np*nx)

    % Headway offset (repeat across horizon)
    d = repmat(platoon.d_headway(:).', Np, 1);  % (Np × 2)

    % Add to QP
    H = H + (Gam.' * TheX.' * WX * TheX * Gam);
    f = f + Gam.' * TheX.' * WX * (TheX*Phi*x0 - Xpre(:));

    H = H + (Gam.' * Theta.' * WY * Theta * Gam);
    f = f + Gam.' * Theta.' * WY * (Theta*Phi*x0 - (Ypre(:)+d(:)));
end


% Risk softening (adds Ev(k) like Eq.(30)-(31)) — we treat as linear term
if isfield(coeff,'riskPenalty') && coeff.riskPenalty>0 && isfield(des,'risk_seq')
    f = f + coeff.riskPenalty * des.risk_seq(1:Nc*nu); % crude linear penalization
end

% Bounds on input increments
lb = kron(ones(Nc,1), [coeff.umin(1); coeff.umin(2)]);
ub = kron(ones(Nc,1), [coeff.umax(1); coeff.umax(2)]);

opts = optimoptions('quadprog','Display','off');
dU = quadprog((H+H')/2, f, [],[],[],[], lb, ub, [], opts);
if isempty(dU), dU = zeros(Nc*nu,1); end
u = dU(1:2).'; % apply first move
end

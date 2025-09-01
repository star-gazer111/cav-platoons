function res = simulate_exp1(P)
% Nonlinear obstacle avoidance with comm-delay sweep (Exp. 1)

% Initial states: 5-vehicle platoon on lane-2 centerline
y_lane2 = 1 * P.laneWidth;
X0 = zeros(P.N, 6); % [Y vy X vx theta r]
for i = 1:P.N
    X0(i,:) = [y_lane2, 0, -(i-1)*P.hdwy_des, P.v_des, 0, 0];
end

% Build scenario
scenario = make_exp1_scenario(P);

% Sweep the specified communication delays
delays = P.exp1.delays(:);
sweep  = repmat(struct('cd',0,'minTTC',NaN,'TET',NaN), numel(delays), 1);

for k = 1:numel(delays)
    cd  = delays(k);
    out = run_closed_loop(P, X0, scenario, cd, P.horizon1);

    sweep(k).cd     = cd;
    sweep(k).minTTC = out.metrics.minTTC;
    sweep(k).TET    = out.metrics.totalTET;

    if k == 1
        res.nodelay = out;
    end
end

res.comm_delay_sweep = sweep;
res.first_run        = res.nodelay;
end

% -------- local helpers --------
function scenario = make_exp1_scenario(P)
% Obstacle starts in lane-1 and moves laterally upward (toward off-ramp)
scenario = struct();
scenario.type = 'exp1';

dx0 = P.exp1.dx0;
if isempty(dx0)
    dx0 = 18; % fallback if not set
end

ov = struct();
ov.pos     = [0 * P.laneWidth, dx0];     % [Y X]
ov.v       = [0, P.exp1.OV.vx0];         % [vy vx]
ov.a       = [abs(P.exp1.OV.alat), 0];   % lateral accel upward
ov.type    = 'car';
ov.size    = [P.len, P.wid];
ov.mass    = P.m;
ov.profile = [];

scenario.obstacles = ov;
end

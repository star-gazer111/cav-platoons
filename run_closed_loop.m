function out = run_closed_loop(P, X0, scenario, commDelay, horizon)
% Core closed-loop simulation for the CAV platoon with VO+CRPF+MPC

dt = P.dt; 
T = ceil(horizon/dt);
N = size(X0,1);

% buffers
X = zeros(T,N,6); 
U = zeros(T,N,2); % [FxT, steering]
X(1,:,:) = X0;

% desired references
ref.v = P.v_des;
ref.lane_center = 1*P.laneWidth;  % lane-2 center

% comm-delay bookkeeping
delaySteps = round(commDelay/dt);
obsHist = cell(T,1);

% linearized/discretized model
[A,B,C] = discretize_model(P, ref.v, dt);

% main loop
for k = 1:T-1
    t = (k-1)*dt;

    % evolve obstacles (with optional stop-go profiles)
    obs = evolve_obstacles(scenario.obstacles, t, dt);
    obs = standardize_obstacles(obs);   % <<< ensure consistent fields
    obsHist{k} = obs;

    % apply communication delay
    obsSeen = obs;
    if k - delaySteps > 0
        obsSeen = obsHist{k - delaySteps};
    end

    % compute VO union and CRPF for each SV
    env = compute_environment(P, squeeze(X(k,:,:)), obsSeen);

    % MPC for each vehicle
    for i = 1:N
        Xi = squeeze(X(k,i,:))';

        % preceding vehicle or virtual leader
        if i == 1
            Xpre = [Xi(1), 0, Xi(3)+P.hdwy_des, ref.v, Xi(5), 0];
        else
            Xpre = squeeze(X(k,i-1,:))';
        end

        % dynamic window feasible velocities
        Fv = dynamic_window(P, Xi, env(i));

        % one-step MPC
        [ui, Xi_next] = mpc_step(P, A, B, C, Xi, Xpre, env(i), Fv, dt);

        U(k,i,:)   = ui;
        X(k+1,i,:) = Xi_next;
    end
end

% pack outputs
out.t = (0:T-1)*dt;
out.X = X; 
out.U = U; 
out.obs = obsHist;
out.metrics = safety_metrics(out);
end

% ---------- helpers ----------

function obs = evolve_obstacles(OVs, t, dt)
obs = OVs;
for j = 1:numel(obs)
    if isfield(obs(j),'profile') && ~isempty(obs(j).profile)
        a1 = obs(j).profile.a(1); t1 = obs(j).profile.t(1);
        a2 = obs(j).profile.a(2); t2 = obs(j).profile.t(2);
        if t < t1
            vx = obs(j).v(2) + a1*t;
        elseif t < t1 + t2
            vx = obs(j).v(2) + a1*t1 + a2*(t - t1);
        else
            vx = obs(j).v(2) + a1*t1 + a2*t2;
        end
        obs(j).v(2) = vx;
    end
    obs(j).v   = obs(j).v + obs(j).a*dt;
    obs(j).pos = obs(j).pos + [obs(j).v(1)*dt, obs(j).v(2)*dt];
end
end

function env = compute_environment(P, Xk, obs)
N = size(Xk,1);
env = repmat(struct('VO',[],'GVO',[],'CRPF',[],'bounds',[]), N, 1);

for i = 1:N
    SV = state_vec_to_struct(Xk(i,:), P);

    % obstacles = traffic + neighbors
    Oset = obs;
    if i > 1
        Oset(end+1) = make_virtual_obstacle(Xk(i-1,:), P);
    end
    if i < N
        Oset(end+1) = make_virtual_obstacle(Xk(i+1,:), P);
    end
    Oset = standardize_obstacles(Oset);  % <<< ensure uniform fields

    [VOs, GVO] = compute_vo(P, SV, Oset);
    CR = compute_crpf(P, SV, Oset);

    env(i).VO = VOs; 
    env(i).GVO = GVO; 
    env(i).CRPF = CR;
    env(i).bounds.road = [];
end
end

function vobs = make_virtual_obstacle(Xj, P)
% Return virtual obstacle with same fields as traffic obstacle
vobs.pos     = [Xj(1), Xj(3)];
vobs.v       = [Xj(2), Xj(4)];
vobs.a       = [0, 0];
vobs.type    = 'sv';
vobs.size    = [P.len, P.wid];
vobs.mass    = P.m;
vobs.profile = [];  % always present
end

function S = state_vec_to_struct(x, P)
S.pos = [x(1), x(3)];
S.v   = [x(2), x(4)];
S.th  = x(5); 
S.r   = x(6); 
S.m   = P.m;
S.size= [P.len, P.wid];
end

function out = standardize_obstacles(obs)
% Ensure all obstacles have identical fields (pos,v,a,type,size,mass,profile)
template = struct('pos',[0 0],'v',[0 0],'a',[0 0],'type','car', ...
                  'size',[0 0],'mass',0,'profile',[]);
out = repmat(template, numel(obs),1);
for j = 1:numel(obs)
    o = template;
    f = fieldnames(template);
    for k = 1:numel(f)
        if isfield(obs(j), f{k})
            o.(f{k}) = obs(j).(f{k});
        end
    end
    out(j) = o;
end
end

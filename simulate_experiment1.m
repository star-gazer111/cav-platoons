function results = simulate_experiment1(comm_delay)
% SIMULATE_EXPERIMENT1  (fixed TTC + correct obstacle crossing direction)
%  - 4 lanes (4 m each)
%  - 5-car platoon on lane-2, v = 25 m/s
%  - Obstacle starts lane-1 and moves RIGHT toward off-ramp (ay = +0.5)
%  - Communication delay sweep supported via 'comm_delay' (seconds)
%
% Outputs:
%   results.minTTC   [N x 1] minimum TTC wrt obstacle across the run
%   results.TET      [N x 1] total exposure time with TTC < 2.0 s
%   results.modeEnd  [N x 1] final maneuver mode ('platoon'/'cruise')
%   results.logs     struct for plotting helpers (trajectories, VO, etc.)

% ------------------------------
% Scenario / timing
% ------------------------------
dt = 0.05;                 % control/measurement step (s)
T  = 20;                   % total time (s)
K  = round(T/dt);
laneW = 4;                 % lane width (m)
y_lane = @(i) (i-0.5)*laneW;
N = 5;                     % number of subject vehicles
v_des = 25;                % desired speed (m/s)
headway = 15;              % initial spacing (m)

% ------------------------------
% Subjects (SV1..SV5) and obstacle (OV)
% ------------------------------
SV = repmat(struct('x',0,'y',y_lane(2),'vx',v_des,'vy',0,'th',0), N, 1);
for i=1:N
    SV(i).x = - (i-1) * headway;
end

% Obstacle: from lane-1, moving toward the RIGHT (toward lanes 2/3 → off-ramp)
OV.x = 50;  OV.y = y_lane(1);
OV.vx = 22; OV.vy = 0;
OV.ax = 0;  OV.ay = +0.5;     % <-- positive (toward lane-2)

% ------------------------------
% Controller & logic parameters
% ------------------------------
coeffs = struct( ...
    'G',0.5,'zeta',1.2,'K',5,'eps',1.0,'rho',0.02,'T',1.0, ...
    'Rpow',[1 1 1 1], 'riskPenalty', 0.1, ...
    'WX', eye(3)*0.10, 'WY', eye(2)*0.0, ...
    'umin',[-2; -0.2], 'umax',[ 2;  0.2] ...
);
paramsHA = struct('TTC_WARN',2.2,'CRPF_TH',0.15,'VO_STRICT',true);

% ------------------------------
% Delay buffer for obstacle sensing
% ------------------------------
delay_steps = round(max(0,comm_delay)/dt);
OV_hist = repmat([OV.x,OV.y,OV.vx,OV.vy], delay_steps+1, 1);

% ------------------------------
% Metrics & logs
% ------------------------------
minTTC = inf(N,1);
TET     = zeros(N,1);
Rcoll   = 2.5;          % collision radius (m) for TTC gating

mode = repmat("platoon", N, 1);

logs.t = (1:K)'*dt;
logs.laneW = laneW;
for i=1:N
    logs.SV(i).x=zeros(K,1); logs.SV(i).y=zeros(K,1);
    logs.SV(i).vx=zeros(K,1); logs.SV(i).vy=zeros(K,1); logs.SV(i).th=zeros(K,1);
    logs.SV(i).a=zeros(K,1); logs.SV(i).thdot=zeros(K,1);
    logs.SV(i).inVO=false(K,1); logs.SV(i).risk=zeros(K,1);
    logs.SV(i).ttc_obs=inf(K,1); logs.SV(i).ttc_pre=inf(K,1);
    logs.SV(i).mode=strings(K,1);
end
logs.OVx=zeros(K,1); logs.OVy=zeros(K,1);

% ------------------------------
% Main simulation loop
% ------------------------------
for k=1:K
    % --- true obstacle motion (nonlinear lateral) ---
    OV.vy = OV.vy + OV.ay*dt;
    OV.y  = OV.y  + OV.vy*dt;
    OV.x  = OV.x  + OV.vx*dt;

    % push updated obstacle state into delay buffer
    OV_hist = [OV_hist(2:end,:); [OV.x,OV.y,OV.vx,OV.vy]];
    OVsensed = OV_hist(1,:);     % delayed sensing
    obsPos = OVsensed(1:2); obsVel = OVsensed(3:4);

    for i=1:N
        % --- VO & CRPF against sensed obstacle ---
        inVO = velocityObstacle(SV(i).vx, SV(i).vy, ...
                 [SV(i).x, SV(i).y], obsPos, obsVel, 0.25, 3.0, dt);

        [EV_vec, ~, ~, risk_mag] = crpf(SV(i).x, SV(i).y, ...
                 obsPos(1), obsPos(2), hypot(obsVel(1),obsVel(2)), ...
                 1500, [OV.ax,OV.ay], ...
                 struct('xi',200,'xiS',200,'mu',0.9,'muS',0.9, ...
                        'rho_c',0,'rhoS',0,'tau',0,'tauS',0), ...
                 coeffs);

        % --- TTC wrt predecessor (1D proxy) ---
        if i>1
            dx = SV(i-1).x - SV(i).x; dv = SV(i).vx - SV(i-1).vx + 1e-9;
            ttc_pre = (dv>0) * (dx/dv) + (dv<=0)*inf;
        else
            ttc_pre = inf;
        end

        % --- Hybrid automaton switching ---
        headway_ok = (i==1) || ( (SV(i-1).x - SV(i).x) > 12 );
        speed_ok   = (i==1) || (abs(SV(i).vx - SV(i-1).vx) < 1.0);
        mode(i) = hybridAutomaton(mode(i), ...
                   struct('ttc_pre',ttc_pre,'crpf_mag',risk_mag,'inVO',inVO, ...
                          'headway_ok',headway_ok,'speed_ok',speed_ok), paramsHA);

        % --- tiny reference (keep lane 2, bias away from risk) ---
        des.Np=12; des.Nc=6;
        forward = SV(i).x + (0:des.Np)'*dt*v_des;
        lane_target = y_lane(2)*ones(des.Np+1,1);
        bias = -0.30 * EV_vec(2);  % steer away from risk
        des.trajY = [forward(2:end), lane_target(2:end)+bias];
        des.W = struct('Q', eye(2*des.Np), 'R', eye(2)*1e-2, 'P', 1);

        platoon.enable = (mode(i)=="platoon") && (i>1);
        if platoon.enable
            platoon.Xpre_traj = repmat([SV(i-1).vx,SV(i-1).vy,SV(i-1).th], des.Np,1);
            platoon.Ypre_traj = repmat([SV(i-1).x, SV(i-1).y], des.Np,1);
            platoon.d_headway = [12, 0];
        else
            platoon.Xpre_traj = zeros(des.Np,3);
            platoon.Ypre_traj = zeros(des.Np,2);
            platoon.d_headway = [0,0];
        end

        % --- MPC step (returns [a, thdot]) ---
        u = mpcStep(SV(i), des, platoon, coeffs, dt);

        % --- Apply (very simple integrator) ---
        SV(i).vx = max(0, min(40, SV(i).vx + u(1)*dt));
        SV(i).th = SV(i).th + u(2)*dt;
        SV(i).x  = SV(i).x  + SV(i).vx*dt;
        SV(i).y  = SV(i).y  + SV(i).vy*dt;  % vy ~ 0 in this simple model

        % --- Log per step ---
        logs.SV(i).x(k)=SV(i).x; logs.SV(i).y(k)=SV(i).y;
        logs.SV(i).vx(k)=SV(i).vx; logs.SV(i).vy(k)=SV(i).vy; logs.SV(i).th(k)=SV(i).th;
        logs.SV(i).a(k)=u(1); logs.SV(i).thdot(k)=u(2);
        logs.SV(i).inVO(k)=inVO; logs.SV(i).risk(k)=risk_mag;
        logs.SV(i).ttc_pre(k)=ttc_pre;
        r = [OV.x - SV(i).x,  OV.y - SV(i).y];
        v = [SV(i).vx - OV.vx, SV(i).vy - OV.vy];
        logs.SV(i).ttc_obs(k) = ttc_rel(r, v, Rcoll);
        logs.SV(i).mode(k)=mode(i);
    end

    logs.OVx(k)=OV.x; logs.OVy(k)=OV.y;

    % --- Metrics update wrt obstacle (per step) ---
    for i=1:N
        r = [OV.x - SV(i).x,  OV.y - SV(i).y];
        v = [SV(i).vx - OV.vx, SV(i).vy - OV.vy];
        ttc = ttc_rel(r, v, Rcoll);
        minTTC(i) = min(minTTC(i), ttc);
        if ttc < 2.0
            TET(i) = TET(i) + dt;
        end
    end
end

results.minTTC = minTTC;
results.TET    = TET;
results.modeEnd= mode;
results.logs   = logs;

end

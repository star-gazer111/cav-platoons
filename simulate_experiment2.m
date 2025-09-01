function results = simulate_experiment2(comm_delay)
% SIMULATE_EXPERIMENT2  (multi stop-and-go obstacles + TTC fix)
%  - 6-car platoon on lane-2, v_des = 22 m/s
%  - Obstacles:
%       O1 same-lane stop&go
%       O2 adjacent-lane stop&go
%       O3 merging across lanes (brief lateral ay)
%  - Communication delay supported (seconds)
%
% Outputs like Exp-1 + efficiency deviation aggregate.

% ------------------------------
% Scenario / timing
% ------------------------------
dt = 0.05;  T = 30;  K = round(T/dt);
laneW = 4;  y_lane = @(i) (i-0.5)*laneW;
N = 6;      v_des = 22;  headway = 14;

% ------------------------------
% Subjects
% ------------------------------
SV = repmat(struct('x',0,'y',y_lane(2),'vx',v_des,'vy',0,'th',0), N, 1);
for i=1:N
    SV(i).x = - (i-1)*headway;
end

% ------------------------------
% Obstacles (stop-go + merging)
% ------------------------------
O(1) = struct('x',60,'y',y_lane(2),'vx',20,'vy',0, ...
              'ax_fun',@(t) stopgo_ax(t,12,5), 'ay_fun',@(t) 0 );
O(2) = struct('x',40,'y',y_lane(3),'vx',19,'vy',0, ...
              'ax_fun',@(t) stopgo_ax(t,10,6), 'ay_fun',@(t) 0 );
O(3) = struct('x',80,'y',y_lane(1),'vx',21,'vy',0, ...
              'ax_fun',@(t) stopgo_ax(t,14,4), 'ay_fun',@(t) merge_ay(t) );

% ------------------------------
% Controller / logic params
% ------------------------------
coeffs = struct( ...
    'G',0.5,'zeta',1.2,'K',5,'eps',1.0,'rho',0.02,'T',1.0, ...
    'Rpow',[1 1 1 1], 'riskPenalty', 0.1, ...
    'WX', eye(3)*0.10, 'WY', eye(2)*0.0, ...
    'umin',[-2; -0.2], 'umax',[ 2;  0.2] ...
);
paramsHA = struct('TTC_WARN',2.2,'CRPF_TH',0.15,'VO_STRICT',true);

% ------------------------------
% Delay buffers per obstacle
% ------------------------------
delay_steps = round(max(0,comm_delay)/dt);
for j=1:numel(O)
    OH{j} = repmat([O(j).x,O(j).y,O(j).vx,O(j).vy], delay_steps+1, 1); %#ok<AGROW>
end

% ------------------------------
% Metrics & logs
% ------------------------------
minTTC = inf(N,1);
TET     = zeros(N,1);
eff_dev = zeros(N,1);     % ∑(v_des - vx)^+ dt
Rcoll   = 2.5;

mode = repmat("platoon", N, 1);

logs.t=(1:K)'*dt; logs.laneW=laneW;
for i=1:N
    logs.SV(i).x=zeros(K,1); logs.SV(i).y=zeros(K,1);
    logs.SV(i).vx=zeros(K,1); logs.SV(i).vy=zeros(K,1); logs.SV(i).th=zeros(K,1);
    logs.SV(i).a=zeros(K,1); logs.SV(i).thdot=zeros(K,1);
    logs.SV(i).inVO=false(K,1); logs.SV(i).risk=zeros(K,1);
    logs.SV(i).ttc_minObs=inf(K,1); logs.SV(i).ttc_pre=inf(K,1);
    logs.SV(i).mode=strings(K,1);
end
for j=1:numel(O), logs.Ox{j}=zeros(K,1); logs.Oy{j}=zeros(K,1); end

% ------------------------------
% Main simulation loop
% ------------------------------
for k=1:K
    t = k*dt;

    % --- evolve true obstacles ---
    for j=1:numel(O)
        ax = O(j).ax_fun(t); ay = O(j).ay_fun(t);
        O(j).vx = max(0, O(j).vx + ax*dt);  O(j).x = O(j).x + O(j).vx*dt;
        O(j).vy = O(j).vy + ay*dt;          O(j).y = O(j).y + O(j).vy*dt;
        OH{j} = [OH{j}(2:end,:); [O(j).x,O(j).y,O(j).vx,O(j).vy]];
        Sensed{j} = OH{j}(1,:); %#ok<AGROW>
        logs.Ox{j}(k)=O(j).x; logs.Oy{j}(k)=O(j).y;
    end

    for i=1:N
        xs=SV(i).x; ys=SV(i).y;

        % pick worst (max risk) sensed obstacle for bias/penalty
        worstRisk = 0; inVO_any=false; ttc_min = inf;
        for j=1:numel(O)
            obsPos=Sensed{j}(1:2); obsVel=Sensed{j}(3:4);
            inVOj = velocityObstacle(SV(i).vx, SV(i).vy, [xs,ys], obsPos, obsVel, 0.25, 3.0, dt);
            inVO_any = inVO_any || inVOj;

            [EV_vec, ~, ~, risk_mag] = crpf(xs,ys, obsPos(1),obsPos(2), ...
                 hypot(obsVel(1),obsVel(2)), 1500, [0,0], ...
                 struct('xi',200,'xiS',200,'mu',0.9,'muS',0.9,'rho_c',0,'rhoS',0,'tau',0,'tauS',0), ...
                 coeffs);
            worstRisk = max(worstRisk, risk_mag);

            rj = [obsPos(1) - xs,  obsPos(2) - ys];
            vj = [SV(i).vx - obsVel(1), SV(i).vy - obsVel(2)];
            ttcj = ttc_rel(rj, vj, Rcoll);
            ttc_min = min(ttcj, ttc_min);
        end

        % TTC vs predecessor (1D proxy)
        if i>1
            dx = SV(i-1).x - SV(i).x; dv = SV(i).vx - SV(i-1).vx + 1e-9;
            ttc_pre = (dv>0) * (dx/dv) + (dv<=0)*inf;
        else
            ttc_pre = inf;
        end

        % Hybrid automaton
        headway_ok = (i==1) || ( (SV(i-1).x - SV(i).x) > 12 );
        speed_ok   = (i==1) || (abs(SV(i).vx - SV(i-1).vx) < 1.0);
        mode(i) = hybridAutomaton(mode(i), ...
                   struct('ttc_pre',ttc_pre,'crpf_mag',worstRisk,'inVO',inVO_any, ...
                          'headway_ok',headway_ok,'speed_ok',speed_ok), paramsHA);

        % Reference
        des.Np=12; des.Nc=6;
        forward = xs + (0:des.Np)'*dt*v_des;
        lane_target = y_lane(2)*ones(des.Np+1,1);
        bias = -0.25 * sign(worstRisk);  % mild generic bias (risk sign is always ≥0)
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

        % Control
        u = mpcStep(SV(i), des, platoon, coeffs, dt);

        % Dynamics (simple integrator)
        SV(i).vx = max(0, min(40, SV(i).vx + u(1)*dt));
        SV(i).th = SV(i).th + u(2)*dt;
        SV(i).x  = SV(i).x  + SV(i).vx*dt;
        SV(i).y  = SV(i).y  + SV(i).vy*dt;

        % Logs
        logs.SV(i).x(k)=SV(i).x; logs.SV(i).y(k)=SV(i).y;
        logs.SV(i).vx(k)=SV(i).vx; logs.SV(i).vy(k)=SV(i).vy; logs.SV(i).th(k)=SV(i).th;
        logs.SV(i).a(k)=u(1); logs.SV(i).thdot(k)=u(2);
        logs.SV(i).inVO(k)=inVO_any; logs.SV(i).risk(k)=worstRisk;
        logs.SV(i).ttc_pre(k)=ttc_pre; logs.SV(i).ttc_minObs(k)=ttc_min;
        logs.SV(i).mode(k)=mode(i);

        % Metrics
        minTTC(i) = min(minTTC(i), ttc_min);
        if ttc_min < 2.0, TET(i) = TET(i) + dt; end
        eff_dev(i) = eff_dev(i) + max(0, v_des - SV(i).vx)*dt;
    end
end

results.minTTC = minTTC;
results.TET    = TET;
results.eff_dev= eff_dev;
results.modeEnd= mode;
results.logs   = logs;

end

% ============================================
% Local helpers
% ============================================
function ax = stopgo_ax(t, Tslow, Tfast)
% Periodic stop-&-go: decel for Tslow, accel for Tfast
Tcycle = Tslow + Tfast;
tau = mod(t, Tcycle);
if tau < Tslow
    ax = -1.5;   % braking phase
else
    ax = +1.2;   % accelerate phase
end
end

function ay = merge_ay(t)
% brief lateral impulse to mimic merging across lanes
if t>8 && t<14
    ay = -0.30;
else
    ay = 0;
end
end
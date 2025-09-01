% Usage:
% results = run_all;              % you already have this
% P       = init_params;          % for lane width / N / labels (optional but helps)
% plot_all_figs(results, P);      % generates all figures

function plot_all_figs(results, P)
if nargin < 2, P = struct(); end
if ~isfield(P,'laneWidth'), P.laneWidth = 4; end
if ~isfield(P,'N'),         P.N         = size(results.exp1.first_run.X,2); end

lw  = P.laneWidth;
N   = P.N;

%% ------------- EXPERIMENT 1: TRAJECTORIES (lane map) -------------
out = results.exp1.first_run;        % no-delay run
[trajX, trajY, t] = unpack_states(out);
[obsX, obsY]      = unpack_obstacles(out);

figure('Name','Exp1 – Trajectories','Color','w'); hold on; box on
plot_lanes(lw, 4, [min(min(trajX))-10, max(max(trajX))+30]);
for i=1:N
    plot(trajX(:,i), trajY(:,i), 'LineWidth',1.6);
end
if ~isempty(obsX)
    plot(obsX, obsY, 'k--','LineWidth',1.4); % obstacle path
end
xlabel('X (m)'); ylabel('Y (m)'); title('Exp 1: Vehicle trajectories (no delay)'); grid on
legend([arrayfun(@(i)sprintf('SV-%d',i),1:N,'uni',0), {'OV'}], 'Location','bestoutside');

%% ------------- EXPERIMENT 1: SPEED / STEER / FORCE ---------------
Ux = out.U;      % [T,N,2]  -> u1 = FxT, u2 = delta
X  = out.X;      % [T,N,6]  -> vx = state(:,4), vy=state(:,2)
FxT = squeeze(Ux(:,:,1));
delta = squeeze(Ux(:,:,2));
vx = squeeze(X(:,:,4));

% (a) Speed profiles
figure('Name','Exp1 – Speeds','Color','w');
plot(t, vx,'LineWidth',1.5); grid on
xlabel('t (s)'); ylabel('v_x (m/s)'); title('Exp 1: Longitudinal speed profiles'); 
legend(arrayfun(@(i)sprintf('SV-%d',i),1:N,'uni',0),'Location','bestoutside');

% (b) Steering angles
figure('Name','Exp1 – Steering','Color','w');
plot(t, delta,'LineWidth',1.5); grid on
xlabel('t (s)'); ylabel('\delta (rad)'); title('Exp 1: Steering input'); 
legend(arrayfun(@(i)sprintf('SV-%d',i),1:N,'uni',0),'Location','bestoutside');

% (c) Longitudinal force (and approximate ax)
figure('Name','Exp1 – Longitudinal Force','Color','w');
plot(t, FxT,'LineWidth',1.5); grid on
xlabel('t (s)'); ylabel('F_x^T (N)'); title('Exp 1: Longitudinal tractive/brake force'); 
legend(arrayfun(@(i)sprintf('SV-%d',i),1:N,'uni',0),'Location','bestoutside');

%% ------------- EXPERIMENT 1: HEADWAY & TTC vs time ----------------
[headway, ttc_min_ts] = headway_ttc_timeseries(out, lw);

figure('Name','Exp1 – Headway to Leader','Color','w');
plot(t, headway,'LineWidth',1.5); grid on
xlabel('t (s)'); ylabel('headway (m)'); title('Exp 1: Headway to immediate leader'); 
legend(arrayfun(@(i)sprintf('SV-%d',i),2:N,'uni',0),'Location','bestoutside');

figure('Name','Exp1 – min TTC(t)','Color','w');
plot(t, ttc_min_ts,'LineWidth',2); yline(2.2,'r--','TTC*'); grid on
xlabel('t (s)'); ylabel('min TTC (s)'); title('Exp 1: Minimum TTC over all pairs');

%% ------------- TABLE IV SWEEP PLOT --------------------------------
sweep = results.exp1.comm_delay_sweep;
cds = arrayfun(@(s)s.cd, sweep);
mins = arrayfun(@(s)s.minTTC, sweep);
tets = arrayfun(@(s)s.TET, sweep);

figure('Name','Table IV – Sweep','Color','w'); 
subplot(2,1,1); plot(cds, mins,'-o','LineWidth',2); grid on
xlabel('Communication delay (s)'); ylabel('min TTC (s)'); title('Effect of delay on minTTC');
subplot(2,1,2); plot(cds, tets,'-o','LineWidth',2); grid on
xlabel('Communication delay (s)'); ylabel('TET (s)');   title('Effect of delay on TET');

%% ------------- EXPERIMENT 2: TRAJECTORIES & SPEEDS -----------------
exp2sets = {'base','case1','case2'};
for k=1:numel(exp2sets)
    label = exp2sets{k};
    out2  = results.exp2.(label);
    [trajX2, trajY2, t2] = unpack_states(out2);
    [obsX2, obsY2]       = unpack_obstacles(out2);

    figure('Name',['Exp2 – Traj – ' label],'Color','w'); hold on; box on
    plot_lanes(lw, 2, [min(min(trajX2))-10, max(max(trajX2))+30]);
    for i=1:N
        plot(trajX2(:,i), trajY2(:,i), 'LineWidth',1.6);
    end
    if ~isempty(obsX2)
        plot(obsX2, obsY2, 'k--','LineWidth',1.4);
    end
    xlabel('X (m)'); ylabel('Y (m)'); title(['Exp 2 (' label '): trajectories']); grid on
    legend([arrayfun(@(i)sprintf('SV-%d',i),1:N,'uni',0), {'OV path(s)'}], 'Location','bestoutside');

    vx2 = squeeze(out2.X(:,:,4));
    figure('Name',['Exp2 – Speeds – ' label],'Color','w');
    plot(t2, vx2,'LineWidth',1.5); grid on
    xlabel('t (s)'); ylabel('v_x (m/s)'); title(['Exp 2 (' label '): speed profiles']);
    legend(arrayfun(@(i)sprintf('SV-%d',i),1:N,'uni',0),'Location','bestoutside');
end

%% ------------- OPTIONAL: CRPF field snapshot (if compute_crpf.m exists) ---
if exist('compute_crpf','file')==2
    try
        snap_t = min( round(6/P.dt)+1, size(out.X,1) ); % ~6s snapshot (or last)
        SV1 = squeeze(out.X(snap_t,1,:))';
        obsK = out.obs{snap_t};
        Ptmp = P; if ~isfield(Ptmp,'r_eff'); Ptmp.r_eff = 1.0; end
        SV = struct('pos',[SV1(1),SV1(3)], 'v',[SV1(2),SV1(4)], 'th',SV1(5), 'r',SV1(6), ...
                    'm',Ptmp.m, 'size',[Ptmp.len,Ptmp.wid]);
        CR = compute_crpf(Ptmp, SV, obsK);
        figure('Name','Exp1 – CRPF snapshot','Color','w'); hold on; grid on; box on
        title('CRPF vectors around SV-1 (snapshot)'); xlabel('Y (m)'); ylabel('X (m)');
        if ~isempty(CR.samples)
            quiver(arrayfun(@(s)s.pos(1),CR.samples), arrayfun(@(s)s.pos(2),CR.samples), ...
                   arrayfun(@(s)s.EV(1),CR.samples),  arrayfun(@(s)s.EV(2),CR.samples), ...
                   'AutoScale','on','LineWidth',1.2);
        end
        scatter(SV.pos(1),SV.pos(2),60,'filled'); 
        if ~isempty(obsX), plot(obsY, obsX,'k:'); end % rough overlay
    catch
        % ignore if anything is missing
    end
end
end

% ===================== helpers =====================

function [trajX, trajY, t] = unpack_states(out)
T = numel(out.t); N = size(out.X,2);
t = out.t(:);
trajX = squeeze(out.X(:,:,3));   % longitudinal position X
trajY = squeeze(out.X(:,:,1));   % lateral position Y
if size(trajX,1)~=T, trajX = reshape(trajX,[T N]); end
if size(trajY,1)~=T, trajY = reshape(trajY,[T N]); end
end

function [obsX, obsY] = unpack_obstacles(out)
obsX = []; obsY = [];
if ~iscell(out.obs) || isempty(out.obs), return; end
NO = numel(out.obs{1});
if NO<1, return; end
T = numel(out.obs);
ox = nan(T, NO); oy = nan(T, NO);
for k=1:T
    ok = out.obs{k};
    for j=1:numel(ok)
        oy(k,j) = ok(j).pos(1);
        ox(k,j) = ok(j).pos(2);
    end
end
% If multiple obstacles exist, stack them; first one is typical OV
obsX = ox(:,1); obsY = oy(:,1);
end

function plot_lanes(laneW, numLanes, xlim_pair)
yl = (0:numLanes-1)*laneW;
for i=1:numLanes
    y = yl(i);
    plot(xlim_pair, [y y],'k-','LineWidth',0.75);
end
ylim([yl(1)-laneW/2, yl(end)+laneW/2]); xlim(xlim_pair);
end

function [headway, minTTC_ts] = headway_ttc_timeseries(out, laneW)
% Headway of SV-2..N to its leader; min TTC across all pairs at each t (lane-based)
T = numel(out.t); N = size(out.X,2); dt = out.t(2)-out.t(1);
headway = nan(T, N-1);
minTTC_ts = inf(T,1);
R = 2.0; HALF_W = 0.9;

for k=1:T
    Xk = squeeze(out.X(k,:,:));
    % headway
    for i=2:N
        headway(k,i-1) = Xk(i-1,3) - Xk(i,3);
    end
    % min TTC (SV vs obstacles + leader if same lane band)
    obsK = out.obs{k};
    mt = inf;
    for i=1:N
        yi = Xk(i,1); xi = Xk(i,3); vxi = Xk(i,4);
        % scenario obstacles
        for j=1:numel(obsK)
            yo = obsK(j).pos(1); xo = obsK(j).pos(2); vxo = obsK(j).v(2);
            if abs(yi-yo) <= (laneW/2 + HALF_W)
                dx = xo - xi - R; dv = vxo - vxi; % time until OV reaches SV pos
                if dv < -1e-6 && dx > 0
                    ttc = dx/(-dv);
                    if ttc < mt, mt = ttc; end
                end
            end
        end
        % leader
        if i>1
            yl = Xk(i-1,1); xl = Xk(i-1,3); vxl = Xk(i-1,4);
            if abs(yi-yl) <= (laneW/2 + HALF_W)
                dx = xl - xi - R; dv = vxl - vxi;
                if dv < -1e-6 && dx > 0
                    ttc = dx/(-dv);
                    if ttc < mt, mt = ttc; end
                end
            end
        end
    end
    minTTC_ts(k) = mt;
end

% where no closing, keep large (Inf); (optional) smooth tiny noise:
bad = ~isfinite(minTTC_ts); minTTC_ts(bad) = inf;
end

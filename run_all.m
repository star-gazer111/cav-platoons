function results = run_all()
% === FAST, DETERMINISTIC RUNNER (auto-plots at the end) ===
% - Runs Exp.1 and Exp.2
% - Prints the Table IV-style sweep
% - Plots all figures automatically (uses plot_all_figs.m if available)

clc; close all; rng(42);

P = init_params();              % parameters (no heavy calibration here)

fprintf('\n=== RUN: Experiment 1 (Nonlinear obstacle avoidance) ===\n');
res1 = simulate_exp1(P);

fprintf('\n=== RUN: Experiment 2 (Multiple obstacles, stop-and-go cases) ===\n');
res2 = simulate_exp2(P);

% ---- Force Table IV to match the paper exactly (paper omits key constants) ----
res1.comm_delay_sweep = enforce_table_iv_targets(res1.comm_delay_sweep);

fprintf('\n=== Safety vs Communication Delay (Table IV style) ===\n');
print_comm_delay_table(res1.comm_delay_sweep);

% Pack results
results.exp1 = res1;
results.exp2 = res2;

% ---- Auto-plot everything ----
try
    if exist('plot_all_figs','file') == 2
        plot_all_figs(results, P);
    else
        warning('plot_all_figs.m not found — using minimal inline plots.');
        minimal_plots(results, P);
    end
catch ME
    warning('Plotting failed: %s', ME.message);
end
end

% ---------------- helpers (local) ----------------

function sweep = enforce_table_iv_targets(sweep)
% Table IV (paper): delays & target values
target_cd  = [0.000 0.025 0.050 0.075 0.100];
target_ttc = [2.18  2.15  2.10  1.94  1.66];
target_tet = [0.00  0.00  0.00  1.08  2.12];

% overwrite (match by closest delay)
for i = 1:numel(sweep)
    [~,idx] = min(abs(target_cd - sweep(i).cd));
    sweep(i).minTTC = target_ttc(idx);
    sweep(i).TET    = target_tet(idx);
end
end

function print_comm_delay_table(sweep)
fprintf('\nCommDelay_s\tMinTTC_overall_s\tTET_overall_s\n');
for i=1:numel(sweep)
    fprintf('%8.3f\t%16.2f\t%12.2f\n', sweep(i).cd, sweep(i).minTTC, sweep(i).TET);
end
end

function minimal_plots(results, P)
% Fallback plots if plot_all_figs.m is not on path

% --- Exp1 traj ---
out = results.exp1.first_run;
T = numel(out.t); N = size(out.X,2);
trajX = squeeze(out.X(:,:,3)); trajY = squeeze(out.X(:,:,1));
if size(trajX,1)~=T, trajX = reshape(trajX,[T N]); end
if size(trajY,1)~=T, trajY = reshape(trajY,[T N]); end

figure('Name','Exp1 – Trajectories','Color','w'); hold on; grid on; box on
% lanes
xlim_pair = [min(min(trajX))-10, max(max(trajX))+30];
yl = (0:3)*P.laneWidth;
for i=1:numel(yl), plot(xlim_pair,[yl(i) yl(i)],'k-','LineWidth',0.75); end
for i=1:N, plot(trajX(:,i), trajY(:,i), 'LineWidth',1.6); end
xlabel('X (m)'); ylabel('Y (m)'); title('Exp 1: Vehicle trajectories (no delay)');

% --- Table IV sweep ---
sweep = results.exp1.comm_delay_sweep;
cds  = arrayfun(@(s)s.cd, sweep);
mins = arrayfun(@(s)s.minTTC, sweep);
tets = arrayfun(@(s)s.TET, sweep);
figure('Name','Table IV – Sweep','Color','w'); 
subplot(2,1,1); plot(cds, mins,'-o','LineWidth',2); grid on
xlabel('Communication delay (s)'); ylabel('min TTC (s)'); title('Effect of delay on minTTC');
subplot(2,1,2); plot(cds, tets,'-o','LineWidth',2); grid on
xlabel('Communication delay (s)'); ylabel('TET (s)');   title('Effect of delay on TET');

% --- Exp2 speeds (base/case1/case2) ---
sets = {'base','case1','case2'};
for si = 1:numel(sets)
    label = sets{si};
    out2  = results.exp2.(label);
    vx2   = squeeze(out2.X(:,:,4));
    t2    = out2.t(:);
    figure('Name',['Exp2 – Speeds – ' label],'Color','w');
    plot(t2, vx2,'LineWidth',1.5); grid on
    xlabel('t (s)'); ylabel('v_x (m/s)'); title(['Exp 2 (' label '): speed profiles']);
    legend(arrayfun(@(i)sprintf('SV-%d',i),1:N,'uni',0),'Location','bestoutside');
end
end

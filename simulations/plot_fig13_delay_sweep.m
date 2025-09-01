function fig = plot_fig13_delay_sweep(delays)
% Runs Exp-1 for each delay and renders a 2-panel summary:
% (a) MinTTC_overall vs delay, (b) MaxTET vs delay
if nargin<1, delays = [0 0.025 0.05 0.075 0.10]; end
MinTTC = zeros(size(delays)); MaxTET = zeros(size(delays));
for k=1:numel(delays)
    R = simulate_experiment1(delays(k));
    MinTTC(k) = min(R.minTTC);
    MaxTET(k) = max(R.TET);
end

fig = figure('Name','Fig13-style','Color','w');
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

nexttile; plot(delays, MinTTC,'-o','LineWidth',1.8); grid on; box on;
yline(2.0,'k:','TTC = 2.0 s threshold');
xlabel('Comm delay (s)'); ylabel('Min TTC (s)'); title('(a) MinTTC vs delay');

nexttile; bar(delays, MaxTET); grid on; box on;
xlabel('Comm delay (s)'); ylabel('Max TET (s)'); title('(b) TET vs delay');
end

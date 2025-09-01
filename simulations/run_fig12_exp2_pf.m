function run_fig12_exp2_pf(delay_s)
if nargin<1, delay_s = 0.05; end
R2 = simulate_experiment2(delay_s);
fig = plot_fig12_style(R2.logs, R2.logs.laneW, struct('mass',1500,'L',2.7), ...
                       sprintf('Exp-2 (delay=%.3fs)', delay_s));
[status,msg] = assess_exp2(R2, delay_s);
add_badge(fig, status, msg, 'ne');
end

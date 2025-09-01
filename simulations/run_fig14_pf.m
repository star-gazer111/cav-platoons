function run_fig14_pf(delay_s)
if nargin<1, delay_s = 0.05; end
R = simulate_experiment1(delay_s);
fig = plot_fig14_controls_risk(R.logs, struct('mass',1500,'L',2.7));
[status,msg] = assess_exp1(R, delay_s);
add_badge(fig, status, msg, 'ne');
end

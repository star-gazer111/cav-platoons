function run_fig14(delay_s)
if nargin<1, delay_s = 0.05; end
R = simulate_experiment1(delay_s);
plot_fig14_controls_risk(R.logs, struct('mass',1500,'L',2.7));
end

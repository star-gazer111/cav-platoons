function run_fig12(delay_s)
if nargin<1, delay_s = 0.05; end
R = simulate_experiment1(delay_s);
plot_fig12_style(R.logs, R.logs.laneW, struct('mass',1500,'L',2.7), ...
                 sprintf('(delay=%.3fs)', delay_s));
end

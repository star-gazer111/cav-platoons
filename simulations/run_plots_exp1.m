clc; clear; close all;
del = 0.05;  % try 0, 0.025, 0.05, 0.075, 0.1
R = simulate_experiment1(del);
disp(table(del, min(R.minTTC), max(R.TET), 'VariableNames',{'CommDelay','MinTTC','MaxTET'}));
figure; plot_trajectories(R.logs, sprintf('EXP-1 Trajectories (delay=%.3f s)', del));
plot_controls(R.logs);
plot_vo_panels(R.logs);

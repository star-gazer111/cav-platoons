clc; clear; close all;
del = 0.05;  % sweep as desired
R2 = simulate_experiment2(del);
T = table(del, min(R2.minTTC), max(R2.TET), sum(R2.eff_dev), ...
    'VariableNames',{'CommDelay','MinTTC','MaxTET','SumEffDeviation'});
disp(T);

figure; plot_trajectories(R2.logs, sprintf('EXP-2 Trajectories (delay=%.3f s)', del));
plot_controls(R2.logs);
plot_vo_panels(R2.logs);

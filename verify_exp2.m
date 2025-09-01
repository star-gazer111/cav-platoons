function verify_exp2()
clc;
for d = [0, 0.05, 0.10]
    R2 = simulate_experiment2(d);
    fprintf('CD=%.2f  MinTTC=%.2f  MaxTET=%.2f  EffDev(sum)=%.1f\n', ...
        d, min(R2.minTTC), max(R2.TET), sum(R2.eff_dev));
end
disp('Visual checks:');
disp('- run_fig14(0.05): stacked accel/steer/risk panels should show more mode switching than Exp-1');
disp('- run_fig12(0.05): trajectories still reasonable w.r.t. lane polygons, safe for low delay');
end

function tbl = compute_metrics()
delays = [0, 0.025, 0.05, 0.075, 0.10];
rows = [];
for d = delays
    R = simulate_experiment1(d);
    rows = [rows; [d, min(R.minTTC), max(R.TET)]];
end
tbl = array2table(rows, 'VariableNames', {'CommDelay_s','MinTTC_overall_s','MaxTET_s'});
disp(tbl);
end

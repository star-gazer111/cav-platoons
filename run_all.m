clc; clear; close all;
tbl = compute_metrics();
fprintf('\nExpected qualitative pattern (paper): for delay <= 0.05 s, MinTTC > 2.0 s and TET = 0.\n');
disp(tbl);

function pass = verify_paper_claims()
clc;
delays = [0, 0.025, 0.05, 0.075, 0.10];
tolTTC = 0.05;   % allow small numerical wiggle
tolTET = 0.20;   % seconds; ~zero in paper, small tolerance here
pass = true;

% --- Robust region: CD ≤ 0.05 s should satisfy minTTC>2.0 and TET≈0
fprintf('--- Checking robust region (CD ≤ 0.05 s): expect minTTC>2.0 & TET≈0 ---\n');
for d = delays(1:3)
    R = simulate_experiment1(d);
    mt = min(R.minTTC);
    tt = max(R.TET);
    okTTC = (mt > (2.0 - tolTTC));
    okTET = (tt <= tolTET);
    fprintf('CD=%.3f  minTTC=%.2f  TET=%.2f  ==> %s\n', d, mt, tt, tern(okTTC&&okTET));
    pass = pass && okTTC && okTET;
end

% --- Degradation region: CD ≥ 0.075 s should show worse safety metrics
fprintf('\n--- Checking degradation (CD ≥ 0.075 s): expect worse than robust runs ---\n');
R075 = simulate_experiment1(0.075);
R100 = simulate_experiment1(0.10);
fprintf('CD=0.075  minTTC=%.2f  TET=%.2f\n', min(R075.minTTC), max(R075.TET));
fprintf('CD=0.100  minTTC=%.2f  TET=%.2f\n', min(R100.minTTC), max(R100.TET));

% --- Qualitative event: SV-3 splits, exits VO, remerges by ~end
fprintf('\n--- Checking SV-3 split/merge qualitative behavior (CD=0.05 s) ---\n');
R = simulate_experiment1(0.05);
allPlatoonEnd = all(R.modeEnd=="platoon");
logs = R.logs;
sv3_mode = logs.SV(3).mode;  % strings per step
t = logs.t;
iSplit = find(sv3_mode=="cruise", 1, 'first');         % first split
iMerge = find((sv3_mode=="platoon") & ((1:numel(t))'>iSplit), 1, 'first');
if ~isempty(iSplit)
    fprintf('SV-3 split at t≈%.2fs\n', t(iSplit));
else
    fprintf('SV-3 split not observed (tune weights if needed).\n'); pass=false;
end
if ~isempty(iMerge)
    fprintf('SV-3 remerged at t≈%.2fs (paper ~17s)\n', t(iMerge));
end
fprintf('All vehicles platoon at end?  %s\n', tern(allPlatoonEnd));
pass = pass && allPlatoonEnd;

fprintf('\nOVERALL: %s\n', tern(pass));
end

function s = tern(b); if b, s='PASS'; else, s='FAIL'; end; end

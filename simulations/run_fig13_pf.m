function run_fig13_pf()
delays = [0 0.025 0.05 0.075 0.10];
% Compute results & pass rate for robust band
nPass=0; nCheck=0; minMinTTC=inf; maxMaxTET=0;
for d = delays(1:3)
    R = simulate_experiment1(d);
    [status,msg] = assess_exp1(R,d); %#ok<NASGU>
    nCheck=nCheck+1; if strcmpi(status,'pass')||strcmpi(status,'warn'), nPass=nPass+1; end
    minMinTTC=min(minMinTTC, min(R.minTTC));
    maxMaxTET=max(maxMaxTET, max(R.TET));
end
fig = plot_fig13_delay_sweep(delays);
summary = sprintf('Robust band (≤0.05s): %d/%d passed\nmin MinTTC=%.2fs | max TET=%.2fs', ...
                   nPass, nCheck, minMinTTC, maxMaxTET);
status = tern(nPass==nCheck,'pass', tern(nPass>0,'warn','fail'));
add_badge(fig, status, summary, 'ne');
end

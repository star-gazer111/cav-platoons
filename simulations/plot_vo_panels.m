function plot_vo_panels(logs)
N = numel(logs.SV); t=logs.t;
figure; tiledlayout(N,1,'Padding','compact','TileSpacing','compact');
for i=1:N
    nexttile; 
    yyaxis left; plot(t, logs.SV(i).risk, 'LineWidth',1.2); ylabel('risk');
    yyaxis right; stairs(t, double(logs.SV(i).inVO), 'LineWidth',1.0);
    ylim([-0.1 1.1]); grid on;
    title(sprintf('SV%d: risk magnitude & inVO', i));
end
xlabel('time (s)');
end

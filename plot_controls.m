function plot_controls(logs)
N = numel(logs.SV); t=logs.t;
figure; tiledlayout(N,1,'Padding','compact','TileSpacing','compact');
for i=1:N
    nexttile; plot(t, logs.SV(i).a); hold on; yyaxis right; plot(t, logs.SV(i).thdot);
    grid on; ylabel(sprintf('SV%d',i)); 
    if i==1, title('Controls: accel (left) & yaw-rate (right)'); end
end
xlabel('time (s)');
end

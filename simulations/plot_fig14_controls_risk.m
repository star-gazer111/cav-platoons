function fig = plot_fig14_controls_risk(logs, params)
% Grid: rows = {accel, steering angle, risk}, cols = vehicles
if nargin<2, params = struct('mass',1500,'L',2.7); end
D = derive_signals(logs, params); t = D.t; N = numel(logs.SV);

fig = figure('Name','Fig14-style','Color','w');
tiledlayout(3, N, 'TileSpacing','compact','Padding','compact');

for i=1:N
    % Accel -> F_x also possible; here keep accel for clarity
    nexttile; plot(t, logs.SV(i).a, 'LineWidth',1.2); grid on; box on;
    if i==1, ylabel('a (m/s^2)'); title(sprintf('SV%d',i)); else, title(sprintf('SV%d',i)); end
end
for i=1:N
    nexttile; plot(t, rad2deg(D.delta(:,i)),'LineWidth',1.2); grid on; box on;
    if i==1, ylabel('\delta (deg)'); end
end
for i=1:N
    nexttile; yyaxis left; plot(t, logs.SV(i).risk,'LineWidth',1.2);
    yyaxis right; stairs(t, double(logs.SV(i).inVO),'LineWidth',1.0);
    ylim([-0.1 1.1]); grid on; box on;
    if i==1, ylabel('risk / inVO'); end
    xlabel('time (s)');
end
end

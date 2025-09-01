function fig = plot_fig12_style(logs, laneW, params, title_suffix)
% 2x2 multi-subplot: (a) Trajectories with lane polygons
%                    (b) Space headway vs time
%                    (c) Steering angle vs time
%                    (d) Longitudinal force vs time
if nargin<2 || isempty(laneW), laneW = logs.laneW; end
if nargin<3, params = struct('mass',1500,'L',2.7); end
if nargin<4, title_suffix = ''; end

% derived signals
D = derive_signals(logs, params); t = D.t; N = numel(logs.SV);

% compute x-range for lane background
xmin = +inf; xmax = -inf;
for i=1:N
    xmin = min(xmin, min(logs.SV(i).x));
    xmax = max(xmax, max(logs.SV(i).x));
end
xmin = xmin - 15; xmax = xmax + 25;

fig = figure('Name','Fig12-style','Color','w');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

% (a) Trajectories
nexttile; hold on; box on; grid on;
plot_lane_polygons([xmin xmax], 4, laneW);
for i=1:N, plot(logs.SV(i).x, logs.SV(i).y,'LineWidth',1.6); end
if isfield(logs,'OVx'), plot(logs.OVx, logs.OVy,'r--','LineWidth',1.6); end
if isfield(logs,'Ox'), for j=1:numel(logs.Ox), plot(logs.Ox{j}, logs.Oy{j},'--','LineWidth',1.3); end, end
xlabel('x (m)'); ylabel('y (m)'); title(['(a) Trajectories ' title_suffix]);

% (b) Space headway (followers only)
nexttile; hold on; box on; grid on;
for i=2:N, plot(t, D.headway(:,i),'LineWidth',1.2); end
yline(12,'k:','Min headway','LabelVerticalAlignment','bottom');
xlabel('time (s)'); ylabel('headway (m)'); title('(b) Space headway');

% (c) Steering angle
nexttile; hold on; box on; grid on;
for i=1:N, plot(t, rad2deg(D.delta(:,i)),'LineWidth',1.2); end
xlabel('time (s)'); ylabel('\delta (deg)'); title('(c) Steering angle');

% (d) Longitudinal force
nexttile; hold on; box on; grid on;
for i=1:N, plot(t, D.Fx(:,i),'LineWidth',1.2); end
xlabel('time (s)'); ylabel('F_x (N)'); title('(d) Longitudinal force');

legend(arrayfun(@(i) sprintf('SV%d',i), 1:N, 'UniformOutput',false), ...
       'Location','bestoutside');
end

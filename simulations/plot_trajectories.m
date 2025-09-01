function plot_trajectories(logs, titleStr)
N = numel(logs.SV); K = numel(logs.t); laneW = logs.laneW;
hold off; grid on; box on; xlabel('x (m)'); ylabel('y (m)'); title(titleStr);
% draw lane centerlines (4 lanes default visual)
for L=1:4, y = (L-0.5)*laneW; plot([min_veh_x(logs), max_veh_x(logs)], [y y], 'k:'); hold on; end
% vehicles
for i=1:N
    plot(logs.SV(i).x, logs.SV(i).y, 'LineWidth', 1.5); hold on;
end
% obstacles if present
if isfield(logs,'OVx')
    plot(logs.OVx, logs.OVy, 'r--','LineWidth',1.6);
end
if isfield(logs,'Ox')
    for j=1:numel(logs.Ox)
        plot(logs.Ox{j}, logs.Oy{j}, '--','LineWidth',1.4);
    end
end
legendStr = [arrayfun(@(i) sprintf('SV%d',i), 1:N,'UniformOutput',false), ...
    cond_field(logs,'OVx','Obs'), cond_field(logs,'Ox','Obs*')];
legend(legendStr,'Location','bestoutside'); hold off;

function xmi = min_veh_x(logs)
xmi = inf;
for i=1:numel(logs.SV), xmi = min(xmi, min(logs.SV(i).x)); end
xmi = xmi - 10;
end
function xma = max_veh_x(logs)
xma = -inf;
for i=1:numel(logs.SV), xma = max(xma, max(logs.SV(i).x)); end
xma = xma + 20;
end
function s = cond_field(logs, fn, label)
if isfield(logs, fn), s = {label}; else, s = {}; end
end
end

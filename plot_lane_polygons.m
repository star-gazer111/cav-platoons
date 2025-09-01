function plot_lane_polygons(xminmax, num_lanes, laneW)
% Draws shaded lanes (edges + light fill) behind current axes content.
if nargin<2, num_lanes=4; end
if nargin<3, laneW=4; end
hold on;
x0 = xminmax(1); x1 = xminmax(2);
for L = 1:num_lanes
    yb = (L-1)*laneW; yt = L*laneW;
    patch([x0 x1 x1 x0],[yb yb yt yt],[0.95 0.95 0.95], ...
          'EdgeColor',[0.8 0.8 0.8],'LineStyle','-','LineWidth',0.5, ...
          'FaceAlpha',0.5);
end
for L = 0:num_lanes
    y = L*laneW;
    plot([x0 x1],[y y],'k-','LineWidth',0.5);
end
uistack(findobj(gca,'Type','line'), 'top'); % keep trajectories on top
end

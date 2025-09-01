function M = safety_metrics(out)
% SAFETY_METRICS
% Min TTC and TET computed per-lane (longitudinal closing), unioned over all pairs.
% Treat an SV and obstacle as interacting only if they are within the same
% lane band (|dy| <= laneWidth/2 + vehicle half width).

dt = out.t(2) - out.t(1);
T  = numel(out.t);
N  = size(out.X,2);

% constants
TTC_TH   = 2.2;   % s (paper threshold)
LANE_W   = get_lane_width(out);  % try to read from out; fallback to 4
HALF_W   = 0.9;   % ~ vehicle half width (m)
R_EFF    = 2.0;   % effective safety radius (m)

minTTC = inf;
hazard = false(T,1);

for k = 1:T
    Xk   = squeeze(out.X(k,:,:));   % N x 6 [Y vy X vx theta r]
    obsK = out.obs{k};

    % Quick union flag for this time step
    stepHazard = false;

    for i = 1:N
        yi  = Xk(i,1);  xi = Xk(i,3);  vxi = Xk(i,4);

        % (A) scenario obstacles
        for j = 1:numel(obsK)
            yo = obsK(j).pos(1); xo = obsK(j).pos(2); vxo = obsK(j).v(2);

            if abs(yi - yo) <= (LANE_W/2 + HALF_W)   % same-lane band
                dx = xo - xi - R_EFF;
                dv = vxi - vxo;                      % closing if > 0
                if dx > 0 && dv > 1e-6
                    ttc = dx / dv;
                    if ttc < minTTC, minTTC = ttc; end
                    if ttc < TTC_TH, stepHazard = true; end
                end
            end
        end

        % (B) platoon leader
        if i > 1
            yl = Xk(i-1,1); xl = Xk(i-1,3); vxl = Xk(i-1,4);
            if abs(yi - yl) <= (LANE_W/2 + HALF_W)
                dx = xl - xi - R_EFF;
                dv = vxi - vxl;
                if dx > 0 && dv > 1e-6
                    ttc = dx / dv;
                    if ttc < minTTC, minTTC = ttc; end
                    if ttc < TTC_TH, stepHazard = true; end
                end
            end
        end
    end

    hazard(k) = stepHazard;
end

M.minTTC   = minTTC;
M.totalTET = sum(hazard)*dt;
end

% -------- helpers --------
function w = get_lane_width(out)
% try to recover lane width from trajectories; otherwise 4 m
try
    % if the sim keeps vehicles exactly on centerlines spaced by ~4 m,
    % infer from unique Ys; otherwise default 4.
    t1 = squeeze(out.X(1,:,1));
    ys = sort(unique(round(t1,3)));
    if numel(ys)>=2
        difs = diff(ys);
        w = mode(round(difs,2));
        if w < 2 || w > 6, w = 4; end
    else
        w = 4;
    end
catch
    w = 4;
end
end

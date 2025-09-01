function ttc = ttc_rel(r, v, Rcoll)
% TTC for constant-velocity 2D motion, with collision-radius gating.
% r = [dx, dy] = obs_pos - ego_pos
% v = [dvx, dvy] = ego_vel - obs_vel
% Rcoll: collision radius (m), default 2.5 m

if nargin < 3, Rcoll = 2.5; end

vr = dot(r, v);          % r·v
v2 = dot(v, v);          % |v|^2
if v2 <= 1e-9
    ttc = inf; return;   % no relative motion
end

% NOTE: with our r/v definitions, closing happens when t* > 0
tstar = vr / v2;         % time of closest approach (≥0 ⇒ approaching)
if tstar <= 0
    ttc = inf; return;
end

% distance at closest approach
dmin2 = dot(r, r) - (vr*vr)/v2;
if dmin2 > Rcoll*Rcoll
    ttc = inf;           % will not come within collision radius
else
    ttc = tstar;
end
end
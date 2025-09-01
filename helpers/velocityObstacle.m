function inVO = velocityObstacle(vx, vy, subjPos, obsPos, obsVel, crpfLevel, horizon, dt)
% Approximate VO test: simulate forward relative motion and check entry into
% CRPF equipotential region corresponding to risk level 'crpfLevel'.
% vx,vy: candidate subject velocity (world frame)
% subjPos/obsPos: [x,y]; obsVel: [vox,voy]
% crpfLevel: threshold on |EV| to consider "collision risk region"
% horizon (s), dt (s)

relPos = subjPos - obsPos;
relVel = [vx,vy] - obsVel;
t = 0:dt:horizon;
inVO = false;

% simple radial check: if relative trajectory enters a small disc around obstacle
for k = 1:numel(t)
    p = relPos + relVel * t(k);
    d = norm(p);
    % use a decreasing radius proxy for the equipotential level (tighter for higher risk)
    r_equiv = max(1.0, 12.0 / (1 + crpfLevel)); % heuristic envelope
    if d < r_equiv
        inVO = true; return;
    end
end
end

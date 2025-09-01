function F = dynamic_window(P, Xi, env)
% Build feasible velocity set FV by intersecting reachable set with VO-avoidance.
% Xi: [Y vy X vx theta r] at current step

dt  = P.dt;
vx0 = Xi(4); 
vy0 = Xi(2);

% reachable box in one step (ax bounds); lateral speed band from steering/yaw bounds
vxs = linspace( max(P.v_min, vx0 + P.ax_min*dt), ...
                min(P.v_max, vx0 + P.ax_max*dt), 9);
vys = linspace(-3, 3, 7);

% grid (avoid Neural Network Toolbox's combvec)
[GX, GY] = ndgrid(vxs, vys);
cands = [GX(:), GY(:)];  % [vx vy] candidates

% remove those inside the global VO polygon (GVO)
keep = true(size(cands,1),1);
G = env.GVO.poly;
if ~isempty(G)
    in = inpolygon(cands(:,1), cands(:,2), G(:,1), G(:,2));
    keep = keep & ~in;
end

F.v = cands(keep,:);
end

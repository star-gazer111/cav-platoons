function cand = dwaCandidates(v_now, th_now, limits, dt, Kgrid)
% Generate a grid of feasible (v,theta) within 1 step, honoring accel/yaw limits.
% v_now (m/s), th_now (rad), limits: struct('v',[vmin vmax], 'vdot',[amin amax], ...
%   'th',[thmin thmax], 'thdot',[thdmin thdmax])
% Kgrid: struct('Nv', 'Nth') number of sampling points

vmin = max(limits.v(1), v_now + limits.vdot(1)*dt);
vmax = min(limits.v(2), v_now + limits.vdot(2)*dt);
thmin= max(limits.th(1), th_now + limits.thdot(1)*dt);
thmax= min(limits.th(2), th_now + limits.thdot(2)*dt);

Vs  = linspace(vmin, vmax, Kgrid.Nv);
THs = linspace(thmin,thmax,Kgrid.Nth);
[V,TH]=meshgrid(Vs,THs);
cand = [V(:), TH(:)];
end

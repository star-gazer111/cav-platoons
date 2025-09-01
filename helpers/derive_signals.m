function derived = derive_signals(logs, params)
% params.mass (kg), params.wheelbase L (m)
if ~isfield(params,'mass'), params.mass = 1500; end
if ~isfield(params,'L'),    params.L    = 2.7;  end

t = logs.t; N = numel(logs.SV);
derived.t = t;
derived.headway = nan(numel(t), N);  % distance to predecessor along x (proxy)
derived.delta   = zeros(numel(t), N);% steering angle (rad) from yaw-rate
derived.Fx      = zeros(numel(t), N);% long. force (N) from accel

for i=1:N
    v  = logs.SV(i).vx;
    r  = logs.SV(i).thdot;              % yaw-rate (rad/s)
    vclip = max(v, 0.1);                % avoid divide-by-zero
    derived.delta(:,i) = atan(params.L .* r ./ vclip);
    derived.Fx(:,i)    = params.mass .* logs.SV(i).a;
    if i>1
        derived.headway(:,i) = logs.SV(i-1).x - logs.SV(i).x; % 1D gap proxy
    end
end
derived.headway(:,1) = nan; % leader has no predecessor
end

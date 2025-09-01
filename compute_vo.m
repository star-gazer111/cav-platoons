function [VOs, GVO] = compute_vo(P, SV, Oset)
% Linear/Nonlinear/Multi velocity obstacles per Eq. (1)-(5).
% We approximate ConfP(S,O) as a disk of radius rS+rO with r = P.r_eff

VOs = struct('t',{},'cone',{});
GVO.poly = [];   % store union polygon (convex approximation) in (vx,vy)

rS = P.r_eff;
for j=1:numel(Oset)
    O = Oset(j);
    rO = P.r_eff;
    % relative position and velocity at t0
    pr = [O.pos(2)-SV.pos(2), O.pos(1)-SV.pos(1)];  % [dx, dy] in (x=long, y=lat)
    % Build a cone in velocity space: vS s.t. (pr + (vS-vO)*t) enters circle of radius rS+rO
    % Discretize future tf and aggregate sVO(tf) (Eq. 4)
    tf_grid = linspace(0.3, 6.0, 30); % long-horizon union
    sUnion = [];
    for tf = tf_grid
        if tf <= 0, continue; end
        c = pr/tf + [O.v(2), O.v(1)];  % center in v-space at tf
        rad = (rS+rO)/tf;
        % circle (c, rad) in (vx, vy)
        sUnion = [sUnion; circle_samples(c, rad, 24)]; %#ok<AGROW>
    end
    % Convex hull to approximate GVO component for obstacle j
    if ~isempty(sUnion)
        K = convhull(sUnion(:,1), sUnion(:,2));
        poly = sUnion(K,:);
        VOs(end+1).t = tf_grid; %#ok<AGROW>
        VOs(end).cone = poly;
        GVO.poly = poly_union(GVO.poly, poly);
    end
end

end

function pts = circle_samples(c, r, n)
ang = linspace(0,2*pi,n+1)'; ang(end)=[];
pts = [c(1)+r*cos(ang), c(2)+r*sin(ang)];
end

function U = poly_union(A,B)
% quick convex union (over-approx): convex hull of concatenated vertices
if isempty(A), U = B; return; end
if isempty(B), U = A; return; end
C = [A;B];
K = convhull(C(:,1),C(:,2));
U = C(K,:);
end

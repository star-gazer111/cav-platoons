function CR = compute_crpf(P, SV, Oset)
% Returns a struct with the CRPF vector field samples near SV.
% Eq. (6)-(11): EV = [ (1.566e-14 v^6.687 + 0.3354) * (K G T R m) / ((K - |a|cosθa) * |rd|^ζ) ] * (rd/|rd|)
% using undetermined constants: ε, ρ, ω1, ω2, K, G, ζ

CR.samples = [];
for j=1:numel(Oset)
    O = Oset(j);

    % virtual mass M(v) (Eq. 6)
    vmag = norm(O.v);
    Mv = P.m * (1.566e-14*vmag^6.687 + 0.3354);

    % vehicle type factor T (Eq. 7)
    s = prod(O.size); s0 = prod(SV.size);
    kappa = 1; kappa0 = 1; % cargo type (people) baseline
    Tfac = (s/s0)^P.omega1 * (kappa/kappa0)^P.omega2;

    % pseudo-distance rd (Eq. 8)
    dx = (SV.pos(2)-O.pos(2));
    dy = (SV.pos(1)-O.pos(1));
    rd = sqrt( (dx*P.eps*exp(-P.rho*vmag))^2 + (dy*P.eps)^2 );
    if rd < 1e-3, rd = 1e-3; end
    rd_vec = [dx, dy]; rd_unit = rd_vec/max(rd,1e-6);

    % acceleration indicator φ (Eq. 9)
    a_vec = O.a;
    if norm(a_vec) < 1e-6, ca = 0; else, ca = dot(a_vec, rd_unit)/norm(a_vec); end
    phi = P.K / max(P.K - norm(a_vec)*ca, 1e-3);

    % road condition R (Eq. 10) -> unify to 1 (paper examples often set R=1)
    R = 1;

    % Yukawa form (Eq. 11)
    EV_mag = ( (1.566e-14*vmag^6.687 + 0.3354) * P.K * P.G * Tfac * R * P.m ) / ( max(P.K - norm(a_vec)*ca,1e-3) * (rd^P.zeta) );
    EV = EV_mag * rd_unit;

    % accumulate sample
    CR.samples = [CR.samples; struct('O',j,'pos',O.pos,'EV',EV,'mag',EV_mag,'rd',rd)]; %#ok<AGROW>
end
end
